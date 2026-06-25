#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <lxmaster/lxmaster.hpp>

static std::atomic<bool> g_interrupted{false};

static void onSignal(int) { g_interrupted = true; }

static void installSignalHandler() {
  struct sigaction sa{};
  sa.sa_handler = onSignal;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);
}

/**
 * ENI-driven generic CiA402 velocity (CSV) sine demo. Commands a sinusoidal target velocity:
 *   vel(t) = amplitude * sin(2 * pi * frequency * t)   [counts/s]
 * sin(0)=0, so the drive starts and ends at zero velocity — no startup step.
 *
 * Only the ENI path and sine profile are configured below; the EtherCAT interface comes from the
 * LXMASTER_RT_IFACE env, and the cyclic period / sync mode come from the ENI. The ENI must map
 * target velocity 0x60FF and set modes-of-operation 0x6060 to CSV (9). Edit the kXxx constants to
 * retarget.
 */
constexpr std::int32_t kAmplitudeCountsPerSec = 100000;
constexpr double kFrequencyHz = 0.5;
constexpr int kCycles = 5;

constexpr double kTwoPi = 6.283185307179586476925286766559;

static double elapsedS(std::chrono::steady_clock::time_point now,
                       std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(now - start).count();
}

static std::int32_t clampI32(double v) {
  return static_cast<std::int32_t>(std::max(
      static_cast<double>(std::numeric_limits<std::int32_t>::min()),
      std::min(static_cast<double>(std::numeric_limits<std::int32_t>::max()), v)));
}

/* vel(t) = amplitude * sin(ωt); sin(0)=0 so it ramps up smoothly from rest. */
static std::int32_t velocityAt(std::chrono::steady_clock::time_point now,
                               std::chrono::steady_clock::time_point start) {
  const double omega_t = kTwoPi * kFrequencyHz * elapsedS(now, start);
  return clampI32(static_cast<double>(kAmplitudeCountsPerSec) * std::sin(omega_t));
}

int main() {

  // Capture Ctrl+c so we can stop the comms early and stop the trajectory cleanly
  installSignalHandler();

  // Get a default set of Ethercat Network parameters
  lxmaster::NetworkConfig cfg = lxmaster::NetworkConfig::defaults();

  // check to make sure ethercat interface name is set from ENV
  if (cfg.bus.ifname.empty()) {
    std::cerr << "No EtherCAT interface: set LXMASTER_RT_IFACE in /etc/profile.d/lxmaster-config.sh "
                 "(run lxmaster host setup first).\n";
    return 1;
  }

  // set the path for network ENI file (generate by: lxmaster eni gen -h )
  cfg.eni.eni_path = "/home/user/myenifolder/myenifile.xml";

  // define an ethercat network
  lxmaster::EcNetwork net(cfg);

  // prepare the network
  if (!net.prepare()) {
    std::cerr << "EcNetwork::prepare() failed: " << net.lastError() << "\n";
    return 1;
  }

  std::vector<lxmaster::Axis*> axes = net.axes();
  if (axes.empty()) {
    std::cerr << "ENI produced no motion axes to command.\n";
    return 1;
  }

  // iterate over servo drive axes and set them to Cyclic Synchronous Velocity mode
  for (lxmaster::Axis* ax : axes) {
    ax->setDriveMode(lxmaster::DriveOpMode::Csv);
    ax->configure();  // walk this axis up to OP
  }

  // take the first drive on the network
  lxmaster::Axis* drive = axes.front();

  // start the network
  if (!net.start()) {
    std::cerr << "EcNetwork::start() failed: " << net.lastError() << "\n";
    return 1;
  }

  // Let the RT thread publish a couple of cycles so `velocity()` is meaningful.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // set some parameters for sinusoidal trajectory
  const double period_s = 1.0 / kFrequencyHz;
  const double total_s  = period_s * static_cast<double>(kCycles);
  std::cout << "Operational. CSV velocity sine profile:\n"
            << "  amplitude (cnt/s) = " << kAmplitudeCountsPerSec << "\n"
            << "  frequency (Hz)    = " << kFrequencyHz << "\n"
            << "  cycles            = " << kCycles
            << "  -> duration " << total_s << " s\n"
            << "  cycle period      = " << net.cycleTimeNs() << " ns (from ENI)\n";

  // start at zero velocity so the profile does not step away from rest at t=0
  drive->moveAtVelocity(0);

  // we will try to create a trajectory that is in same step as the ethercat network cycletime
  const auto update_period = std::chrono::nanoseconds(net.cycleTimeNs());
  std::this_thread::sleep_for(update_period);
  const auto start    = std::chrono::steady_clock::now();
  const auto deadline = start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    std::chrono::duration<double>(total_s));

  // feed the trajectory points as long as system is not interrupted
  while (net.isRunning() && !g_interrupted) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) break;

    // take a new trajectory point and send it to drive
    drive->moveAtVelocity(velocityAt(now, start));

    std::this_thread::sleep_for(update_period);
  }

  /* Latch the early-exit flag and error reason *before* stop() — stop() joins the cyclic
   * thread which is the one that may have set the watchdog error string; reading it after
   * is still safe, but latching `stopped_early` here makes the intent obvious. */
  const bool stopped_early = !net.isRunning();

  /* Before tearing down, command zero velocity so we don't leave the drive coasting at a
   * random point along the sine — let it decelerate to rest. One cycle period is enough for
   * the RT thread to apply it; avoid a long sleep because `stop()` reads vendor diagnostics
   * while still in OP. */
  if (!stopped_early) {
    drive->moveAtVelocity(0);
    std::this_thread::sleep_for(update_period);
  }

  // stop the ethercat network, this starts graceful shutdown process and joins the realtime thread.
  net.stop();

  if (stopped_early) {
    std::cerr << '\n';
    net.reportDeviceStatus(std::cerr);
    const std::string reason = net.lastError();
    std::cerr << "\nRun ended early (velocity sine profile incomplete).\n";
    if (!reason.empty()) {
      std::cerr << "  Reason: " << reason << "\n";
    } else {
      std::cerr << "  Reason: (none reported)\n";
    }
    if (drive->isFaulted()) {
      std::cerr << "  Fault:  axis '" << drive->name() << "' faulted (statusword=0x" << std::hex
                << drive->statusword() << std::dec
                << "); see the device status and [cia402-fault] blocks above.\n";
    }
    return 2;
  }

  std::cout << "\n";
  net.reportDeviceStatus(std::cout);
  std::cout << "Done.\n";
  return 0;
}
