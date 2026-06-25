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
 * ENI-driven generic CiA402 sine demo (CSP). Commands a sinusoidal target around the drive's
 * actual position on entry:
 *   pos(t) = hold + (amplitude / 2) * cos(2 * pi * frequency * t) - (amplitude / 2)
 *
 * Only the ENI path and sine profile are configured below; the EtherCAT interface comes from the
 * LXMASTER_RT_IFACE env, and the cyclic period / sync mode come from the ENI. Edit the kXxx
 * constants to retarget.
 */
constexpr std::int32_t kAmplitudeCounts = 10000;
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

/* cos(ωt) with no phase offset; scale by amp/2 then subtract amp/2 so cos(0)=1 lands on hold.
 * Trough at half period is hold − amp; each full period returns to hold. */
static std::int32_t positionAt(std::chrono::steady_clock::time_point now,
                               std::chrono::steady_clock::time_point start, std::int32_t hold) {
  const double omega_t = kTwoPi * kFrequencyHz * elapsedS(now, start);
  const double amp = static_cast<double>(kAmplitudeCounts);
  return clampI32(static_cast<double>(hold) + 0.5 * amp * std::cos(omega_t) - 0.5 * amp);
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

  // iterate over servo drive axes and set them to Cyclic Synchronous Position mode
  for (lxmaster::Axis* ax : axes) {
    ax->setDriveMode(lxmaster::DriveOpMode::Csp);
    ax->configure();  // walk this axis up to OP
  }

  // take the first drive on the network
  lxmaster::Axis* drive = axes.front();

  // start the network
  if (!net.start()) {
    std::cerr << "EcNetwork::start() failed: " << net.lastError() << "\n";
    return 1;
  }

  // Let the RT thread publish a couple of cycles so `position()` is non-zero.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // get current position of the drive
  const std::int32_t hold = drive->actualPosition();

  ///= set some parameters for sinusoidal trajectory
  const double period_s = 1.0 / kFrequencyHz;
  const double total_s  = period_s * static_cast<double>(kCycles);
  std::cout << "Operational. Sine profile:\n"
            << "  center (counts)  = " << hold << "\n"
            << "  amplitude        = " << kAmplitudeCounts << "\n"
            << "  frequency (Hz)   = " << kFrequencyHz << "\n"
            << "  cycles           = " << kCycles
            << "  -> duration " << total_s << " s\n"
            << "  cycle period     = " << net.cycleTimeNs() << " ns (from ENI)\n";

  /* Latch the first target at hold so the cosine profile does not step away from the
   * drive's actual position before t=0. */
  drive->moveTo(hold);

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
    drive->moveTo(positionAt(now, start, hold));

    std::this_thread::sleep_for(update_period);
  }

  /* Latch the early-exit flag and error reason *before* stop() — stop() joins the cyclic
   * thread which is the one that may have set the watchdog error string; reading it after
   * is still safe, but latching `stopped_early` here makes the intent obvious. */
  const bool stopped_early = !net.isRunning();

  /* Before tearing down, command the center position so we don't leave the drive parked at
   * a random point along the sine — keeps subsequent runs starting from a consistent spot.
   * One cycle period is enough for the RT thread to apply it; avoid a long sleep here
   * because `stop()` reads vendor diagnostics while still in OP. */
  if (!stopped_early) {
    drive->moveTo(hold);
    std::this_thread::sleep_for(update_period);
  }

  // stop the ethercat network, this starts graceful shutdown process and joins the realtime thread.
  net.stop();

  if (stopped_early) {
    std::cerr << '\n';
    net.reportDeviceStatus(std::cerr);
    const std::string reason = net.lastError();
    std::cerr << "\nRun ended early (sine profile incomplete).\n";
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
