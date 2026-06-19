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

#include <lxmstr/lxmstr.hpp>

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
 * ENI-driven generic CiA402 torque (CST) sine demo. Commands a sinusoidal target torque:
 *   trq(t) = amplitude * sin(2 * pi * frequency * t)   [per-mille of rated torque]
 * sin(0)=0, so the drive starts and ends at zero torque — no startup step.
 *
 * WARNING: in torque mode the drive applies torque directly with no position/velocity loop. An
 * unloaded shaft will accelerate; ensure the axis is safe to spin freely.
 *
 * Only the ENI path and sine profile are configured below; the EtherCAT interface comes from the
 * LXMSTR_RT_IFACE env, and the cyclic period / sync mode come from the ENI. The ENI must map
 * target torque 0x6071 and set modes-of-operation 0x6060 to CST (10). Edit the kXxx constants to
 * retarget.
 */
/* Target torque (0x6071) unit is per-mille of rated torque (1000 = rated). 100 = 10%. */
constexpr std::int32_t kAmplitudePerMille = 100;
constexpr double kFrequencyHz = 0.5;
constexpr int kCycles = 5;

constexpr double kTwoPi = 6.283185307179586476925286766559;

static double elapsedS(std::chrono::steady_clock::time_point now,
                       std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(now - start).count();
}

/* Target torque 0x6071 is INT16, so clamp to that range before commanding. */
static std::int32_t clampI16(double v) {
  return static_cast<std::int32_t>(std::max(
      static_cast<double>(std::numeric_limits<std::int16_t>::min()),
      std::min(static_cast<double>(std::numeric_limits<std::int16_t>::max()), v)));
}

/* trq(t) = amplitude * sin(ωt); sin(0)=0 so it ramps up smoothly from no torque. */
static std::int32_t torqueAt(std::chrono::steady_clock::time_point now,
                             std::chrono::steady_clock::time_point start) {
  const double omega_t = kTwoPi * kFrequencyHz * elapsedS(now, start);
  return clampI16(static_cast<double>(kAmplitudePerMille) * std::sin(omega_t));
}

int main() {

  // Capture Ctrl+c so we can stop the comms early and stop the trajectory cleanly
  installSignalHandler();

  // Get a default set of Ethercat Network parameters
  lxmstr::NetworkConfig cfg = lxmstr::NetworkConfig::defaults();

  // check to make sure ethercat interface name is set from ENV
  if (cfg.bus.ifname.empty()) {
    std::cerr << "No EtherCAT interface: set LXMSTR_RT_IFACE in /etc/profile.d/lxmstr-config.sh "
                 "(run lxmstr host tune first).\n";
    return 1;
  }

  // set the path for network ENI file (generate by: lxmstr eni gen -h )
  cfg.eni.eni_path = "~/myenifolder/myenifile.xml";

  // define an ethercat network
  lxmstr::EcNetwork net(cfg);

  // prepare the network
  if (!net.prepare()) {
    std::cerr << "EcNetwork::prepare() failed: " << net.lastError() << "\n";
    return 1;
  }

  std::vector<lxmstr::Axis*> axes = net.axes();
  if (axes.empty()) {
    std::cerr << "ENI produced no motion axes to command.\n";
    return 1;
  }

  // iterate over servo drive axes and set them to Cyclic Synchronous Torque mode
  for (lxmstr::Axis* ax : axes) {
    ax->setDriveMode(lxmstr::DriveOpMode::Cst);
    ax->configure();  // walk this axis up to OP
  }

  // take the first drive on the network
  lxmstr::Axis* drive = axes.front();

  // start the network
  if (!net.start()) {
    std::cerr << "EcNetwork::start() failed: " << net.lastError() << "\n";
    return 1;
  }

  // Let the RT thread publish a couple of cycles before commanding torque.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // set some parameters for sinusoidal trajectory
  const double period_s = 1.0 / kFrequencyHz;
  const double total_s  = period_s * static_cast<double>(kCycles);
  std::cout << "Operational. CST torque sine profile:\n"
            << "  amplitude (per-mille) = " << kAmplitudePerMille << "\n"
            << "  frequency (Hz)        = " << kFrequencyHz << "\n"
            << "  cycles                = " << kCycles
            << "  -> duration " << total_s << " s\n"
            << "  cycle period          = " << net.cycleTimeNs() << " ns (from ENI)\n";

  // start at zero torque so the profile does not step away from rest at t=0
  drive->applyTorque(0);

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
    drive->applyTorque(torqueAt(now, start));

    std::this_thread::sleep_for(update_period);
  }

  /* Latch the early-exit flag and error reason *before* stop() — stop() joins the cyclic
   * thread which is the one that may have set the watchdog error string; reading it after
   * is still safe, but latching `stopped_early` here makes the intent obvious. */
  const bool stopped_early = !net.isRunning();

  /* Before tearing down, command zero torque so we don't leave the drive applying torque at a
   * random point along the sine. One cycle period is enough for the RT thread to apply it;
   * avoid a long sleep because `stop()` reads vendor diagnostics while still in OP. */
  if (!stopped_early) {
    drive->applyTorque(0);
    std::this_thread::sleep_for(update_period);
  }

  // stop the ethercat network, this starts graceful shutdown process and joins the realtime thread.
  net.stop();

  if (stopped_early) {
    std::cerr << '\n';
    net.reportDeviceStatus(std::cerr);
    const std::string reason = net.lastError();
    std::cerr << "\nRun ended early (torque sine profile incomplete).\n";
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
