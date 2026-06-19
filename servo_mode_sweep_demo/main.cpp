#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <functional>
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
 * Live mode-switch demo. Brings every axis to OPERATIONAL and sweeps one drive through all three
 * cyclic CiA402 modes back-to-back WITHOUT leaving OP, switching the operating mode live over the
 * PDO (0x6060):
 *
 *   Phase 1  CSP  - target-position sine around the start position,
 *   Phase 2  CSV  - target-velocity sine (starts/ends at zero velocity),
 *   Phase 3  CST  - target-torque sine   (starts/ends at zero torque),
 *
 * each running kCyclesPerPhase full sine cycles. Requires an ENI that maps modes-of-operation
 * (0x6060) plus all three command targets into the PDO (generate with eni_gen, which does this for
 * CiA402 drives). The ENI path, sine shapes, and DC tuning are hardcoded below -- edit the kXxx
 * constants to retarget.
 */
constexpr double kFrequencyHz = 1.0;
constexpr int kCyclesPerPhase = 5;
constexpr double kPhaseSeconds = kCyclesPerPhase / kFrequencyHz;

/** Per-phase sine amplitudes (engineering units of each mode). Conservative defaults. */
constexpr std::int32_t kPositionAmplitudeCounts = 50000;       // CSP: encoder counts.
constexpr std::int32_t kVelocityAmplitudeCountsPerSec = 100000; // CSV: counts/s.
constexpr std::int32_t kTorqueAmplitudePerMille = 50;          // CST: per-mille of rated torque.

/** Time spent commanding the neutral setpoint between phases so the drive settles before the
 *  operating mode changes (avoids a step when the next mode takes over). */
constexpr double kSettleSeconds = 0.5;

constexpr double kTwoPi = 6.283185307179586476925286766559;

using TimePoint = std::chrono::steady_clock::time_point;

static double elapsedS(TimePoint now, TimePoint start) {
  return std::chrono::duration<double>(now - start).count();
}

static std::int32_t clampI32(double v) {
  return static_cast<std::int32_t>(std::max(
      static_cast<double>(std::numeric_limits<std::int32_t>::min()),
      std::min(static_cast<double>(std::numeric_limits<std::int32_t>::max()), v)));
}

/* CSP: cos around hold so the profile starts at the center. */
static std::int32_t cspPositionAt(TimePoint now, TimePoint start, std::int32_t hold) {
  const double wt = kTwoPi * kFrequencyHz * elapsedS(now, start);
  const double amp = static_cast<double>(kPositionAmplitudeCounts);
  return clampI32(hold + 0.5 * amp * std::cos(wt) - 0.5 * amp);
}

/* CSV: sin so it starts/ends at zero velocity. */
static std::int32_t csvVelocityAt(TimePoint now, TimePoint start) {
  const double wt = kTwoPi * kFrequencyHz * elapsedS(now, start);
  return clampI32(static_cast<double>(kVelocityAmplitudeCountsPerSec) * std::sin(wt));
}

/* CST: sin so it starts/ends at zero torque. */
static std::int32_t cstTorqueAt(TimePoint now, TimePoint start) {
  const double wt = kTwoPi * kFrequencyHz * elapsedS(now, start);
  return clampI32(static_cast<double>(kTorqueAmplitudePerMille) * std::sin(wt));
}

/**
 * Run one phase for `duration` seconds, invoking `command(now, start)` every cycle to set the
 * target. Returns false if the run ended early (watchdog / fault / Ctrl-C), true if completed.
 */
bool runPhase(lxmstr::EcNetwork& net, const char* label, double duration,
              const std::function<void(TimePoint, TimePoint)>& command) {
  std::cout << "Executing " << label << " mode\n";
  const auto update_period = std::chrono::nanoseconds(net.cycleTimeNs());
  const auto start = std::chrono::steady_clock::now();
  while (net.isRunning() && !g_interrupted) {
    const auto now = std::chrono::steady_clock::now();
    if (elapsedS(now, start) >= duration) break;

    command(now, start);
    std::this_thread::sleep_for(update_period);
  }
  return net.isRunning() && !g_interrupted;
}

/** Hold the current neutral command for `seconds` so the drive settles before a mode change. */
void settle(lxmstr::EcNetwork& net, double seconds) {
  const auto update_period = std::chrono::nanoseconds(net.cycleTimeNs());
  const auto start = std::chrono::steady_clock::now();
  while (net.isRunning() && !g_interrupted) {
    if (elapsedS(std::chrono::steady_clock::now(), start) >= seconds) break;
    std::this_thread::sleep_for(update_period);
  }
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

  // register a callback to report bus faults if a slave drops out mid-run
  cfg.on_bus_fault = [](const lxmstr::BusFault& fault) {
    std::cerr << "\n[bus-fault] " << fault.description << "\n";
    if (fault.break_slave != 0) {
      std::cerr << "[bus-fault] break at slave " << fault.break_slave << " '"
                << fault.break_slave_name << "' port " << fault.break_port << "\n";
    }
    for (const lxmstr::LostSlave& ls : fault.lost_slaves) {
      std::cerr << "[bus-fault]   lost: slave " << ls.index << " '" << ls.name << "'\n";
    }
  };

  // define an ethercat network
  lxmstr::EcNetwork net(cfg);

  std::cout << "servo mode-sweep demo: iface=" << cfg.bus.ifname << " eni=" << cfg.eni.eni_path
            << "\n";

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

  // iterate over servo drive axes and set them to Cyclic Synchronous Position mode
  for (lxmstr::Axis* ax : axes) {
    ax->setDriveMode(lxmstr::DriveOpMode::Csp);
    ax->configure();  // walk this axis up to OP
  }

  // take the first drive on the network
  lxmstr::Axis* drive = axes.front();

  // start the network
  if (!net.start()) {
    std::cerr << "EcNetwork::start() failed: " << net.lastError() << "\n";
    return 1;
  }

  // Let the RT thread publish a couple of cycles so position() is meaningful.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // get current position of the drive
  const std::int32_t hold = drive->actualPosition();
  const auto update_period = std::chrono::nanoseconds(net.cycleTimeNs());

  std::cout << "Operational. Live mode sweep on axis '" << drive->name() << "':\n"
            << "  CSP position  amplitude = " << kPositionAmplitudeCounts << " counts\n"
            << "  CSV velocity  amplitude = " << kVelocityAmplitudeCountsPerSec << " counts/s\n"
            << "  CST torque    amplitude = " << kTorqueAmplitudePerMille << " per-mille\n"
            << "  " << kCyclesPerPhase << " cycles/phase @ " << kFrequencyHz
            << " Hz -> " << kPhaseSeconds << " s each\n"
            << "  center=" << hold << " sync="
            << (net.syncMode() == lxmstr::SyncMode::DcSync0 ? "DC" : "SM-event")
            << " cycle=" << net.cycleTimeNs() << " ns\n";

  bool completed = true;

  // phase 1: CSP position sine around hold
  drive->setDriveMode(lxmstr::DriveOpMode::Csp);
  drive->moveTo(hold);
  std::this_thread::sleep_for(update_period);
  completed = runPhase(net, "CSP", kPhaseSeconds, [&](TimePoint now, TimePoint start) {
    drive->moveTo(cspPositionAt(now, start, hold));
  });

  // phase 2: CSV velocity sine (starts/ends at zero velocity)
  if (completed) {
    drive->moveTo(hold);
    settle(net, kSettleSeconds);
    drive->setDriveMode(lxmstr::DriveOpMode::Csv);
    drive->moveAtVelocity(0);
    std::this_thread::sleep_for(update_period);
    completed = runPhase(net, "CSV", kPhaseSeconds, [&](TimePoint now, TimePoint start) {
      drive->moveAtVelocity(csvVelocityAt(now, start));
    });
  }

  // phase 3: CST torque sine (starts/ends at zero torque)
  if (completed) {
    drive->moveAtVelocity(0);
    settle(net, kSettleSeconds);
    drive->setDriveMode(lxmstr::DriveOpMode::Cst);
    drive->applyTorque(0);
    std::this_thread::sleep_for(update_period);
    completed = runPhase(net, "CST", kPhaseSeconds, [&](TimePoint now, TimePoint start) {
      drive->applyTorque(cstTorqueAt(now, start));
    });
  }

  const bool stopped_early = !completed;

  // return to a neutral setpoint before tearing down
  if (!stopped_early) {
    drive->applyTorque(0);
    drive->moveAtVelocity(0);
    std::this_thread::sleep_for(update_period);
  }

  // stop the ethercat network, this starts graceful shutdown process and joins the realtime thread.
  net.stop();

  if (stopped_early) {
    std::cerr << '\n';
    net.reportDeviceStatus(std::cerr);
    const std::string reason = net.lastError();
    std::cerr << "\nRun ended early (mode sweep incomplete).\n";
    std::cerr << "  Reason: " << (reason.empty() ? "(none reported)" : reason) << "\n";
    if (drive->isFaulted()) {
      std::cerr << "  Fault:  axis '" << drive->name() << "' faulted (statusword=0x" << std::hex
                << drive->statusword() << std::dec << ").\n";
    }
    return 2;
  }

  std::cout << "\n";
  net.reportDeviceStatus(std::cout);
  std::cout << "Done.\n";
  return 0;
}
