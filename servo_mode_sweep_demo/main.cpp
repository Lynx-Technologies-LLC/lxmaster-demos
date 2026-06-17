#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <thread>
#include <vector>

#include <lxmstr/lxmstr.hpp>

#include "apps_common/signal.hpp"

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
 * CiA402 drives). Only the bus location is taken on the command line; the sine shapes and DC tuning
 * are hardcoded below -- edit the kXxx constants to retarget.
 */
namespace servo_mode_sweep_demo {

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

struct Options {
  const char* eni_path = nullptr;
};

void printUsage(const char* exe) {
  std::cerr << "Usage: " << exe << " --eni <file>\n"
            << "\n"
            << "Live CiA402 mode-switch demo. Brings every axis to OPERATIONAL, then sweeps the\n"
            << "first axis through CSP (position) -> CSV (velocity) -> CST (torque), switching the\n"
            << "operating mode live over the PDO without leaving OP. Each phase runs "
            << kCyclesPerPhase << " sine\n"
            << "cycles at " << kFrequencyHz << " Hz. The ENI must map 0x6060 and all command targets\n"
            << "into the PDO (eni_gen does this for CiA402 drives).\n"
            << "\n"
            << "Required:\n"
            << "  --eni <file>     ENI (EtherCATConfig) to load, validate, and run.\n"
            << "\n"
            << "The EtherCAT interface comes from the LXMSTR_RT_IFACE env (set by lxmstr host tune).\n";
}

bool parseCli(int argc, char* argv[], Options& opts) {
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "-h") || !std::strcmp(argv[i], "--help")) {
      printUsage(argv[0]);
      std::exit(0);
    }
    if (!std::strcmp(argv[i], "--eni")) {
      if (i + 1 >= argc) { std::cerr << "Missing value for --eni\n"; return false; }
      opts.eni_path = argv[++i];
      continue;
    }
    std::cerr << "Unknown argument: " << argv[i] << "\n";
    return false;
  }
  if (opts.eni_path == nullptr) { std::cerr << "Missing --eni <file>\n"; printUsage(argv[0]); return false; }
  return true;
}

lxmstr::NetworkConfig makeConfig(const Options& opts) {
  lxmstr::NetworkConfig cfg = lxmstr::NetworkConfig::defaults();

  /* DC-PI tuning, busy-wait window, warmup and OP-entry gate all come from
   * NetworkConfig::defaults(). */
  cfg.eni.eni_path = opts.eni_path;
  return cfg;
}

std::int32_t clampToI32(double v) {
  return static_cast<std::int32_t>(std::max(
      static_cast<double>(std::numeric_limits<std::int32_t>::min()),
      std::min(static_cast<double>(std::numeric_limits<std::int32_t>::max()), v)));
}

/**
 * Run one phase for `duration` seconds, invoking `command(t)` every cycle to set the target.
 * Returns false if the run ended early (watchdog / fault / Ctrl-C), true if the phase completed.
 */
bool runPhase(lxmstr::EcNetwork& net, const char* label, double duration,
              const std::function<void(double)>& command) {
  std::cout << "Executing " << label << " mode\n";
  const auto update_period = std::chrono::nanoseconds(net.cycleTimeNs());
  const auto start = std::chrono::steady_clock::now();
  while (net.isRunning() && !apps_common::interrupted()) {
    const double t =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (t >= duration) break;

    command(t);
    std::this_thread::sleep_for(update_period);
  }
  return net.isRunning() && !apps_common::interrupted();
}

/** Hold the current neutral command for `seconds` so the drive settles before a mode change. */
void settle(lxmstr::EcNetwork& net, double seconds) {
  const auto update_period = std::chrono::nanoseconds(net.cycleTimeNs());
  const auto start = std::chrono::steady_clock::now();
  while (net.isRunning() && !apps_common::interrupted()) {
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() >= seconds) {
      break;
    }
    std::this_thread::sleep_for(update_period);
  }
}

}  // namespace servo_mode_sweep_demo

int main(int argc, char* argv[]) {
  namespace app = servo_mode_sweep_demo;
  app::Options opts;
  if (!app::parseCli(argc, argv, opts)) return 1;

  apps_common::installSignalHandler();

  lxmstr::NetworkConfig cfg = app::makeConfig(opts);
  if (cfg.bus.ifname.empty()) {
    std::cerr << "No EtherCAT interface: set LXMSTR_RT_IFACE in /etc/profile.d/lxmstr-config.sh "
                 "(run lxmstr host tune first).\n";
    return 1;
  }
  /* Demonstrate the structured bus-fault callback: fired once (off the RT thread) if a slave
   * drops out mid-run. The app is free to print or take any action; here we just report it. */
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
  lxmstr::EcNetwork net(cfg);

  std::cout << "servo mode-sweep demo: iface=" << cfg.bus.ifname << " eni=" << opts.eni_path << "\n";

  if (!net.prepare()) {
    std::cerr << "EcNetwork::prepare() failed: " << net.lastError() << "\n";
    return 1;
  }

  /* Bring every axis to OP, starting in CSP. configure() opts the device into the OPERATIONAL
   * bring-up; the live mode switches happen later over the PDO. */
  std::vector<lxmstr::Axis*> axes = net.axes();
  for (lxmstr::Axis* ax : axes) {
    ax->setDriveMode(lxmstr::DriveOpMode::Csp);
    ax->configure();
  }
  if (axes.empty()) {
    std::cerr << "ENI produced no motion axes to command.\n";
    return 1;
  }
  lxmstr::Axis* drive = axes.front();

  if (!net.start()) {
    std::cerr << "EcNetwork::start() failed: " << net.lastError() << "\n";
    return 1;
  }

  /* Let the RT thread publish a couple of cycles so position() is meaningful. */
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const std::int32_t hold = drive->actualPosition();
  const auto update_period = std::chrono::nanoseconds(net.cycleTimeNs());

  std::cout << "Operational. Live mode sweep on axis '" << drive->name() << "':\n"
            << "  CSP position  amplitude = " << app::kPositionAmplitudeCounts << " counts\n"
            << "  CSV velocity  amplitude = " << app::kVelocityAmplitudeCountsPerSec << " counts/s\n"
            << "  CST torque    amplitude = " << app::kTorqueAmplitudePerMille << " per-mille\n"
            << "  " << app::kCyclesPerPhase << " cycles/phase @ " << app::kFrequencyHz
            << " Hz -> " << app::kPhaseSeconds << " s each\n"
            << "  center=" << hold << " sync="
            << (net.syncMode() == lxmstr::SyncMode::DcSync0 ? "DC" : "SM-event")
            << " cycle=" << net.cycleTimeNs() << " ns\n";

  bool completed = true;

  /* ---- Phase 1: CSP position sine around `hold` (cos so it starts at the center). ---- */
  drive->setDriveMode(lxmstr::DriveOpMode::Csp);
  drive->moveTo(hold);
  std::this_thread::sleep_for(update_period);
  completed = app::runPhase(net, "CSP", app::kPhaseSeconds, [&](double t) {
    const double wt = app::kTwoPi * app::kFrequencyHz * t;
    const double amp = static_cast<double>(app::kPositionAmplitudeCounts);
    drive->moveTo(app::clampToI32(hold + 0.5 * amp * std::cos(wt) - 0.5 * amp));
  });

  /* ---- Phase 2: CSV velocity sine (sin so it starts/ends at zero velocity). ---- */
  if (completed) {
    drive->moveTo(hold);
    app::settle(net, app::kSettleSeconds);
    drive->setDriveMode(lxmstr::DriveOpMode::Csv);
    drive->moveAtVelocity(0);
    std::this_thread::sleep_for(update_period);
    completed = app::runPhase(net, "CSV", app::kPhaseSeconds, [&](double t) {
      const double wt = app::kTwoPi * app::kFrequencyHz * t;
      const double amp = static_cast<double>(app::kVelocityAmplitudeCountsPerSec);
      drive->moveAtVelocity(app::clampToI32(amp * std::sin(wt)));
    });
  }

  /* ---- Phase 3: CST torque sine (sin so it starts/ends at zero torque). ---- */
  if (completed) {
    drive->moveAtVelocity(0);
    app::settle(net, app::kSettleSeconds);
    drive->setDriveMode(lxmstr::DriveOpMode::Cst);
    drive->applyTorque(0);
    std::this_thread::sleep_for(update_period);
    completed = app::runPhase(net, "CST", app::kPhaseSeconds, [&](double t) {
      const double wt = app::kTwoPi * app::kFrequencyHz * t;
      const double amp = static_cast<double>(app::kTorqueAmplitudePerMille);
      drive->applyTorque(app::clampToI32(amp * std::sin(wt)));
    });
  }

  const bool stopped_early = !completed;

  /* Return to a neutral setpoint before tearing down. */
  if (!stopped_early) {
    drive->applyTorque(0);
    drive->moveAtVelocity(0);
    std::this_thread::sleep_for(update_period);
  }

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
