#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <thread>
#include <vector>

#include <lxmstr/lxmstr.hpp>

#include "apps_common/net_config.hpp"
#include "apps_common/signal.hpp"

namespace servo_sin_torque_demo {

struct CliOptions {
  apps_common::CommonOptions common;
  const char* eni_path = nullptr;
  /* Target torque (0x6071) unit is per-mille of rated torque (1000 = rated). Default 100 = 10%. */
  std::int32_t amplitude_per_mille = 100;
  double frequency_hz = 0.5;
  int cycles = 5;
  std::size_t sync_trace_capacity = 0;
  std::uint32_t sync_trace_window_ns = 0;
  const char* sync_trace_file = nullptr;
  /* When set, the sync trace ring captures every cyclic iteration (warmup / DC gate /
   * cooldown / OP), not just OPERATIONAL. Required to see the PI lock-up. */
  bool sync_trace_include_warmup = false;
  /** Legacy CiA402: auto fault-reset pulse + re-enable after drive faults during OP. */
  bool auto_fault_recover = false;
};

void printUsage(const char* exe) {
  std::cerr
      << "Usage: " << exe << " --eni <file> [options]\n"
      << "\n"
      << "ENI-driven generic CiA402 torque (CST) sine demo. The ENI is validated against the\n"
      << "project-bundled ETG.2100 schema (embedded — no --xsd). Generate one from the CST ESI\n"
      << "(STEPPERONLINE_A6_Servo_V0.04.cst.xml) with eni_gen before running. The EtherCAT\n"
      << "interface is taken from the LXMSTR_RT_IFACE env (set by lxmstr host tune).\n"
      << "\n"
      << "WARNING: in torque mode the drive applies torque directly with no position/velocity\n"
      << "loop. An unloaded shaft will accelerate; ensure the axis is safe to spin freely.\n"
      << "\n"
      << "Required:\n"
      << "  --eni <file>     ENI (EtherCATConfig) to load, validate, and run. Must map target\n"
      << "                   torque 0x6071 and set modes-of-operation 0x6060 to CST (10).\n"
      << "\n"
      << "Commands a sinusoidal CST target torque:\n"
      << "  trq(t) = amplitude * sin(2 * pi * frequency * t)   [per-mille of rated torque]\n"
      << "  (sin(0)=0, so the drive starts and ends at zero torque — no startup step.)\n"
      << "The cyclic period is taken solely from the ENI (<Config><Cyclic><CycleTime>); set it\n"
      << "with eni_gen's --cycle-ns when generating the ENI, not here.\n"
      << "\n"
      << "Options:\n"
      << "  --amplitude N       Torque amplitude in per-mille of rated torque (1000 = rated;\n"
      << "                      default: 100 = 10%). Clamped to INT16 range.\n"
      << "  --frequency F       Sine frequency in Hz (default: 0.5).\n"
      << "  --cycles N          Number of sine cycles to run (default: 5).\n"
      << "  --debug             Console debug tracer (requires -DLXMSTR_ENABLE_DEBUG=ON).\n"
      << "  --sync-trace N      Record last N OPERATIONAL DC cycles (delta + host jitter);\n"
      << "                      after stop(), print full chronological history. Default off.\n"
      << "  --sync-trace-window-ns W  Drive sync-window threshold (ns); counts violations\n"
      << "                      where |dc_delta| > W and logs during the run when it changes.\n"
      << "                      Default: same as LXMSTR_SYNC_WINDOW_NS if set, else 0 (no tally).\n"
      << "  --sync-trace-file PATH  Write the full sync-trace table to PATH (TSV rows).\n"
      << "                      When set, the table is not printed to stdout — only a short\n"
      << "                      confirmation line (useful for narrow terminals).\n"
      << "  --sync-trace-include-warmup  Capture every cyclic iteration in the sync trace,\n"
      << "                           not just OPERATIONAL cycles. The TSV `phase` column\n"
      << "                           distinguishes warmup/gate/cooldown/op. Required to see\n"
      << "                           PI lock-up. Violation count still applies only to OP.\n"
      << "  --auto-fault-recover   Legacy CiA402: automatically fault-reset and re-enable\n"
      << "                         after drive faults during OP. Default off — first fault\n"
      << "                         captures 0x603F/0x203F, prints a diagnostic, and stops.\n";
}

bool parseCli(int argc, char* argv[], CliOptions& opts) {
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--eni")) {
      if (i + 1 >= argc) { std::cerr << "Missing value for --eni\n"; return false; }
      opts.eni_path = argv[++i];
      continue;
    }
    const apps_common::ArgParse common =
        apps_common::parseCommonArg(argc, argv, i, opts.common);
    if (common == apps_common::ArgParse::Consumed) continue;
    if (common == apps_common::ArgParse::Error) return false;
    if (!std::strcmp(argv[i], "--auto-fault-recover")) opts.auto_fault_recover = true;
    else if (!std::strcmp(argv[i], "--sync-trace")) {
      if (i + 1 >= argc) { std::cerr << "Missing value for --sync-trace\n"; return false; }
      opts.sync_trace_capacity = static_cast<std::size_t>(std::strtoull(argv[++i], nullptr, 10));
      if (opts.sync_trace_capacity == 0) {
        std::cerr << "--sync-trace N requires N >= 1\n";
        return false;
      }
    } else if (!std::strcmp(argv[i], "--sync-trace-window-ns")) {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --sync-trace-window-ns\n";
        return false;
      }
      opts.sync_trace_window_ns = static_cast<std::uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (!std::strcmp(argv[i], "--sync-trace-file")) {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --sync-trace-file\n";
        return false;
      }
      opts.sync_trace_file = argv[++i];
    } else if (!std::strcmp(argv[i], "--sync-trace-include-warmup")) {
      opts.sync_trace_include_warmup = true;
    } else if (!std::strcmp(argv[i], "--amplitude")) {
      if (i + 1 >= argc) { std::cerr << "Missing value for --amplitude\n"; return false; }
      opts.amplitude_per_mille = static_cast<std::int32_t>(std::strtol(argv[++i], nullptr, 10));
    } else if (!std::strcmp(argv[i], "--frequency")) {
      if (i + 1 >= argc) { std::cerr << "Missing value for --frequency\n"; return false; }
      opts.frequency_hz = std::strtod(argv[++i], nullptr);
      if (!(opts.frequency_hz > 0.0)) {
        std::cerr << "--frequency must be > 0\n";
        return false;
      }
    } else if (!std::strcmp(argv[i], "--cycles")) {
      if (i + 1 >= argc) { std::cerr << "Missing value for --cycles\n"; return false; }
      opts.cycles = static_cast<int>(std::strtol(argv[++i], nullptr, 10));
      if (opts.cycles < 1) opts.cycles = 1;
    } else if (!std::strcmp(argv[i], "--update-us") || !std::strcmp(argv[i], "--cycle-ns")) {
      std::cerr << argv[i]
                << " is not accepted; the cyclic period comes solely from the ENI. Set it with "
                   "eni_gen --cycle-ns when generating the ENI.\n";
      return false;
    } else if (!std::strcmp(argv[i], "-h") || !std::strcmp(argv[i], "--help")) {
      printUsage(argv[0]);
      std::exit(0);
    } else {
      std::cerr << "Unknown argument: " << argv[i] << "\n";
      return false;
    }
  }
  if (!opts.eni_path) { std::cerr << "Missing --eni <file>\n"; printUsage(argv[0]); return false; }
  if (opts.sync_trace_file && opts.sync_trace_file[0] != '\0' && opts.sync_trace_capacity == 0) {
    std::cerr << "--sync-trace-file requires --sync-trace N\n";
    return false;
  }
  return true;
}

lxmstr::NetworkConfig make_network_config(const CliOptions& opts) {
  lxmstr::NetworkConfig cfg = apps_common::makeBaseNetworkConfig(opts.common);
  /* Host-side DC knobs carry the LXMSTR_* env / defaults() values via makeBaseNetworkConfig.
   * They are configured exclusively through the host config (built-in default < ENV); the demo
   * provides no CLI override for them. */
  cfg.dc.sync_trace_include_warmup = opts.sync_trace_include_warmup;
  cfg.debug.debug = opts.common.debug;
  cfg.debug.sync_trace_capacity = opts.sync_trace_capacity;
  cfg.debug.sync_trace_window_ns = opts.sync_trace_window_ns;

  cfg.eni.eni_path = opts.eni_path;
  return cfg;
}

std::uint32_t syncTraceWindowFromEnvIfUnset(std::uint32_t cli_window) {
  if (cli_window != 0) return cli_window;
  if (const char* env = std::getenv("LXMSTR_SYNC_WINDOW_NS")) {
    if (env[0] != '\0') {
      const unsigned long v = std::strtoul(env, nullptr, 10);
      if (v > 0 && v <= 65535UL) return static_cast<std::uint32_t>(v);
    }
  }
  return 0;
}

void printSyncTraceReport(const lxmstr::EcNetwork::SyncTraceReport& rep,
                          std::uint32_t violation_window_ns, std::ostream& os) {
  os << "\n[sync-trace] ring_capacity=" << rep.ring_capacity << " total_writes=" << rep.total_writes
     << " linearized_samples=" << rep.samples.size();
  if (rep.ring_capacity > 0 && rep.total_writes > rep.ring_capacity) {
    os << "  (oldest samples overwritten — raise --sync-trace N)";
  }
  os << "\n";
  if (violation_window_ns > 0) {
    os << "[sync-trace] violation_window_ns=" << violation_window_ns
       << "  violation_cycles=" << rep.violation_count << "\n";
    std::size_t yes_rows_in_file = 0;
    for (const lxmstr::SyncTraceSample& r : rep.samples) {
      const std::int64_t ad = r.dc_delta_ns >= 0 ? r.dc_delta_ns : -r.dc_delta_ns;
      if (ad > static_cast<std::int64_t>(violation_window_ns)) ++yes_rows_in_file;
    }
    os << "[sync-trace] YES rows in this file only=" << yes_rows_in_file
       << " (if violation_cycles is larger, early OP cycles were overwritten — use a larger "
          "--sync-trace N)\n";
  } else {
    os << "[sync-trace] violation_window_ns=0 (no per-cycle violation tally; dc_delta is raw "
          "phase error before PI)\n";
  }

  if (rep.samples.empty()) {
    os << "[sync-trace] (no samples — need DC+ec_sync_to_dc mode and OPERATIONAL cycles)\n";
    return;
  }

  /* Worst |dc_delta| considers only OPERATIONAL samples — warmup/gate/cooldown excursions
   * are an artifact of the PI locking up and should not be reported as the bus's worst. */
  std::int64_t worst_abs = 0;
  std::uint64_t worst_cycle = 0;
  std::size_t op_sample_count = 0;
  for (const lxmstr::SyncTraceSample& r : rep.samples) {
    if (r.phase != lxmstr::SyncTracePhase::Operational) continue;
    ++op_sample_count;
    const std::int64_t ad = r.dc_delta_ns >= 0 ? r.dc_delta_ns : -r.dc_delta_ns;
    if (ad > worst_abs) {
      worst_abs = ad;
      worst_cycle = r.rt_cycle;
    }
  }
  os << "[sync-trace] worst |dc_delta_ns|=" << worst_abs << " at rt_cycle=" << worst_cycle
     << "  (over " << op_sample_count << " OPERATIONAL samples; non-OP excluded)\n";

  auto phaseStr = [](lxmstr::SyncTracePhase p) -> const char* {
    switch (p) {
      case lxmstr::SyncTracePhase::Warmup:      return "warmup";
      case lxmstr::SyncTracePhase::DcGate:      return "dcgate";
      case lxmstr::SyncTracePhase::Cooldown:    return "cooldn";
      case lxmstr::SyncTracePhase::Operational: return "op";
      case lxmstr::SyncTracePhase::Shutdown:    return "shutdn";
    }
    return "?";
  };

  os << "[sync-trace] columns (tab-separated): "
        "seq\trt_cycle\tphase\top_cycle\tdc_delta_ns\tjitter_err_ns\tabs_dc"
        "\tintegral_ns\ttoff_ns\tsync_violation\n";
  os << "[sync-trace] phase: warmup|dcgate|cooldn|op|shutdn\n";
  if (violation_window_ns > 0) {
    os << "[sync-trace] sync_violation: YES if abs_dc > violation_window_ns AND phase=op\n";
  }
  std::size_t seq = 0;
  for (const lxmstr::SyncTraceSample& r : rep.samples) {
    const std::int64_t ad = r.dc_delta_ns >= 0 ? r.dc_delta_ns : -r.dc_delta_ns;
    const bool viol = violation_window_ns > 0 && r.phase == lxmstr::SyncTracePhase::Operational &&
                      ad > static_cast<std::int64_t>(violation_window_ns);
    os << seq << '\t' << r.rt_cycle << '\t' << phaseStr(r.phase) << '\t'
       << r.operational_cycle_seq << '\t' << r.dc_delta_ns << '\t' << r.jitter_err_ns << '\t'
       << ad << '\t' << r.integral_ns << '\t' << r.toff_ns << '\t' << (viol ? "YES" : "no") << '\n';
    ++seq;
  }
  os << "[sync-trace] end of history (" << rep.samples.size() << " rows)\n";
}

}  // namespace servo_sin_torque_demo

int main(int argc, char* argv[]) {
  servo_sin_torque_demo::CliOptions opts;
  if (!servo_sin_torque_demo::parseCli(argc, argv, opts)) return 1;
  apps_common::installSignalHandler();

  if (opts.sync_trace_capacity > 0) {
    opts.sync_trace_window_ns =
        servo_sin_torque_demo::syncTraceWindowFromEnvIfUnset(opts.sync_trace_window_ns);
  }

  lxmstr::NetworkConfig cfg = servo_sin_torque_demo::make_network_config(opts);
  if (cfg.bus.ifname.empty()) {
    std::cerr << "No EtherCAT interface: set LXMSTR_RT_IFACE in /etc/profile.d/lxmstr-config.sh "
                 "(run lxmstr host tune first).\n";
    return 1;
  }
  lxmstr::EcNetwork net(cfg);

  if (!net.prepare()) {
    std::cerr << "EcNetwork::prepare() failed: " << net.lastError() << "\n";
    return 1;
  }

  for (lxmstr::Axis* ax : net.axes()) {
    ax->setDriveMode(lxmstr::DriveOpMode::Cst);
    ax->configure();  // walk this axis up to OP
  }

  std::vector<lxmstr::Axis*> axes = net.axes();
  if (axes.empty()) {
    std::cerr << "ENI produced no motion axes to command.\n";
    return 1;
  }
  lxmstr::Axis* drive = axes.front();

  if (!net.start()) {
    std::cerr << "EcNetwork::start() failed: " << net.lastError() << "\n";
    return 1;
  }

  /* Let the RT thread publish a couple of cycles before commanding torque. */
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  const double period_s = 1.0 / opts.frequency_hz;
  const double total_s  = period_s * static_cast<double>(opts.cycles);

  std::cout << "Operational. CST torque sine profile:\n"
            << "  amplitude (per-mille) = " << opts.amplitude_per_mille << "\n"
            << "  frequency (Hz)        = " << opts.frequency_hz << "\n"
            << "  cycles                = " << opts.cycles
            << "  -> duration " << total_s << " s\n"
            << "  cycle period          = " << net.cycleTimeNs() << " ns (from ENI)\n";
  if (net.syncMode() == lxmstr::SyncMode::DcSync0) {
    std::cout << "  DC busy-wait     = " << cfg.dc.dc_sync_busy_wait_ns << " ns";
    if (cfg.dc.dc_sync_busy_wait_ns == 0) std::cout << " (disabled)";
    std::cout << "\n";
  }
  if (opts.sync_trace_capacity > 0) {
    std::cout << "  sync trace ring  = " << opts.sync_trace_capacity << " cycles";
    if (cfg.debug.sync_trace_window_ns > 0) {
      std::cout << "  violation_window_ns=" << cfg.debug.sync_trace_window_ns;
    }
    if (cfg.dc.sync_trace_include_warmup) {
      std::cout << "  include_warmup=yes";
    }
    std::cout << "\n";
  }

  constexpr double kTwoPi = 6.283185307179586476925286766559;
  /* Start at zero torque (sin(0)=0) so the profile does not step away from rest at t=0. */
  drive->applyTorque(0);
  const auto update_period = std::chrono::nanoseconds(net.cycleTimeNs());
  std::this_thread::sleep_for(update_period);
  const auto start    = std::chrono::steady_clock::now();
  const auto deadline = start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    std::chrono::duration<double>(total_s));

  std::uint64_t last_violation_count = 0;
  while (net.isRunning() && !apps_common::interrupted()) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) break;

    const double t = std::chrono::duration<double>(now - start).count();
    const double omega_t = kTwoPi * opts.frequency_hz * t;
    const double s = std::sin(omega_t);
    const double amp = static_cast<double>(opts.amplitude_per_mille);
    /* trq(t) = amplitude * sin(ωt); sin(0)=0 so it ramps up smoothly from no torque. Target
     * torque 0x6071 is INT16, so clamp to that range before commanding. */
    const double raw = amp * s;
    const double clamped = std::max(
        static_cast<double>(std::numeric_limits<std::int16_t>::min()),
        std::min(static_cast<double>(std::numeric_limits<std::int16_t>::max()), raw));
    drive->applyTorque(static_cast<std::int32_t>(clamped));

    if (opts.sync_trace_capacity > 0 && cfg.debug.sync_trace_window_ns > 0) {
      const std::uint64_t vc = net.syncTraceViolationCount();
      if (vc != last_violation_count) {
        std::cerr << "[sync-trace] OPERATIONAL cycles with |dc_delta_ns| > "
                  << cfg.debug.sync_trace_window_ns << " ns: " << vc << "\n";
        last_violation_count = vc;
      }
    }

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

  net.stop();

  if (opts.sync_trace_capacity > 0) {
    const lxmstr::EcNetwork::SyncTraceReport rep = net.syncTraceReport();
    bool wrote_file = false;
    if (opts.sync_trace_file && opts.sync_trace_file[0] != '\0') {
      std::ofstream out(opts.sync_trace_file);
      if (!out) {
        std::cerr << "[sync-trace] failed to open \"" << opts.sync_trace_file
                  << "\" — printing full report to stdout instead.\n";
      } else {
        servo_sin_torque_demo::printSyncTraceReport(rep, cfg.debug.sync_trace_window_ns, out);
        wrote_file = true;
        std::cout << "[sync-trace] wrote " << rep.samples.size() << " sample row(s) to \""
                  << opts.sync_trace_file << "\"\n";
      }
    }
    if (!wrote_file) {
      servo_sin_torque_demo::printSyncTraceReport(rep, cfg.debug.sync_trace_window_ns, std::cout);
    }
  }

  if (stopped_early) {
    std::cerr << '\n';
    net.reportDeviceStatus(std::cerr);
    const std::string reason = net.lastError();
    std::cerr << "\nRun ended early (torque sine profile incomplete).\n";
    if (!reason.empty()) {
      std::cerr << "  Reason: " << reason << "\n";
    } else {
      std::cerr << "  Reason: (none reported — run with --debug for cycle-level trace)\n";
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
