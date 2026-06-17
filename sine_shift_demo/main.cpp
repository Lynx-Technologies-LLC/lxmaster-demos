#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <lxmstr/lxmstr.hpp>

#include "apps_common/signal.hpp"

/**
 * Combined demo: drive a servo in a sine (CSP) while walking a single active bit
 * across a digital-output module, both running OPERATIONAL on the same bus.
 *
 * Only the ENI is taken on the command line (--eni); the EtherCAT interface comes from the
 * LXMSTR_RT_IFACE env, and every other knob (sine shape, bit timing, DC tuning) is hardcoded
 * below — edit the kXxx constants to retarget.
 */
namespace sine_shift_demo {

/** Sine profile (encoder counts / Hz / number of full cycles to run). */
constexpr std::int32_t kAmplitudeCounts = 50000;
constexpr double kFrequencyHz = 1.0;

/** Digital-output bit walk: one active bit, advanced once per second. */
constexpr int kNumBits = 16;
constexpr double kBitIntervalS = 1.0;

/** Total run length is driven by the bit walk (kNumBits seconds). */
constexpr double kRunSeconds = kNumBits * kBitIntervalS;

struct Options {
  const char* eni_path = nullptr;
};

void printUsage(const char* exe) {
  std::cerr << "Usage: " << exe << " --eni <file>\n"
            << "\n"
            << "Combined sine (CSP servo) + digital-output bit-walk demo. Brings every axis and the\n"
            << "first I/O module with >= " << kNumBits << " digital outputs to OPERATIONAL, then drives a\n"
            << "sine on the servo while walking one active output bit, both for " << kRunSeconds << " s.\n"
            << "Sine shape, bit timing, and DC tuning are hardcoded; the EtherCAT interface comes from\n"
            << "the LXMSTR_RT_IFACE env (set by lxmstr host tune).\n"
            << "\n"
            << "Required:\n"
            << "  --eni <file>     ENI (EtherCATConfig) to load, validate, and run.\n";
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
   * NetworkConfig::defaults(). A mixed bus with an OP servo resolves to DC; the I/O module
   * keeps running on the SM event. */
  cfg.eni.eni_path = opts.eni_path;
  return cfg;
}

}  // namespace sine_shift_demo

int main(int argc, char* argv[]) {
  sine_shift_demo::Options opts;
  if (!sine_shift_demo::parseCli(argc, argv, opts)) return 1;

  apps_common::installSignalHandler();

  lxmstr::NetworkConfig cfg = sine_shift_demo::makeConfig(opts);
  if (cfg.bus.ifname.empty()) {
    std::cerr << "No EtherCAT interface: set LXMSTR_RT_IFACE in /etc/profile.d/lxmstr-config.sh "
                 "(run lxmstr host tune first).\n";
    return 1;
  }
  lxmstr::EcNetwork net(cfg);

  std::cout << "sine+shift demo: iface=" << cfg.bus.ifname << " eni=" << opts.eni_path << "\n";

  if (!net.prepare()) {
    std::cerr << "EcNetwork::prepare() failed: " << net.lastError() << "\n";
    return 1;
  }

  /* Bring every motion axis to OP in CSP. configure() opts the device into the OPERATIONAL
   * bring-up; anything left unconfigured stays at its default PRE_OP. */
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

  /* Find + opt-in the first I/O module with enough digital outputs. */
  lxmstr::IoModule* io = nullptr;
  for (lxmstr::IoModule* m : net.ioModules()) {
    if (m->digitalOutputCount() >= static_cast<std::size_t>(sine_shift_demo::kNumBits)) {
      io = m;
      break;
    }
  }
  if (io == nullptr) {
    std::cerr << "No I/O module with >= " << sine_shift_demo::kNumBits
              << " digital outputs found on the bus.\n";
    return 1;
  }
  io->configure();

  if (!net.start()) {
    std::cerr << "EcNetwork::start() failed: " << net.lastError() << "\n";
    return 1;
  }

  /* Let the RT thread publish a couple of cycles so position() is non-zero. */
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const std::int32_t hold = drive->actualPosition();

  std::cout << "Operational.\n"
            << "  axis '" << drive->name() << "': center=" << hold
            << " amplitude=" << sine_shift_demo::kAmplitudeCounts
            << " freq=" << sine_shift_demo::kFrequencyHz << " Hz\n"
            << "  I/O  '" << io->name() << "': " << io->digitalOutputCount()
            << " DO, " << sine_shift_demo::kNumBits << "-bit walk @ "
            << sine_shift_demo::kBitIntervalS << " s\n"
            << "  sync=" << (net.syncMode() == lxmstr::SyncMode::DcSync0 ? "DC" : "SM-event")
            << " cycle=" << net.cycleTimeNs() << " ns"
            << " -> run " << sine_shift_demo::kRunSeconds << " s\n";

  /* Clear all outputs and latch the first sine target at hold before t=0. */
  for (int ch = 0; ch < sine_shift_demo::kNumBits; ++ch) {
    io->writeDigital(static_cast<std::size_t>(ch), false);
  }
  drive->moveTo(hold);

  constexpr double kTwoPi = 6.283185307179586476925286766559;
  const auto update_period = std::chrono::nanoseconds(net.cycleTimeNs());
  std::this_thread::sleep_for(update_period);

  const auto start = std::chrono::steady_clock::now();
  int active_bit = -1;
  while (net.isRunning() && !apps_common::interrupted()) {
    const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (t >= sine_shift_demo::kRunSeconds) break;

    /* --- Digital output: advance the active bit once per interval. --- */
    const int bit = static_cast<int>(t / sine_shift_demo::kBitIntervalS);
    if (bit != active_bit && bit < sine_shift_demo::kNumBits) {
      active_bit = bit;
      for (int ch = 0; ch < sine_shift_demo::kNumBits; ++ch) {
        io->writeDigital(static_cast<std::size_t>(ch), false);
      }
      io->writeDigital(static_cast<std::size_t>(bit), true);
      std::cout << "Bit " << bit << " ON\n";
    }

    /* --- Servo: cos(ωt) sine around hold (matches servo_sin_demo's profile). --- */
    const double omega_t = kTwoPi * sine_shift_demo::kFrequencyHz * t;
    const double amp = static_cast<double>(sine_shift_demo::kAmplitudeCounts);
    const double raw = static_cast<double>(hold) + 0.5 * amp * std::cos(omega_t) - 0.5 * amp;
    const double clamped = std::max(
        static_cast<double>(std::numeric_limits<std::int32_t>::min()),
        std::min(static_cast<double>(std::numeric_limits<std::int32_t>::max()), raw));
    drive->moveTo(static_cast<std::int32_t>(clamped));

    std::this_thread::sleep_for(update_period);
  }

  const bool stopped_early = !net.isRunning();

  /* Clear outputs and recenter the drive before tearing down. */
  if (!stopped_early) {
    for (int ch = 0; ch < sine_shift_demo::kNumBits; ++ch) {
      io->writeDigital(static_cast<std::size_t>(ch), false);
    }
    drive->moveTo(hold);
    std::this_thread::sleep_for(update_period);
  }

  net.stop();

  if (stopped_early) {
    std::cerr << '\n';
    net.reportDeviceStatus(std::cerr);
    const std::string reason = net.lastError();
    std::cerr << "\nRun ended early (sine/shift incomplete).\n";
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
