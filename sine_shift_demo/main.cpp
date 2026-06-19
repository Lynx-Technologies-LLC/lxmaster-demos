#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
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
 * Combined demo: drive a servo in a sine (CSP) while walking a single active bit
 * across a digital-output module, both running OPERATIONAL on the same bus.
 *
 * The EtherCAT interface comes from the LXMSTR_RT_IFACE env; the ENI path and every other knob
 * (sine shape, bit timing, DC tuning) are hardcoded below — edit the kXxx constants to retarget.
 */
/** Sine profile (encoder counts / Hz / number of full cycles to run). */
constexpr std::int32_t kAmplitudeCounts = 50000;
constexpr double kFrequencyHz = 1.0;

/** Digital-output bit walk: one active bit, advanced once per second. */
constexpr int kNumBits = 16;
constexpr double kBitIntervalS = 1.0;

/** Total run length is driven by the bit walk (kNumBits seconds). */
constexpr double kRunSeconds = kNumBits * kBitIntervalS;

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

/* cos(ωt) sine around hold (matches servo_sin_demo's profile). */
static std::int32_t positionAt(std::chrono::steady_clock::time_point now,
                               std::chrono::steady_clock::time_point start, std::int32_t hold) {
  const double omega_t = kTwoPi * kFrequencyHz * elapsedS(now, start);
  const double amp = static_cast<double>(kAmplitudeCounts);
  return clampI32(static_cast<double>(hold) + 0.5 * amp * std::cos(omega_t) - 0.5 * amp);
}

/* Index of the single active output bit for the current time, or -1 once the walk is done. */
static int activeBitAt(std::chrono::steady_clock::time_point now,
                       std::chrono::steady_clock::time_point start) {
  const int bit = static_cast<int>(elapsedS(now, start) / kBitIntervalS);
  return bit < kNumBits ? bit : -1;
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

  std::cout << "sine+shift demo: iface=" << cfg.bus.ifname << " eni=" << cfg.eni.eni_path << "\n";

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

  // find the first I/O module with enough digital outputs
  lxmstr::IoModule* io = nullptr;
  for (lxmstr::IoModule* m : net.ioModules()) {
    if (m->digitalOutputCount() >= static_cast<std::size_t>(kNumBits)) {
      io = m;
      break;
    }
  }
  if (io == nullptr) {
    std::cerr << "No I/O module with >= " << kNumBits
              << " digital outputs found on the bus.\n";
    return 1;
  }

  // opt this I/O module into the OPERATIONAL bring-up
  io->configure();

  // start the network
  if (!net.start()) {
    std::cerr << "EcNetwork::start() failed: " << net.lastError() << "\n";
    return 1;
  }

  // Let the RT thread publish a couple of cycles so position() is non-zero.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // get current position of the drive
  const std::int32_t hold = drive->actualPosition();

  std::cout << "Operational.\n"
            << "  axis '" << drive->name() << "': center=" << hold
            << " amplitude=" << kAmplitudeCounts
            << " freq=" << kFrequencyHz << " Hz\n"
            << "  I/O  '" << io->name() << "': " << io->digitalOutputCount()
            << " DO, " << kNumBits << "-bit walk @ "
            << kBitIntervalS << " s\n"
            << "  sync=" << (net.syncMode() == lxmstr::SyncMode::DcSync0 ? "DC" : "SM-event")
            << " cycle=" << net.cycleTimeNs() << " ns"
            << " -> run " << kRunSeconds << " s\n";

  // clear all outputs and latch the first sine target at hold before t=0
  for (int ch = 0; ch < kNumBits; ++ch) {
    io->writeDigital(static_cast<std::size_t>(ch), false);
  }
  drive->moveTo(hold);

  // we will try to create a trajectory that is in same step as the ethercat network cycletime
  const auto update_period = std::chrono::nanoseconds(net.cycleTimeNs());
  std::this_thread::sleep_for(update_period);

  const auto start = std::chrono::steady_clock::now();
  int active_bit = -1;

  // feed the trajectory points and walk output bits as long as system is not interrupted
  while (net.isRunning() && !g_interrupted) {
    const auto now = std::chrono::steady_clock::now();
    const int bit = activeBitAt(now, start);
    if (bit < 0) break;

    // advance the active output bit once per interval
    if (bit != active_bit) {
      active_bit = bit;
      for (int ch = 0; ch < kNumBits; ++ch) {
        io->writeDigital(static_cast<std::size_t>(ch), false);
      }
      io->writeDigital(static_cast<std::size_t>(bit), true);
      std::cout << "Bit " << bit << " ON\n";
    }

    // take a new trajectory point and send it to drive
    drive->moveTo(positionAt(now, start, hold));

    std::this_thread::sleep_for(update_period);
  }

  const bool stopped_early = !net.isRunning();

  // clear outputs and recenter the drive before tearing down
  if (!stopped_early) {
    for (int ch = 0; ch < kNumBits; ++ch) {
      io->writeDigital(static_cast<std::size_t>(ch), false);
    }
    drive->moveTo(hold);
    std::this_thread::sleep_for(update_period);
  }

  // stop the ethercat network, this starts graceful shutdown process and joins the realtime thread.
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
