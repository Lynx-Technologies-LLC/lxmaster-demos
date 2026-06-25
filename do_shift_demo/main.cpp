#include <atomic>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <iostream>
#include <thread>

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

constexpr int kNumBits = 16;
constexpr double kBitIntervalS = 1.0;

static double elapsedS(std::chrono::steady_clock::time_point now,
                       std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(now - start).count();
}

/* Index of the single active output bit for the current time, or -1 once the walk is done. */
static int activeBitAt(std::chrono::steady_clock::time_point now,
                       std::chrono::steady_clock::time_point start) {
  const int bit = static_cast<int>(elapsedS(now, start) / kBitIntervalS);
  return bit < kNumBits ? bit : -1;
}

/* Clear every output, then light only `bit`. */
static void applyBit(lxmaster::IoModule* mod, int bit) {
  for (int ch = 0; ch < kNumBits; ++ch) {
    mod->writeDigital(static_cast<std::size_t>(ch), false);
  }
  mod->writeDigital(static_cast<std::size_t>(bit), true);
  std::cout << "Bit " << bit << " ON\n";
}

int main() {

  // Capture Ctrl+c so we can stop the comms early and stop the trajectory cleanly
  installSignalHandler();

  // Get a default set of Ethercat Network parameters
  lxmaster::NetworkConfig cfg = lxmaster::NetworkConfig::defaults();

  // check to make sure ethercat interface name is set from ENV
  if (cfg.bus.ifname.empty()) {
    std::cerr << "No EtherCAT interface: set LXMASTER_RT_IFACE in /etc/profile.d/lxmaster-config.sh "
                 "(run lxmaster host tune first).\n";
    return 1;
  }

  // set the path for network ENI file (generate by: lxmaster eni gen -h )
  cfg.eni.eni_path = "~/myenifolder/myenifile.xml";
  /* Sync mode is not chosen here: it is decided by the ENI, which is generated from the discovered
   * modules. A pure-I/O bus (no SYNC0 device) resolves to SM-event automatically; a mixed bus that
   * also carries a DC-strict slave (e.g. a servo) resolves to DC, in which case SYNC0 is activated
   * per-slave (only the drives) while the I/O modules keep running on the SM event. */

  // define an ethercat network
  lxmaster::EcNetwork net(cfg);

  // prepare the network
  if (!net.prepare()) {
    std::cerr << "EcNetwork::prepare() failed: " << net.lastError() << "\n";
    return 1;
  }

  // find the first I/O module with enough digital outputs
  lxmaster::IoModule* mod = nullptr;
  for (lxmaster::IoModule* m : net.ioModules()) {
    if (m->digitalOutputCount() >= static_cast<std::size_t>(kNumBits)) {
      mod = m;
      break;
    }
  }

  if (mod == nullptr) {
    std::cerr << "No I/O module with >= " << kNumBits
              << " digital outputs found on the bus.\n";
    return 1;
  }

  // opt this module into the OPERATIONAL bring-up
  mod->configure();

  // start the network
  if (!net.start()) {
    std::cerr << "EcNetwork::start() failed: " << net.lastError() << "\n";
    return 1;
  }

  std::cout << "Found I/O module: \"" << mod->name() << "\""
            << " (" << mod->digitalOutputCount() << " digital outputs)\n"
            << "Running " << kNumBits << "-bit left shift...\n";

  // clear all outputs before starting
  for (int ch = 0; ch < kNumBits; ++ch) {
    mod->writeDigital(static_cast<std::size_t>(ch), false);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // walk one active output bit across all channels, one per second
  const auto start = std::chrono::steady_clock::now();
  int active_bit = -1;
  while (net.isRunning() && !g_interrupted) {
    const auto now = std::chrono::steady_clock::now();
    const int bit = activeBitAt(now, start);
    if (bit < 0) break;

    if (bit != active_bit) {
      active_bit = bit;
      applyBit(mod, bit);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  /* The cyclic RT thread can drop out mid-run (WKC watchdog, DC-sync loss, a slave leaving OP).
   * If that happened, output writes never reached the wire — surface it instead of running blind. */
  if (!net.isRunning() && !g_interrupted) {
    std::cerr << "EtherCAT cyclic thread stopped mid-run: " << net.lastError() << "\n";
    net.stop();
    return 1;
  }

  // clear all outputs on exit
  for (int ch = 0; ch < kNumBits; ++ch) {
    mod->writeDigital(static_cast<std::size_t>(ch), false);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // stop the ethercat network, this starts graceful shutdown process and joins the realtime thread.
  net.stop();
  std::cout << "Done.\n";
  return 0;
}
