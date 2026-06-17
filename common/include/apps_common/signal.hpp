#pragma once

#include <atomic>
#include <csignal>

/**
 * Shared SIGINT / SIGTERM handling for the example programs. Every example installs the
 * same handler, which flips a single process-wide flag; the program's main loop polls
 * `apps_common::interrupted()` and shuts the bus down gracefully instead of dying mid
 * frame. Header-only so it adds no link dependency.
 */
namespace apps_common {

/** Process-wide interrupt flag, set from the async-signal-safe handler. */
inline std::atomic<bool>& interruptFlag() {
  static std::atomic<bool> flag{false};
  return flag;
}

inline void onSignal(int) { interruptFlag().store(true, std::memory_order_release); }

/** Install the handler for SIGINT and SIGTERM. Call once, early in `main`. */
inline void installSignalHandler() {
  struct sigaction sa {};
  sa.sa_handler = &onSignal;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);
}

/** True once SIGINT / SIGTERM has been received. */
inline bool interrupted() { return interruptFlag().load(std::memory_order_acquire); }

}  // namespace apps_common
