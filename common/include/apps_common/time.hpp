#pragma once

#include <cstdint>
#include <ctime>

/**
 * Small monotonic-clock helpers shared by the examples that run a hand-rolled cyclic loop
 * (e.g. `network_probe`). These mirror the deadline arithmetic the RT `CyclicExecutor` does
 * internally; keeping one copy here stops duplicates from drifting. Header-only — no link
 * dependency.
 */
namespace apps_common {

/** Advance `ts` by `delta_ns` (may be negative), normalizing tv_nsec into [0, 1e9). */
inline void addTimespecNsec(timespec* ts, std::int64_t delta_ns) {
  std::int64_t ns = ts->tv_nsec + delta_ns;
  while (ns >= 1'000'000'000L) {
    ns -= 1'000'000'000L;
    ++ts->tv_sec;
  }
  while (ns < 0) {
    ns += 1'000'000'000L;
    --ts->tv_sec;
  }
  ts->tv_nsec = static_cast<long>(ns);
}

/**
 * A CLOCK_MONOTONIC start deadline rounded up to a whole-millisecond boundary `lead_ms`
 * milliseconds in the future, so the first `clock_nanosleep(TIMER_ABSTIME)` lands on a clean
 * tick rather than mid-millisecond.
 */
inline timespec alignedStartDeadline(int lead_ms) {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  ts.tv_nsec = ((ts.tv_nsec / 1'000'000L) + lead_ms) * 1'000'000L;
  while (ts.tv_nsec >= 1'000'000'000L) {
    ++ts.tv_sec;
    ts.tv_nsec -= 1'000'000'000L;
  }
  return ts;
}

}  // namespace apps_common
