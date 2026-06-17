#pragma once

#include "ecmaster/rt_scheduling.hpp"

/**
 * Thin convenience wrapper over `ecmaster::applyRealtimeScheduling` for examples that run a
 * hand-rolled cyclic loop on the main thread (e.g. network_probe). The
 * canonical implementation lives in `ecmaster` next to `RtCycleClock`; this header just
 * adapts the loose (enable, prio, cpu) argument style the examples parse from the CLI.
 */
namespace apps_common {

inline bool applyRealtimeScheduling(bool enable, int rt_priority, int cpu_affinity) {
  ecmaster::RtSchedulingParams params;
  params.enable = enable;
  params.rt_priority = rt_priority;
  params.cpu_affinity = cpu_affinity;
  return ecmaster::applyRealtimeScheduling(params);
}

}  // namespace apps_common
