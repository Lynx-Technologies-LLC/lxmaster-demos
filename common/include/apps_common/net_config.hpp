#pragma once

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "ecnet/network_config.hpp"

/**
 * Shared CLI flags and base `NetworkConfig` construction for the EcNetwork-driven examples
 * (servo_sin_demo). Each example keeps its own bespoke flags (sine profile, DC-PI tuning, ...)
 * and folds the common ones in via `parseCommonArg`.
 */
namespace apps_common {

/** Flags every EcNetwork example understands. The cyclic period is intentionally NOT here: it
 *  is owned by the ENI (`<Config><Cyclic><CycleTime>`) and adopted at load time. The EtherCAT
 *  interface is not here either: it comes from the `LXMSTR_RT_IFACE` env via
 *  `NetworkConfig::defaults()`. */
struct CommonOptions {
  bool debug = false;
};

enum class ArgParse { Consumed, NotMine, Error };

/**
 * Try to consume one common flag at `argv[i]`. Returns `NotMine` when the token is not a common
 * flag so the caller can fall through to its own parser; `Error` on a malformed flag.
 */
inline ArgParse parseCommonArg(int argc, char* argv[], int& i, CommonOptions& o) {
  (void)argc;
  if (!std::strcmp(argv[i], "--debug")) {
    o.debug = true;
    return ArgParse::Consumed;
  }
  return ArgParse::NotMine;
}

/**
 * Base config shared by the EcNetwork examples plus the debug flag. The cyclic period is left
 * unset (adopted from the ENI) and the sync mode is decided by the ENI, not the app. Callers
 * layer their own tuning on top of the returned config.
 */
inline ecnet::NetworkConfig makeBaseNetworkConfig(const CommonOptions& o) {
  ecnet::NetworkConfig cfg = ecnet::NetworkConfig::defaults();
  cfg.debug.debug = o.debug;
  return cfg;
}

}  // namespace apps_common
