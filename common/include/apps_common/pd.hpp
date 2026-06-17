#pragma once

#include "ecmaster/ec_master.hpp"

/**
 * Process-data warm-up helper shared by the SAFE_OP examples. Before SYNC0 is attached (and
 * after each SYNC0 retune) the drive needs to see a handful of SM2/SM3 events, so every
 * example ran the same `send → receive` prime loop. One copy here.
 */
namespace apps_common {

/** Exchange `cycles` PDO frames back-to-back to warm the bus (SM2/SM3 events before SYNC0). */
inline void primeProcessDataCycles(ecmaster::EcMaster& master, int cycles) {
  for (int i = 0; i < cycles; ++i) {
    master.sendProcessData();
    master.receiveProcessData(ecmaster::kTimeoutRet);
  }
}

}  // namespace apps_common
