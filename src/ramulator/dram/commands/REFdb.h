#ifndef RAMULATOR_DRAM_COMMANDS_REFDB_H
#define RAMULATOR_DRAM_COMMANDS_REFDB_H

#include "ramulator/dram/node.h"

namespace Ramulator::Cmd {

// LPDDR6 dual-bank refresh. The request carries two explicit addresses with
// different bank groups and the same bank address.
template <class T>
struct REFdb {
  static constexpr DRAMCommandMeta meta = {.is_refreshing = true};
  static constexpr BankTarget bank_target = BankTarget::Pair;
};

}  // namespace Ramulator::Cmd

#endif  // RAMULATOR_DRAM_COMMANDS_REFDB_H
