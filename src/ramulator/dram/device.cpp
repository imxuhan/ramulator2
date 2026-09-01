#include "ramulator/dram/device.h"

namespace Ramulator {

void DRAMDevice::init(std::unique_ptr<DRAMSpec> spec) {
  m_spec_owner = std::move(spec);
  m_spec = m_spec_owner.get();
  m_bank_level = m_spec->get_level_id("Bank");
  m_root = std::make_unique<DRAMNode>(m_spec, nullptr, 0, 0);
  m_root->for_each_at_level(m_bank_level, [&](DRAMNode* bank) { m_bank_nodes.push_back(bank); });
}

void DRAMDevice::set_channel_id(int channel_id) {
  m_root->m_node_id = channel_id;
}

void DRAMDevice::issue_command(int command, const AddrVec_t& addr_vec, Clk_t clk,
                               const AddrVec_t* secondary_addr_vec) {
  m_root->update_timing(command, addr_vec, clk);
  if (m_spec->bank_targets[command] == BankTarget::Pair) {
    validate_pair_addr_vecs(addr_vec, secondary_addr_vec);
    int second = get_flat_bank_id(*secondary_addr_vec);
    DRAMNode* second_bg = m_bank_nodes[second]->m_parent_node;
    second_bg->update_timing(command, *secondary_addr_vec, clk);
  }
  apply_action(command, addr_vec, clk, secondary_addr_vec);
}

bool DRAMDevice::check_timing(int command, const AddrVec_t& addr_vec, Clk_t clk,
                              const AddrVec_t* secondary_addr_vec) {
  if (!m_root->check_timing(command, addr_vec, clk)) return false;
  if (m_spec->bank_targets[command] != BankTarget::Pair) return true;

  validate_pair_addr_vecs(addr_vec, secondary_addr_vec);
  int second = get_flat_bank_id(*secondary_addr_vec);
  DRAMNode* second_bg = m_bank_nodes[second]->m_parent_node;
  return second_bg->check_timing(command, *secondary_addr_vec, clk);
}

int DRAMDevice::get_preq_command(int command, const AddrVec_t& addr_vec, Clk_t clk,
                                const AddrVec_t* secondary_addr_vec) {
  auto preq_fn = m_spec->funcs.preqs[command];
  if (!preq_fn) return command;

  int resolved = command;
  for_each_target_bank_while(command, addr_vec, secondary_addr_vec, [&](int flat_bank_id) {
    int preq = preq_fn(m_bank_nodes[flat_bank_id], command, addr_vec, clk);
    if (preq != command) { resolved = preq; return false; }
    return true;
  });
  return resolved;
}

bool DRAMDevice::check_rowbuffer_hit(int command, const AddrVec_t& addr_vec, Clk_t clk) {
  auto rowhit_fn = m_spec->funcs.rowhits[command];
  if (!rowhit_fn) {
    return false;
  }
  int flat_bank_id = get_flat_bank_id(addr_vec);
  return rowhit_fn(m_bank_nodes[flat_bank_id], command, addr_vec, clk);
}

bool DRAMDevice::check_node_open(int command, const AddrVec_t& addr_vec, Clk_t clk) {
  auto rowopen_fn = m_spec->funcs.rowopens[command];
  if (!rowopen_fn) {
    return false;
  }
  int flat_bank_id = get_flat_bank_id(addr_vec);
  return rowopen_fn(m_bank_nodes[flat_bank_id], command, addr_vec, clk);
}

int DRAMDevice::get_flat_bank_id(const AddrVec_t& addr_vec) const {
  int id = 0;
  for (int lvl = 1; lvl <= m_bank_level; lvl++) {
    id = id * m_spec->organization.level_sizes[lvl] + addr_vec[lvl];
  }
  return id;
}

bool DRAMDevice::bank_matches(DRAMNode* bank, const AddrVec_t& addr_vec) {
  for (auto* n = bank; n != nullptr; n = n->m_parent_node) {
    if (addr_vec[n->m_level] != -1 && addr_vec[n->m_level] != n->m_node_id) {
      return false;
    }
  }
  return true;
}

std::vector<int> DRAMDevice::get_target_banks(int command, const AddrVec_t& addr_vec) const {
  return get_target_banks(command, addr_vec, nullptr);
}

std::vector<int> DRAMDevice::get_target_banks(int command, const AddrVec_t& addr_vec,
                                              const AddrVec_t* secondary_addr_vec) const {
  std::vector<int> ids;
  for_each_target_bank(command, addr_vec, secondary_addr_vec, [&](int id) { ids.push_back(id); });
  return ids;
}

void DRAMDevice::validate_pair_addr_vecs(const AddrVec_t& addr_vec,
                                         const AddrVec_t* secondary_addr_vec) const {
  if (secondary_addr_vec == nullptr || secondary_addr_vec->size() != addr_vec.size() ||
      addr_vec.size() != static_cast<size_t>(m_spec->level_count)) {
    throw std::runtime_error("Pair-target command requires two complete address vectors");
  }

  int bg_level = m_spec->get_level_id("BankGroup");
  for (int level = 0; level <= m_bank_level; level++) {
    int first = addr_vec[level];
    int second = (*secondary_addr_vec)[level];
    if (first < 0 || second < 0) {
      throw std::runtime_error("Pair-target command requires concrete channel/rank/bank addresses");
    }
    if (level == bg_level) {
      if (first == second) {
        throw std::runtime_error("Pair-target command requires two different bank groups");
      }
    } else if (first != second) {
      throw std::runtime_error("Pair-target command requires the same channel, rank, and bank address");
    }
  }
}

void DRAMDevice::apply_action(int command, const AddrVec_t& addr_vec, Clk_t clk,
                              const AddrVec_t* secondary_addr_vec) {
  auto action_fn = m_spec->funcs.actions[command];
  if (!action_fn) return;
  for_each_target_bank(command, addr_vec, secondary_addr_vec, [&](int flat_bank_id) {
    action_fn(m_bank_nodes[flat_bank_id], command, addr_vec, clk);
  });
}

}  // namespace Ramulator
