#include "ramulator/frontend/impl/processor/champsimO3/core.h"

#include <algorithm>
#include <stdexcept>

#include "ramulator/frontend/impl/processor/simpleO3/llc.h"

namespace Ramulator {

ChampSimO3Core::InstWindow::InstWindow(int ipc, int depth)
    : m_ipc(ipc), m_depth(depth), m_entries(depth) {
  if (ipc <= 0 || depth <= 0) {
    throw std::runtime_error("ChampSimO3 ipc and inst_window_depth must be positive");
  }
}

void ChampSimO3Core::InstWindow::insert(uint64_t instruction_id, int pending_memory) {
  if (is_full()) {
    throw std::runtime_error("ChampSimO3 instruction window overflow");
  }
  m_entries.at(m_head_idx) = {instruction_id, pending_memory, true};
  m_head_idx = (m_head_idx + 1) % m_depth;
  m_load++;
}

void ChampSimO3Core::InstWindow::complete(uint64_t instruction_id) {
  int index = m_tail_idx;
  for (int count = 0; count < m_load; count++) {
    auto& entry = m_entries.at(index);
    if (entry.valid && entry.instruction_id == instruction_id) {
      if (entry.pending_memory <= 0) {
        throw std::runtime_error("ChampSimO3 completed an instruction memory operation twice");
      }
      entry.pending_memory--;
      return;
    }
    index = (index + 1) % m_depth;
  }
  throw std::runtime_error("ChampSimO3 received a completion for an instruction outside the window");
}

int ChampSimO3Core::InstWindow::retire() {
  int retired = 0;
  while (m_load > 0 && retired < m_ipc) {
    auto& entry = m_entries.at(m_tail_idx);
    if (!entry.valid || entry.pending_memory != 0) {
      break;
    }
    entry = {};
    m_tail_idx = (m_tail_idx + 1) % m_depth;
    m_load--;
    retired++;
  }
  return retired;
}

ChampSimO3Core::ChampSimO3Core(const Clk_t& clk, int id, int ipc, int depth,
                               size_t warmup_insts, size_t num_expected_insts,
                               const std::string& trace_path,
                               const std::string& sidecar_path,
                               ITranslation* translation, SimpleO3LLC* llc)
    : m_clk(clk),
      m_id(id),
      m_ipc(ipc),
      m_num_expected_insts(warmup_insts + num_expected_insts),
      m_warmup_insts(warmup_insts),
      m_roi_insts(num_expected_insts),
      m_trace(trace_path),
      m_sidecar(sidecar_path.empty() ? nullptr : std::make_unique<ObjectSidecar>(sidecar_path)),
      m_window(ipc, depth),
      m_translation(translation),
      m_llc(llc),
      m_current(m_trace.next()) {
  if (num_expected_insts == 0) {
    throw std::runtime_error("ChampSimO3 num_expected_insts must be positive");
  }
  if (m_sidecar) {
    m_sidecar->advance(m_trace.passes_completed(), m_trace.record_index_this_pass());
  }
  if (m_warmup_insts == 0) begin_roi();
}

void ChampSimO3Core::begin_roi() {
  m_roi_started = true;
  reached_warmup = true;
  m_roi_start_insts = s_insts_retired;
  m_num_expected_insts = m_roi_start_insts + m_roi_insts;
  m_roi_start_clk = static_cast<size_t>(m_clk);
}

size_t ChampSimO3Core::roi_insts_retired() const {
  if (s_insts_retired <= m_roi_start_insts) return 0;
  return std::min(s_insts_retired - m_roi_start_insts, m_roi_insts);
}

int ChampSimO3Core::current_memory_count() const {
  int count = 0;
  count += std::count_if(m_current.source_memory.begin(), m_current.source_memory.end(),
                         [](uint64_t addr) { return addr != 0; });
  count += std::count_if(m_current.destination_memory.begin(), m_current.destination_memory.end(),
                         [](uint64_t addr) { return addr != 0; });
  return count;
}

void ChampSimO3Core::advance_trace() {
  m_current = m_trace.next();
  if (m_sidecar) {
    m_sidecar->advance(m_trace.passes_completed(), m_trace.record_index_this_pass());
  }
  m_next_load = 0;
  m_next_store = 0;
  m_current_inserted = false;
}

void ChampSimO3Core::tick() {
  s_insts_retired += m_window.retire();
  if (!m_roi_started && s_insts_retired >= m_warmup_insts) {
    reached_warmup = true;
    return;
  }
  if (m_roi_started && !reached_expected_num_insts &&
      s_insts_retired >= m_num_expected_insts) {
    reached_expected_num_insts = true;
    s_cycles_recorded = static_cast<size_t>(m_clk) - m_roi_start_clk;
  }
  if (reached_expected_num_insts) {
    return;
  }

  int instruction_budget = m_ipc;
  int memory_budget = m_ipc;
  while (instruction_budget > 0) {
    if (!m_current_inserted) {
      if (m_window.is_full()) {
        return;
      }
      m_current_id = m_next_instruction_id++;
      m_window.insert(m_current_id, current_memory_count());
      m_current_inserted = true;
      s_trace_records++;
      instruction_budget--;
    }

    while (m_next_load < m_current.source_memory.size()) {
      uint64_t addr = m_current.source_memory.at(m_next_load);
      if (addr == 0) {
        m_next_load++;
        continue;
      }
      if (memory_budget == 0) {
        return;
      }
      Request req(static_cast<Addr_t>(addr), Request::Type::Read, m_id, m_callback);
      req.ingress_id = static_cast<int>(m_current_id);
      if (m_sidecar) req.memory_class = m_sidecar->classify(addr);
      if (!m_translation->translate(req) || !m_llc->send(req)) {
        return;
      }
      m_next_load++;
      memory_budget--;
    }

    while (m_next_store < m_current.destination_memory.size()) {
      uint64_t addr = m_current.destination_memory.at(m_next_store);
      if (addr == 0) {
        m_next_store++;
        continue;
      }
      if (memory_budget == 0) {
        return;
      }
      Request req(static_cast<Addr_t>(addr), Request::Type::Write, m_id, m_callback);
      req.ingress_id = -1;
      if (m_sidecar) req.memory_class = m_sidecar->classify(addr);
      if (!m_translation->translate(req) || !m_llc->send(req)) {
        return;
      }
      m_window.complete(m_current_id);
      m_next_store++;
      memory_budget--;
    }

    advance_trace();
  }
}

void ChampSimO3Core::receive(Request& req) {
  if (req.ingress_id >= 0) {
    m_window.complete(static_cast<uint64_t>(req.ingress_id));
  }
}

}  // namespace Ramulator
