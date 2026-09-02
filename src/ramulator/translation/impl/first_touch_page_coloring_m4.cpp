#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <fmt/format.h>

#include "ramulator/base/base.h"
#include "ramulator/base/param.h"
#include "ramulator/base/utils.h"
#include "ramulator/translation/i_translation.h"

namespace Ramulator {

class FirstTouchPageColoringM4 final : public ITranslation, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(
      ITranslation, FirstTouchPageColoringM4, "FirstTouchPageColoringM4");

  struct PageKey {
    int source_id;
    uint64_t virtual_page;
    bool operator==(const PageKey& other) const {
      return source_id == other.source_id && virtual_page == other.virtual_page;
    }
  };
  struct PageKeyHash {
    size_t operator()(const PageKey& key) const {
      return std::hash<uint64_t>{}(
          key.virtual_page ^ (static_cast<uint64_t>(static_cast<uint32_t>(key.source_id)) << 48));
    }
  };

  Addr_t m_max_paddr = 0;
  Addr_t m_page_size = 4096;
  int m_ba_bit_offset = 13;
  int m_num_channels = 1;
  Addr_t m_channel_interleave_size = 64;
  int m_channel_bit_offset = 6;
  int m_channel_bits = 0;
  int m_allocator_seed = 0;
  int m_num_cores = 0;
  std::vector<int> m_tolerant_bas;
  std::vector<int> m_reliable_bas;
  std::vector<int> m_tolerant_cores;
  std::unordered_set<int> m_tolerant_core_set;
  struct PageEntry {
    uint64_t physical_page;
    bool tolerant;
  };
  std::unordered_map<PageKey, PageEntry, PageKeyHash> m_page_table;
  std::unordered_set<uint64_t> m_allocated_physical_pages;
  uint64_t m_max_pages = 0;
  std::array<uint64_t, 4> m_next_candidate_page{};
  size_t m_next_tolerant_ba = 0;
  size_t m_next_reliable_ba = 0;

  size_t s_pages_tolerant = 0;
  size_t s_pages_reliable = 0;
  size_t s_pages_borrowed = 0;
  std::array<size_t, 4> s_pages_by_ba{};
  std::array<size_t, 4> s_tolerant_pages_by_ba{};
  std::array<size_t, 4> s_reliable_pages_by_ba{};
  size_t s_page_class_conflicts = 0;
  size_t s_physical_page_aliases = 0;
  size_t s_unique_physical_pages = 0;

  bool is_power_of_two(Addr_t value) const {
    return value > 0 && (value & (value - 1)) == 0;
  }
  Addr_t strip_channel_bits(Addr_t addr) const {
    if (m_channel_bits == 0) return addr;
    const Addr_t low_mask = (static_cast<Addr_t>(1) << m_channel_bit_offset) - 1;
    const Addr_t low = addr & low_mask;
    const Addr_t high = addr >> (m_channel_bit_offset + m_channel_bits);
    return (high << m_channel_bit_offset) | low;
  }
  int address_ba(Addr_t addr) const {
    return static_cast<int>((strip_channel_bits(addr) >> m_ba_bit_offset) & 0x3);
  }
  int page_ba(uint64_t physical_page) const {
    const Addr_t base = static_cast<Addr_t>(physical_page) * m_page_size;
    const int ba = address_ba(base);
    for (Addr_t offset = 0; offset < m_page_size; offset += m_channel_interleave_size) {
      if (address_ba(base + offset) != ba) {
        throw std::runtime_error(
            "FirstTouchPageColoringM4 page spans multiple BA quality classes");
      }
    }
    return ba;
  }
  bool allocate_from_ba(int ba, uint64_t& physical_page) {
    auto& candidate = m_next_candidate_page.at(ba);
    while (candidate < m_max_pages && page_ba(candidate) != ba) candidate++;
    if (candidate >= m_max_pages) return false;
    physical_page = candidate++;
    if (!m_allocated_physical_pages.insert(physical_page).second) {
      s_physical_page_aliases++;
      throw std::runtime_error(
          "FirstTouchPageColoringM4 attempted to reuse a physical page");
    }
    s_unique_physical_pages++;
    s_pages_by_ba.at(ba)++;
    return true;
  }
  bool allocate_from_set(
      const std::vector<int>& bas, size_t& next_index,
      uint64_t& physical_page, int& selected_ba) {
    for (size_t attempt = 0; attempt < bas.size(); attempt++) {
      const int ba = bas.at(next_index++ % bas.size());
      if (allocate_from_ba(ba, physical_page)) {
        selected_ba = ba;
        return true;
      }
    }
    return false;
  }

 public:
  void init() override {
    RAMULATOR_PARSE_PARAM(m_max_paddr, Addr_t, "max_addr").required();
    RAMULATOR_PARSE_PARAM(m_page_size, Addr_t, "page_size").default_val(4096);
    RAMULATOR_PARSE_PARAM(m_ba_bit_offset, int, "ba_bit_offset").default_val(13);
    RAMULATOR_PARSE_PARAM(m_num_channels, int, "channels").default_val(1);
    RAMULATOR_PARSE_PARAM(m_channel_interleave_size, Addr_t, "channel_interleave_size").default_val(64);
    RAMULATOR_PARSE_PARAM(m_allocator_seed, int, "seed").default_val(0);
    RAMULATOR_PARSE_PARAM(m_tolerant_bas, std::vector<int>, "tolerant_bas").required();
    RAMULATOR_PARSE_PARAM(m_num_cores, int, "num_cores").required();
    RAMULATOR_PARSE_PARAM(m_tolerant_cores, std::vector<int>, "tolerant_cores").required();

    if (!is_power_of_two(m_page_size)) {
      throw std::runtime_error("FirstTouchPageColoringM4 page_size must be a power of two");
    }
    if (m_max_paddr < m_page_size || m_max_paddr % m_page_size != 0) {
      throw std::runtime_error("FirstTouchPageColoringM4 max_addr must be page aligned");
    }
    if (m_num_cores <= 0) {
      throw std::runtime_error("FirstTouchPageColoringM4 num_cores must be positive");
    }
    if (m_num_channels <= 0 || !is_power_of_two(m_num_channels)) {
      throw std::runtime_error(
          "FirstTouchPageColoringM4 channels must be a positive power of two");
    }
    if (!is_power_of_two(m_channel_interleave_size) ||
        m_channel_interleave_size > m_page_size) {
      throw std::runtime_error(
          "FirstTouchPageColoringM4 channel interleave must be a power of two no larger than a page");
    }
    if (m_allocator_seed != 0) {
      throw std::runtime_error(
          "FirstTouchPageColoringM4 deterministic M4 allocator requires seed 0");
    }
    m_channel_bit_offset = calc_log2(m_channel_interleave_size);
    m_channel_bits = calc_log2(m_num_channels);
    std::sort(m_tolerant_bas.begin(), m_tolerant_bas.end());
    if (m_tolerant_bas.empty() || m_tolerant_bas.size() >= 4 ||
        std::adjacent_find(m_tolerant_bas.begin(), m_tolerant_bas.end()) != m_tolerant_bas.end() ||
        m_tolerant_bas.front() < 0 || m_tolerant_bas.back() >= 4) {
      throw std::runtime_error(
          "FirstTouchPageColoringM4 tolerant_bas must contain one to three unique BAs in [0, 3]");
    }
    for (int ba = 0; ba < 4; ba++) {
      if (std::find(m_tolerant_bas.begin(), m_tolerant_bas.end(), ba) == m_tolerant_bas.end()) {
        m_reliable_bas.push_back(ba);
      }
    }
    m_max_pages = static_cast<uint64_t>(m_max_paddr / m_page_size);
    m_next_candidate_page.fill(0);
    if (m_ba_bit_offset < calc_log2(m_page_size)) {
      throw std::runtime_error(
          "FirstTouchPageColoringM4 requires both BA bits outside the page offset");
    }
    for (int core : m_tolerant_cores) {
      if (core < 0 || core >= m_num_cores || !m_tolerant_core_set.insert(core).second) {
        throw std::runtime_error(
            "FirstTouchPageColoringM4 tolerant_cores must be unique valid core ids");
      }
    }

    m_stats.add("pages_tolerant", s_pages_tolerant);
    m_stats.add("pages_reliable", s_pages_reliable);
    m_stats.add("pages_borrowed", s_pages_borrowed);
    m_stats.add("page_class_conflicts", s_page_class_conflicts);
    m_stats.add("physical_page_aliases", s_physical_page_aliases);
    m_stats.add("unique_physical_pages", s_unique_physical_pages);
    m_stats.add("allocator_seed", m_allocator_seed);
    m_stats.add("allocator_channels", m_num_channels);
    m_stats.add("allocator_max_addr", m_max_paddr);
    m_stats.add("allocator_channel_interleave_size", m_channel_interleave_size);
    m_stats.add("allocator_intra_channel_ba_bit_offset", m_ba_bit_offset);
    for (int ba = 0; ba < 4; ba++) {
      m_stats.add(fmt::format("pages_ba_{}", ba), s_pages_by_ba.at(ba));
      m_stats.add(
          fmt::format("tolerant_pages_ba_{}", ba), s_tolerant_pages_by_ba.at(ba));
      m_stats.add(
          fmt::format("reliable_pages_ba_{}", ba), s_reliable_pages_by_ba.at(ba));
    }
  }

  bool translate(Request& req) override {
    if (req.source_id < 0 || req.source_id >= m_num_cores) {
      throw std::runtime_error("FirstTouchPageColoringM4 source_id is outside num_cores");
    }
    const Addr_t virtual_addr = req.addr;
    const uint64_t virtual_page = static_cast<uint64_t>(virtual_addr / m_page_size);
    const Addr_t page_offset = virtual_addr & (m_page_size - 1);
    const PageKey key{req.source_id, virtual_page};
    auto found = m_page_table.find(key);
    if (found == m_page_table.end()) {
      const bool tolerant = req.memory_class >= 0
                                ? req.memory_class == 1
                                : m_tolerant_core_set.count(req.source_id) != 0;
      uint64_t physical_page = 0;
      int selected_ba = -1;
      if (tolerant) {
        if (!allocate_from_set(
                m_tolerant_bas, m_next_tolerant_ba,
                physical_page, selected_ba)) {
          if (!allocate_from_set(
                  m_reliable_bas, m_next_reliable_ba,
                  physical_page, selected_ba)) {
            throw std::runtime_error("FirstTouchPageColoringM4 exhausted physical pages");
          }
          s_pages_borrowed++;
        }
        s_pages_tolerant++;
        s_tolerant_pages_by_ba.at(selected_ba)++;
      } else {
        if (!allocate_from_set(
                m_reliable_bas, m_next_reliable_ba,
                physical_page, selected_ba)) {
          throw std::runtime_error(
              "FirstTouchPageColoringM4 exhausted reliable physical pages");
        }
        s_pages_reliable++;
        s_reliable_pages_by_ba.at(selected_ba)++;
      }
      found = m_page_table.emplace(key, PageEntry{physical_page, tolerant}).first;
    } else if (req.memory_class == 0 && found->second.tolerant) {
      s_page_class_conflicts++;
      throw std::runtime_error(
          "FirstTouchPageColoringM4 reliable access aliases a tolerant physical page");
    }
    req.memory_class = found->second.tolerant ? 1 : 0;
    req.addr = static_cast<Addr_t>(found->second.physical_page) * m_page_size + page_offset;
    return true;
  }
};

}  // namespace Ramulator
