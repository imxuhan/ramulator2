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

class FirstTouchPageColoring final : public ITranslation, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(ITranslation, FirstTouchPageColoring, "FirstTouchPageColoring");

  struct PageKey {
    int source_id;
    uint64_t virtual_page;

    bool operator==(const PageKey& other) const {
      return source_id == other.source_id && virtual_page == other.virtual_page;
    }
  };

  struct PageKeyHash {
    size_t operator()(const PageKey& key) const {
      auto source = static_cast<uint64_t>(static_cast<uint32_t>(key.source_id));
      return std::hash<uint64_t>{}(key.virtual_page ^ (source << 48));
    }
  };

  Addr_t m_max_paddr = 0;
  Addr_t m_page_size = 4096;
  int m_ba_bit_offset = 13;
  int m_reduced_ba = 0;
  int m_num_cores = 0;
  std::vector<int> m_tolerant_cores;
  std::unordered_set<int> m_tolerant_core_set;
  std::unordered_map<PageKey, uint64_t, PageKeyHash> m_page_table;
  uint64_t m_pages_per_core = 0;
  std::vector<std::array<uint64_t, 4>> m_next_candidate_page;
  std::vector<size_t> m_next_reliable_ba;

  size_t s_pages_tolerant = 0;
  size_t s_pages_reliable = 0;
  size_t s_pages_borrowed = 0;
  std::array<size_t, 4> s_pages_by_ba{};

  bool is_power_of_two(Addr_t value) const {
    return value > 0 && (value & (value - 1)) == 0;
  }

  int page_ba(uint64_t physical_page) const {
    Addr_t addr = static_cast<Addr_t>(physical_page) * m_page_size;
    return static_cast<int>((addr >> m_ba_bit_offset) & 0x3);
  }

  bool allocate_from_ba(int source_id, int ba, uint64_t& physical_page) {
    uint64_t region_end = static_cast<uint64_t>(source_id + 1) * m_pages_per_core;
    auto& candidate = m_next_candidate_page.at(source_id).at(ba);
    while (candidate < region_end && page_ba(candidate) != ba) {
      candidate++;
    }
    if (candidate >= region_end) {
      return false;
    }
    physical_page = candidate++;
    s_pages_by_ba.at(ba)++;
    return true;
  }

  bool allocate_reliable(int source_id, uint64_t& physical_page) {
    for (size_t attempt = 0; attempt < 3; attempt++) {
      auto& next_ba = m_next_reliable_ba.at(source_id);
      int ba = static_cast<int>(next_ba++ % 4);
      if (ba == m_reduced_ba) {
        ba = static_cast<int>(next_ba++ % 4);
      }
      if (ba != m_reduced_ba && allocate_from_ba(source_id, ba, physical_page)) {
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
    RAMULATOR_PARSE_PARAM(m_reduced_ba, int, "reduced_ba").default_val(0);
    RAMULATOR_PARSE_PARAM(m_num_cores, int, "num_cores").required();
    RAMULATOR_PARSE_PARAM(m_tolerant_cores, std::vector<int>, "tolerant_cores").required();

    if (!is_power_of_two(m_page_size)) {
      throw std::runtime_error("FirstTouchPageColoring page_size must be a power of two");
    }
    if (m_max_paddr < m_page_size || m_max_paddr % m_page_size != 0) {
      throw std::runtime_error("FirstTouchPageColoring max_addr must be page aligned");
    }
    if (m_reduced_ba < 0 || m_reduced_ba >= 4) {
      throw std::runtime_error("FirstTouchPageColoring reduced_ba must be in [0, 3]");
    }
    if (m_num_cores <= 0) {
      throw std::runtime_error("FirstTouchPageColoring num_cores must be positive");
    }
    uint64_t max_pages = static_cast<uint64_t>(m_max_paddr / m_page_size);
    m_pages_per_core = max_pages / static_cast<uint64_t>(m_num_cores);
    if (m_pages_per_core == 0 || max_pages % static_cast<uint64_t>(m_num_cores) != 0) {
      throw std::runtime_error("FirstTouchPageColoring physical pages must divide evenly across cores");
    }
    m_next_candidate_page.resize(m_num_cores);
    m_next_reliable_ba.resize(m_num_cores, 0);
    for (int core = 0; core < m_num_cores; core++) {
      m_next_candidate_page.at(core).fill(static_cast<uint64_t>(core) * m_pages_per_core);
    }
    int page_offset_bits = calc_log2(m_page_size);
    if (m_ba_bit_offset < page_offset_bits) {
      throw std::runtime_error(
          "FirstTouchPageColoring requires the two BA bits to be outside the page offset");
    }
    for (int core : m_tolerant_cores) {
      if (core < 0 || core >= m_num_cores || !m_tolerant_core_set.insert(core).second) {
        throw std::runtime_error("FirstTouchPageColoring tolerant_cores must be unique non-negative ids");
      }
    }

    m_stats.add("pages_tolerant", s_pages_tolerant);
    m_stats.add("pages_reliable", s_pages_reliable);
    m_stats.add("pages_borrowed", s_pages_borrowed);
    for (int ba = 0; ba < 4; ba++) {
      m_stats.add(fmt::format("pages_ba_{}", ba), s_pages_by_ba.at(ba));
    }
  }

  bool translate(Request& req) override {
    if (req.source_id < 0 || req.source_id >= m_num_cores) {
      throw std::runtime_error("FirstTouchPageColoring request source_id is outside num_cores");
    }
    Addr_t virtual_addr = req.addr;
    uint64_t virtual_page = static_cast<uint64_t>(virtual_addr / m_page_size);
    Addr_t page_offset = virtual_addr & (m_page_size - 1);
    PageKey key{req.source_id, virtual_page};

    auto found = m_page_table.find(key);
    if (found == m_page_table.end()) {
      bool tolerant = m_tolerant_core_set.count(req.source_id) != 0;
      uint64_t physical_page = 0;
      if (tolerant) {
        if (!allocate_from_ba(req.source_id, m_reduced_ba, physical_page)) {
          if (!allocate_reliable(req.source_id, physical_page)) {
            throw std::runtime_error("FirstTouchPageColoring exhausted physical pages");
          }
          s_pages_borrowed++;
        }
        s_pages_tolerant++;
      } else {
        if (!allocate_reliable(req.source_id, physical_page)) {
          throw std::runtime_error("FirstTouchPageColoring exhausted reliable physical pages");
        }
        s_pages_reliable++;
      }
      found = m_page_table.emplace(key, physical_page).first;
    }

    req.addr = static_cast<Addr_t>(found->second) * m_page_size + page_offset;
    return true;
  }
};

}  // namespace Ramulator
