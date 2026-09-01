#include "ramulator/frontend/impl/processor/simpleO3/llc.h"

#include <cassert>
#include <algorithm>
#include <fstream>

namespace Ramulator {

SimpleO3LLC::SimpleO3LLC(const Clk_t& clk, int latency, int size_bytes, int linesize_bytes, int associativity,
                         int num_mshrs)
    : m_clk(clk),
      m_latency(latency),
      m_size_bytes(size_bytes),
      m_linesize_bytes(linesize_bytes),
      m_associativity(associativity),
      m_num_mshrs(num_mshrs) {
  m_logger = Logger("SimpleO3LLC");

  m_set_size = m_size_bytes / (m_linesize_bytes * m_associativity);
  m_index_mask = m_set_size - 1;
  m_index_offset = calc_log2(m_linesize_bytes);
  m_tag_offset = calc_log2(m_set_size) + m_index_offset;

  DEBUG_LOG(m_logger, "Index mask: {0:x}", m_index_mask);
  DEBUG_LOG(m_logger, "Index offset: {}", m_index_offset);
  DEBUG_LOG(m_logger, "Tag offset: {}", m_tag_offset);
};

void SimpleO3LLC::tick() {
  // Send miss requests to the memory system when LLC latency is met
  // TODO: Optimization by assuming in-order issue?
  auto it = m_miss_list.begin();
  while (it != m_miss_list.end()) {
    if (m_clk >= it->first) {
      if (!m_memory_system->send(it->second)) {
        it++;
      } else {
        it = m_miss_list.erase(it);
      }
    } else {
      it++;
    }
  }

  // call hit request callback when LLC latency is met
  it = m_hit_list.begin();
  while (it != m_hit_list.end()) {
    if (m_clk >= it->first) {
      std::vector<Request> _req_v{it->second};
      m_receive_requests[align(it->second.addr)] = _req_v;

      it->second.callback(it->second);
      it = m_hit_list.erase(it);
    } else {
      it++;
    }
  }
};

bool SimpleO3LLC::send(Request& req) {
  CacheSet_t& set = get_set(req.addr);

  if (req.type_id == Request::Type::Read) {
    s_llc_read_access++;
  } else if (req.type_id == Request::Type::Write) {
    s_llc_write_access++;
  }

  if (auto line_it = check_set_hit(set, req.addr); line_it != set.end()) {
    // Hit in the set
    DEBUG_LOG(m_logger,
              "[Clk={}] Request Source: {}, Type: {}, Addr: {}, Index: {}, Tag: {}. Hit, will finish at Clk={}", m_clk,
              req.source_id, req.type_id, req.addr, get_index(req.addr), get_tag(req.addr), m_clk + m_latency);

    // Update the LRU status
    set.push_back({req.addr, get_tag(req.addr), line_it->dirty || (req.type_id == Request::Type::Write), true});
    set.erase(line_it);

    // Add to the hit list to callback when finished
    m_hit_list.push_back(std::make_pair(m_clk + m_latency, req));
    return true;
  } else {
    // Miss in the set
    DEBUG_LOG(m_logger, "[Clk={}] Request Source: {}, Type: {}, Addr: {}, Index: {}, Tag: {}. Miss.", m_clk,
              req.source_id, req.type_id, req.addr, get_index(req.addr), get_tag(req.addr));

    if (req.type_id == Request::Type::Read) {
      s_llc_read_misses++;
    } else if (req.type_id == Request::Type::Write) {
      s_llc_write_misses++;
    }

    bool dirty = (req.type_id == Request::Type::Write);
    if (req.type_id == Request::Type::Write) {
      req.type_id = Request::Type::Read;
    }

    // MSHR lookup
    auto mshr_it = check_mshr_hit(req.addr);
    if (mshr_it != m_mshrs.end()) {
      DEBUG_LOG(m_logger, "MSHR Hit.", m_clk);
      // Add new req to MSHR_requests
      m_receive_requests[mshr_it->line_addr].push_back(req);

      mshr_it->line->dirty = dirty || mshr_it->line->dirty;
      return true;
    }

    // MSHR miss
    // Check if there is available MSHR entry
    if (m_mshrs.size() == m_num_mshrs) {
      DEBUG_LOG(m_logger, "No MSHR entry available.", m_clk);
      s_llc_mshr_unavailable++;
      return false;
    }

    // Check if there is available cache line in the set
    bool line_available = false;
    if (set.size() < m_associativity) {
      line_available = true;
    } else {
      for (const auto& line : set) {
        if (line.ready) {
          line_available = true;
        }
      }
    }
    if (!line_available) {
      DEBUG_LOG(m_logger, "No cache line available in the set.", m_clk);
      return false;
    }

    // Allocate a new cache line
    auto newline_it = allocate_line(set, req.addr);
    if (newline_it == set.end()) {
      throw std::runtime_error("Failed to allocate new line when there is available entry.");
    }
    newline_it->dirty = dirty;

    // Add to MSHR entries
    int transaction_count = memory_transactions_per_line();
    m_mshrs.push_back({align(req.addr), newline_it, transaction_count});
    // Add Request to MSHR_requests
    std::vector<Request> _req_v{req};
    m_receive_requests[align(req.addr)] = _req_v;

    // Add one request per physical DRAM transaction. The MSHR remains locked
    // until all transactions for this cache line return.
    enqueue_line_transactions(req, Request::Type::Read, m_clk + m_latency);

    return true;
  }
};

bool SimpleO3LLC::receive(Request& req) {
  Addr_t line_addr = align(req.addr);
  auto it = std::find_if(m_mshrs.begin(), m_mshrs.end(),
                         [line_addr](const MSHREntry_t& entry) { return entry.line_addr == line_addr; });

  DEBUG_LOG(m_logger, "[Clk={}] Request {} received.", m_clk, req.addr);

  if (it != m_mshrs.end()) {
    if (--it->pending_transactions == 0) {
      it->line->ready = true;
      m_mshrs.erase(it);
      return true;
    }
    return false;
  }
  // LLC hits have no MSHR and are complete when their callback fires.
  return true;
};

int SimpleO3LLC::memory_transactions_per_line() const {
  int tx_bytes = m_memory_system->get_tx_bytes();
  if (tx_bytes <= 0 || m_linesize_bytes % static_cast<size_t>(tx_bytes) != 0) {
    throw std::runtime_error("LLC line size must be a positive multiple of the DRAM transaction size");
  }
  return static_cast<int>(m_linesize_bytes / static_cast<size_t>(tx_bytes));
}

void SimpleO3LLC::enqueue_line_transactions(const Request& req, int type_id, Clk_t ready_clk) {
  int tx_bytes = m_memory_system->get_tx_bytes();
  Addr_t line_addr = align(req.addr);
  for (int offset = 0; offset < static_cast<int>(m_linesize_bytes); offset += tx_bytes) {
    Request transaction = req;
    transaction.addr = line_addr + offset;
    transaction.type_id = type_id;
    transaction.size_bytes = tx_bytes;
    m_miss_list.push_back(std::make_pair(ready_clk, transaction));
  }
}

SimpleO3LLC::CacheSet_t& SimpleO3LLC::get_set(Addr_t addr) {
  int set_index = get_index(addr);
  if (m_cache_sets.find(set_index) == m_cache_sets.end()) {
    m_cache_sets.insert(make_pair(set_index, std::list<Line>()));
  }
  return m_cache_sets[set_index];
}

SimpleO3LLC::CacheSet_t::iterator SimpleO3LLC::allocate_line(CacheSet_t& set, Addr_t addr) {
  // Check if we need to evict any line
  if (need_eviction(set, addr)) {
    // Get a victim to evict
    auto victim = std::find_if(set.begin(), set.end(), [this](Line line) { return line.ready; });
    if (victim == set.end()) {
      return victim;  // doesn't exist a line that's already unlocked in each level
    }
    evict_line(set, victim);
  }

  // Allocate new cache line and return an iterator to it
  set.push_back({addr, get_tag(addr)});
  return --set.end();
}

bool SimpleO3LLC::need_eviction(const CacheSet_t& set, Addr_t addr) {
  if (std::find_if(set.begin(), set.end(), [addr, this](Line l) { return (get_tag(addr) == l.tag); }) != set.end()) {
    // Due to MSHR, the program can't reach here. Just for checking
    assert(false);
    return false;
  } else {
    if (set.size() < m_associativity) {
      return false;
    } else {
      return true;
    }
  }
}

void SimpleO3LLC::evict_line(CacheSet_t& set, CacheSet_t::iterator victim_it) {
  DEBUG_LOG(m_logger, "Evicting {}.", victim_it->addr);
  s_llc_eviction++;

  // Generate writeback request if victim line is dirty
  if (victim_it->dirty) {
    Request writeback_req(victim_it->addr, Request::Type::Write);
    enqueue_line_transactions(writeback_req, Request::Type::Write, m_clk + m_latency);

    DEBUG_LOG(m_logger, "Writeback Request will be issued at Clk={}.", m_clk + m_latency);
  }

  set.erase(victim_it);
}

SimpleO3LLC::CacheSet_t::iterator SimpleO3LLC::check_set_hit(CacheSet_t& set, Addr_t addr) {
  auto line_it = std::find_if(set.begin(), set.end(), [addr, this](Line l) { return (l.tag == get_tag(addr)); });
  if (line_it == set.end() || !line_it->ready) {
    return set.end();
  }
  return line_it;
}

SimpleO3LLC::MSHR_t::iterator SimpleO3LLC::check_mshr_hit(Addr_t addr) {
  auto mshr_it = std::find_if(m_mshrs.begin(), m_mshrs.end(), [addr, this](MSHREntry_t mshr_entry) {
    return mshr_entry.line_addr == align(addr);
  });
  return mshr_it;
}

void SimpleO3LLC::serialize(std::string serialization_filename) {
  std::ofstream serialization_file;
  serialization_file.open(serialization_filename, std::ios::out);

  serialization_file << "index,addr,tag,dirty" << std::endl;
  for (auto it1 = m_cache_sets.begin(); it1 != m_cache_sets.end(); it1++) {
    for (auto it2 = it1->second.begin(); it2 != it1->second.end(); it2++) {
      serialization_file << it1->first << "," << it2->addr << "," << it2->tag << "," << it2->dirty << std::endl;
    }
  }
  serialization_file.close();
}

void SimpleO3LLC::deserialize(std::string serialization_filename) {
  std::ifstream serialization_file;
  serialization_file.open(serialization_filename, std::ios::in);

  std::string file_line;
  std::getline(serialization_file, file_line);  // Skip the first line, which is the header
  while (std::getline(serialization_file, file_line)) {
    std::string index_str = file_line.substr(0, file_line.find(","));
    file_line = file_line.substr(file_line.find(",") + 1);
    std::string addr_str = file_line.substr(0, file_line.find(","));
    file_line = file_line.substr(file_line.find(",") + 1);
    std::string tag_str = file_line.substr(0, file_line.find(","));
    file_line = file_line.substr(file_line.find(",") + 1);
    std::string dirty_str = file_line.substr(0, file_line.find(","));

    int index = std::stoi(index_str);
    Addr_t addr = std::stoll(addr_str);
    Addr_t tag = std::stoll(tag_str);
    bool dirty = std::stoi(dirty_str);
    if (m_cache_sets.find(index) == m_cache_sets.end()) {
      m_cache_sets.insert({index, std::list<SimpleO3LLC::Line>()});
    }
    m_cache_sets[index].push_back({addr, tag, dirty, 1});
  }
  serialization_file.close();
}

void SimpleO3LLC::dump_llc() {
  DEBUG_LOG(m_logger, "Dumping LLC");
  DEBUG_LOG(m_logger, "index,addr,tag,dirty,ready");
  for (auto it1 = m_cache_sets.begin(); it1 != m_cache_sets.end(); it1++) {
    for (auto it2 = it1->second.begin(); it2 != it1->second.end(); it2++) {
      DEBUG_LOG(m_logger, "{},{},{},{},{}", it1->first, it2->addr, it2->tag, it2->dirty, it2->ready);
    }
  }
}

}  // namespace Ramulator
