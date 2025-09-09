#ifdef TIMING_SIMULATION
#include "tlb.h"
#include "mmu.h"
#include "mem_fetch.h"
#include "spdlog/spdlog.h"

static inline bool is_canonical48(uint64_t va) {
  return (uint64_t)((int64_t)(va << 16) >> 16) == va;
}

namespace {

// 48-bit canonical 주소로 정규화
static inline uint64_t canonicalize48(uint64_t va) {
  const uint64_t MASK48 = (1ULL << 48) - 1; // 0x0000FFFFFFFFFFFF
  uint64_t a = va & MASK48;
  if (a & (1ULL << 47)) a |= ~MASK48;       // bit47 sign-extend
  return a;
}

// 초기 디버깅용: 비정규 주소 감지
static inline void warn_if_noncano(const char* tag, uint64_t va) {
  if ((va & (1ULL<<47)) && ((va >> 48) != 0xFFFFULL)) {
    spdlog::warn("[{}] non-canonical VA: {:#018x}", tag, va);
  }
}

} // anonymous namespace

namespace NDPSim {

Tlb::Tlb(int id, M2NDPConfig* config, std::string tlb_config,
         fifo_pipeline<mem_fetch>* to_mem_queue)
    : m_id(id), m_config(config), m_to_mem_queue(to_mem_queue) {
  m_page_size = config->get_tlb_page_size();

  // NEW: page_shift 계산
  m_page_shift = 0;
  uint64_t ps = static_cast<uint64_t>(m_page_size);
  while ((1ULL << m_page_shift) < ps) ++m_page_shift;

  m_tlb_config.init(tlb_config, config);
  m_tlb = new ReadOnlyCache("tlb", m_tlb_config, id, 0, m_to_mem_queue);
  m_tlb_entry_size = m_config->get_tlb_entry_size();
  m_finished_mf = fifo_pipeline<mem_fetch>("tlb_finished_mf", 0,
                                           m_config->get_request_queue_size());
  m_tlb_request_queue = DelayQueue<mem_fetch*>(
      "tlb_req_queue", true, m_config->get_request_queue_size());
  m_dram_tlb_latency_queue = DelayQueue<mem_fetch*>(
    "dram_tlb_latency_queue", true, m_config->get_request_queue_size());
  m_tlb_hit_latency = m_config->get_tlb_hit_latency();
  m_accessed_tlb_addr = m_config->get_accessed_tlb_addr();
}

void Tlb::set_ideal_tlb() {
  m_ideal_tlb = true;
  m_tlb_hit_latency = 0;
}
bool Tlb::fill_port_free() { return m_tlb->fill_port_free(); }
bool Tlb::data_port_free() { return m_tlb->data_port_free(); }

bool Tlb::full() { return full(0); }

bool Tlb::full(uint64_t mf_size) {
  return m_tlb_request_queue.size() + m_dram_tlb_latency_queue.size() + mf_size >=
         m_config->get_request_queue_size();
}

void Tlb::fill(mem_fetch* mf) {
  mf->current_state = "TLB Fill";
  if (!m_config->is_dram_tlb_miss_handling_enabled()) {
    assert(mf->get_addr() >= DRAM_TLB_BASE);
    m_tlb->fill(mf, m_config->get_ndp_cycle());
    return;
  } else {
    uint64_t tlb_addr = mf->get_addr();
    if (m_accessed_tlb_addr->find(tlb_addr) == m_accessed_tlb_addr->end()) {
      if(m_config->get_use_dram_tlb()) 
        m_accessed_tlb_addr->insert(tlb_addr);
      m_dram_tlb_latency_queue.push(
          mf, m_config->get_dram_tlb_miss_handling_latency());
      return;
    } else {
      m_tlb->fill(mf, m_config->get_ndp_cycle());
      return;
    }
  }
}

bool Tlb::waiting_for_fill(mem_fetch* mf) {
  return m_tlb->waiting_for_fill(mf);
}

void Tlb::access(mem_fetch* mf) {
  assert(!full());
  m_tlb_request_queue.push(mf, m_tlb_hit_latency);
}

bool Tlb::data_ready() { return !m_finished_mf.empty(); }

mem_fetch* Tlb::get_data() { return m_finished_mf.top(); }

void Tlb::pop_data() { m_finished_mf.pop(); }

void Tlb::cycle() {
  // Tlb cache cycle & decrease latency cycle
  m_tlb->cycle();
  m_tlb_request_queue.cycle();
  m_dram_tlb_latency_queue.cycle();
}

void Tlb::bank_access_cycle() {
  if (!m_tlb_request_queue.empty() && data_port_free()) {
    mem_fetch* mf = m_tlb_request_queue.top();

    uint64_t va = mf->get_addr();
    uint64_t pa = va;
    bool translated = true;

    // write 여부 판단
    bool is_wr = mf->is_write() ||
                 mf->get_type() == WRITE_REQUEST ||
                 mf->get_access_type() == GLOBAL_ACC_W;

    if (m_mmu) {
      translated = m_mmu->Translate(va, pa, /*is_write=*/is_wr);
    }

    if (translated) {
      mf->set_addr(pa);
      mf->set_channel(m_config->get_channel_index(pa));
    }


    m_finished_mf.push(mf);
    m_tlb_request_queue.pop();
  }
}


CacheStats Tlb::get_stats() { return m_tlb->get_stats(); }
static inline uint64_t canonicalize48(uint64_t va) {
  const uint64_t MASK48 = (1ULL << 48) - 1;
  uint64_t a = va & MASK48;
  if (a & (1ULL << 47)) a |= ~MASK48;
  return a;
}

uint64_t Tlb::get_tlb_addr(uint64_t addr) {
  // 과한 경고 억제 (필요하면 유지)
  static unsigned warn_count = 0;
  if (!is_canonical48(addr) && warn_count < 8) {
    spdlog::warn("[DTLB] non-canonical VA: {:#018x}", addr);
    ++warn_count;
  }

  // ★ 반드시 canonical VA로 VPN 계산
  const uint64_t cv  = canonicalize48(addr);
  const uint64_t vpn = (cv >> m_page_shift);
  return vpn * (uint64_t)m_tlb_entry_size + DRAM_TLB_BASE;
}




}  // namespace NDPSim
#endif
