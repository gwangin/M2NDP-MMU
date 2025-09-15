#ifdef TIMING_SIMULATION
#include "tlb.h"
#include "mmu.h"
#include "mem_fetch.h"

namespace NDPSim {

Tlb::Tlb(int id, M2NDPConfig* config, std::string tlb_config,
         fifo_pipeline<mem_fetch>* to_mem_queue)
  : m_id(id)
  , m_config(config)
  , m_to_mem_queue(to_mem_queue)
  , m_finished_mf("tlb_finished_mf", 0, m_config->get_request_queue_size())
  , m_tlb_request_queue("tlb_req_queue", true, m_config->get_request_queue_size())
  , m_dram_tlb_latency_queue("dram_tlb_latency_queue", true, m_config->get_request_queue_size())
{
  m_page_size = m_config->get_tlb_page_size();
  // page_shift
  uint32_t sh = 0;
  while ((1ULL<<sh) < (uint64_t)m_page_size) sh++;
  m_page_shift = sh;

  m_tlb_config.init(tlb_config, m_config);
  m_tlb = new ReadOnlyCache("tlb", m_tlb_config, id, 0, m_to_mem_queue);

  m_tlb_entry_size   = m_config->get_tlb_entry_size();
  m_tlb_hit_latency  = m_config->get_tlb_hit_latency();
  m_accessed_tlb_addr = m_config->get_accessed_tlb_addr();

  // SW TLB 기본 용량은 1024로 시작 (원한다면 config에서 파생)
  m_sw_cap = 1024;
}

void Tlb::set_mmu(MMU* mmu) {
  m_mmu = mmu;
  if (m_mmu) {
    m_mmu->bind_tlb(this);
    m_mmu->bind_config(m_config);
  }
}

void Tlb::set_ideal_tlb() {
  m_ideal_tlb = true;
  m_tlb_hit_latency = 0;
}

bool Tlb::fill_port_free() {
  // MMU 경로는 내부에서 처리하므로 fill 포트 제약을 걸지 않되,
  // MMU가 없을 땐 기존 TLB cache 포트를 존중
  return m_mmu ? true : m_tlb->fill_port_free();
}

bool Tlb::data_port_free() { return m_tlb->data_port_free(); }

bool Tlb::full() { return full(0); }

bool Tlb::full(uint64_t mf_sz) {
  return (m_tlb_request_queue.size() + m_dram_tlb_latency_queue.size() + mf_sz)
          >= m_config->get_request_queue_size();
}

bool Tlb::waiting_for_fill(mem_fetch* mf) {
  // m_tlb(캐시)가 대기중이거나, MMU가 낸 PTW 요청이 대기중이면 true
  return m_tlb->waiting_for_fill(mf) || (m_mmu && m_mmu->waiting_for_fill(mf));
}

void Tlb::fill(mem_fetch* mf) {
  // MMU가 낸 PTW 요청이면 MMU로 반납
  if (m_mmu && m_mmu->waiting_for_fill(mf)) {
    m_mmu->on_mem_fill(mf);
    return;
  }
  // 그 외(존재한다면) TLB 캐시 라인 fill
  mf->current_state = "TLB Fill";
  m_tlb->fill(mf, m_config->get_ndp_cycle());
}

void Tlb::access(mem_fetch* mf) {
  // TLB hit latency를 모델링: 실제 hit/miss 판단은 bank_access_cycle에서 처리
  m_tlb_request_queue.push(mf, m_tlb_hit_latency);
}

bool Tlb::data_ready() { return !m_finished_mf.empty(); }
mem_fetch* Tlb::get_data() { return m_finished_mf.top(); }
void Tlb::pop_data() { m_finished_mf.pop(); }

void Tlb::cycle() {
  // 캐시, 큐 tick
  m_tlb->cycle();
  m_tlb_request_queue.cycle();
  m_dram_tlb_latency_queue.cycle();

  // MMU tick
  if (m_mmu) m_mmu->cycle();
}

bool Tlb::push_mem_req(mem_fetch* mf) {
  if (!m_to_mem_queue->full()) {
    m_to_mem_queue->push(mf);
    return true;
  }
  return false;
}

void Tlb::bank_access_cycle() {
  // (유지) 과거 DRAM-TLB 지연 큐는 사용하지 않지만, 남겨도 무해
  if(!m_dram_tlb_latency_queue.empty()) {
    mem_fetch* mf = m_dram_tlb_latency_queue.top();
    m_tlb->fill(mf, m_config->get_ndp_cycle());
    m_dram_tlb_latency_queue.pop();
  }

  // 1) MMU 완료분 먼저 회수 → SW TLB에 기록 후 완료 큐로 전달
  if (m_mmu) {
    while (m_mmu->has_completed() && !m_finished_mf.full()) {
      MMU::Completed c = m_mmu->pop_completed();
      // SW TLB에 (vpn -> ppn) 설치
      sw_tlb_install(vpn(c.va), ppn(c.pa));
      // 상위로 리턴
      m_finished_mf.push(c.mf);
    }
  }

  // 2) TLB 요청 처리 (hit latency 경과 후)
  if (!m_tlb_request_queue.empty() && data_port_free()) {
    mem_fetch* mf = m_tlb_request_queue.top();
    uint64_t va  = mf->get_addr();
    uint64_t off = page_off(va);
    uint64_t v   = vpn(va);

    // SW TLB 조회
    uint64_t pp;
    if (sw_tlb_lookup(v, pp)) {
      // hit → PA로 변환, 완료로 전달
      uint64_t pa = (pp << m_page_shift) | off;
      mf->set_addr(pa);
      mf->set_channel(m_config->get_channel_index(pa));
      if (!m_finished_mf.full()) {
        m_finished_mf.push(mf);
        m_tlb_request_queue.pop();
      }
      return;
    }

    // miss → MMU로 비동기 제출
    if (m_mmu) {
      m_mmu->submit(mf);
      m_tlb_request_queue.pop();
      return;
    }

    // MMU가 없으면 기존 캐시 경로로 (원형)
    uint64_t tlb_addr = get_tlb_addr(va);
    mem_fetch* tlb_mf = new mem_fetch(
        tlb_addr, TLB_ACC_R, READ_REQUEST, m_tlb_entry_size,
        CXL_OVERHEAD, m_config->get_ndp_cycle());
    tlb_mf->set_from_ndp(true);
    tlb_mf->set_ndp_id(m_id);
    tlb_mf->set_tlb_original_mf(mf);
    tlb_mf->set_channel(m_config->get_channel_index(tlb_addr));

    std::deque<CacheEvent> events;
    CacheRequestStatus stat = m_ideal_tlb ? HIT
                                          : m_tlb->access(tlb_addr, m_config->get_ndp_cycle(), tlb_mf, events);
    if ((stat == HIT || m_ideal_tlb) && !m_finished_mf.full()) {
      m_finished_mf.push(mf);
      delete tlb_mf;
      m_tlb_request_queue.pop();
    } else if (stat == HIT && m_finished_mf.full()) {
      delete tlb_mf;
    } else if (stat != RESERVATION_FAIL) {
      m_tlb_request_queue.pop();
    } else if (stat == RESERVATION_FAIL) {
      delete tlb_mf;
    }
  }
}

CacheStats Tlb::get_stats() { return m_tlb->get_stats(); }

uint64_t Tlb::get_tlb_addr(uint64_t addr) {
  return addr / m_page_size * m_tlb_entry_size + DRAM_TLB_BASE;
}

/************** SW TLB (LRU) **************/

void Tlb::sw_tlb_install(uint64_t v, uint64_t p) {
  auto it = m_sw_map.find(v);
  if (it != m_sw_map.end()) {
    m_sw_lru.erase(it->second.second);
    m_sw_map.erase(it);
  }
  m_sw_lru.push_front(v);
  m_sw_map[v] = {p, m_sw_lru.begin()};
  if (m_sw_map.size() > m_sw_cap) {
    auto victim = m_sw_lru.back();
    m_sw_lru.pop_back();
    m_sw_map.erase(victim);
  }
}

bool Tlb::sw_tlb_lookup(uint64_t v, uint64_t& p_out) {
  auto it = m_sw_map.find(v);
  if (it == m_sw_map.end()) return false;
  // LRU update
  m_sw_lru.erase(it->second.second);
  m_sw_lru.push_front(v);
  it->second.second = m_sw_lru.begin();
  p_out = it->second.first;
  return true;
}

} // namespace NDPSim
#endif // TIMING_SIMULATION
