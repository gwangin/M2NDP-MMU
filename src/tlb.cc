#ifdef TIMING_SIMULATION
#include "tlb.h"
#include "mmu.h"
#include "mem_fetch.h"

namespace NDPSim {

Tlb::Tlb(int id, M2NDPConfig* config, std::string tlb_config,
         fifo_pipeline<mem_fetch>* to_mem_queue)
    : m_id(id), m_config(config), m_to_mem_queue(to_mem_queue) {
  m_page_size = config->get_tlb_page_size();
  m_tlb_config.init(tlb_config, config);
  m_tlb = new ReadOnlyCache("tlb", m_tlb_config, id, 0, m_to_mem_queue);
  m_tlb_entry_size = m_config->get_tlb_entry_size();
  m_finished_mf = fifo_pipeline<mem_fetch>("tlb_finished_mf", 0,
                                           m_config->get_request_queue_size());
  m_tlb_request_queue = DelayQueue<mem_fetch*>(
      "tlb_req_queue", true, m_config->get_request_queue_size());
  m_dram_tlb_latency_queue = DelayQueue<mem_fetch*> (
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

// ★ 핵심 변경점: DRAM-TLB/ATS 경로를 우회하고, TLB miss 채우는 시점에
//   원본 MF의 VA→PA 변환(네 MMU)을 수행한 뒤, 곧바로 TLB를 채움.
void Tlb::fill(mem_fetch* mf) {
  mf->current_state = "TLB Fill";

  // 원본 요청 mf (tlb_mf 생성 시에 set_tlb_original_mf로 연결됨)
  mem_fetch* orig = mf->get_tlb_original_mf();
  if (orig && m_mmu) {
    uint64_t va = orig->get_addr();
    bool is_wr = orig->is_write() ||
                 orig->get_type() == WRITE_REQUEST ||
                 orig->get_access_type() == GLOBAL_ACC_W;
    uint64_t pa = va;
    bool ok = m_mmu->Translate(va, pa, is_wr);
    // 변환 성공 시 물리주소 및 채널 갱신(OLAP 환경에선 va==pa라도 안전)
    if (ok) {
      orig->set_addr(pa);
      orig->set_channel(m_config->get_channel_index(pa));
    } else {
      // 보수적으로 VA 유지
      orig->set_channel(m_config->get_channel_index(va));
    }
  }

  // DRAM-TLB 사용 여부와 무관하게, 바로 TLB 라인을 채운다(ATS latency 우회)
  m_tlb->fill(mf, m_config->get_ndp_cycle());
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
  // (원형 유지) DRAM-TLB latency 큐에서 꺼내 캐시 채우기
  if(!m_dram_tlb_latency_queue.empty()) {
    mem_fetch* mf = m_dram_tlb_latency_queue.top();
    m_tlb->fill(mf, m_config->get_ndp_cycle());
    m_dram_tlb_latency_queue.pop();
  }
  // (원형 유지) 캐시가 준비되면 tlb_mf를 회수하고 원본 mf를 완료 큐로 밀어줌
  if (m_tlb->access_ready() && !m_finished_mf.full()) {
    mem_fetch* mf = m_tlb->pop_next_access();
    if (mf->is_request()) mf->set_reply();
    m_finished_mf.push(mf->get_tlb_original_mf());
    delete mf;
  }
  // (원형 유지) TLB 조회 파이프라인
  if (!m_tlb_request_queue.empty() && data_port_free()) {
    mem_fetch* mf = m_tlb_request_queue.top();
    uint64_t addr = mf->get_addr();
    uint64_t tlb_addr = get_tlb_addr(addr);
    mem_fetch* tlb_mf =
        new mem_fetch(tlb_addr, TLB_ACC_R, READ_REQUEST, m_tlb_entry_size,
                      CXL_OVERHEAD, m_config->get_ndp_cycle());
    tlb_mf->set_from_ndp(true);
    tlb_mf->set_ndp_id(m_id);
    tlb_mf->set_tlb_original_mf(mf);
    tlb_mf->set_channel(m_config->get_channel_index(tlb_addr));
    std::deque<CacheEvent> events;
    CacheRequestStatus stat = MISS;
    if (!m_ideal_tlb)
      stat = m_tlb->access(tlb_addr, m_config->get_ndp_cycle(), tlb_mf, events);
    if ((stat == HIT || m_ideal_tlb) && !m_finished_mf.full()) { // alway hit if ideal tlb
      m_finished_mf.push(mf);
      delete tlb_mf;
      m_tlb_request_queue.pop();
    } else if (stat == HIT && m_finished_mf.full()) {
      delete tlb_mf;
    } else if (stat != RESERVATION_FAIL) {
      // MISS/HIT_RESERVED → 원형 그대로 pop (실제 채움은 fill()에서 즉시 수행됨)
      m_tlb_request_queue.pop();
      // tlb_mf는 캐시가 잡고 있다가 access_ready()에 회수됨(원형과 동일)
    } else if (stat == RESERVATION_FAIL) {
      delete tlb_mf;
    }
  }
}

CacheStats Tlb::get_stats() { return m_tlb->get_stats(); }

uint64_t Tlb::get_tlb_addr(uint64_t addr) {
  return addr / m_page_size * m_tlb_entry_size + DRAM_TLB_BASE;
}
}  // namespace NDPSim
#endif
