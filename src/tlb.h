#ifdef TIMING_SIMULATION
#ifndef TLB_H
#define TLB_H

#include <unordered_map>
#include <list>
#include "cache.h"
#include "common.h"
#include "delay_queue.h"
#include "m2ndp_config.h"

namespace NDPSim {

class MMU;
class mem_fetch;
template <typename T> class fifo_pipeline;

class Tlb {
public:
  Tlb(int id, M2NDPConfig *config, std::string tlb_config,
      fifo_pipeline<mem_fetch> *to_mem_queue);

  void set_ideal_tlb();

  // MMU 주입 & 바인딩
  void set_mmu(MMU* mmu);

  bool fill_port_free();
  bool data_port_free();
  bool full();
  bool full(uint64_t mf_size);

  // NdpUnit::from_mem_handle()에서 호출
  void fill(mem_fetch *mf);

  bool waiting_for_fill(mem_fetch *mf);

  // 상위 요청
  void access(mem_fetch* mf);

  // 완료 데이터
  bool data_ready();
  mem_fetch* get_data();
  void pop_data();

  // 주기 동작
  void cycle();
  void bank_access_cycle();

  // Ramulator to-mem push (MMU가 사용)
  bool push_mem_req(mem_fetch* mf);

  CacheStats get_stats();

private:
  uint64_t get_tlb_addr(uint64_t addr);

  inline uint64_t vpn(uint64_t va) const { return va >> m_page_shift; }
  inline uint64_t ppn(uint64_t pa) const { return pa >> m_page_shift; }
  inline uint64_t page_off(uint64_t a) const { return a & (m_page_size - 1); }

  // 간단한 SW TLB (LRU)
  void sw_tlb_install(uint64_t vpn, uint64_t ppn);
  bool sw_tlb_lookup(uint64_t vpn, uint64_t& ppn_out);

private:
  MMU* m_mmu = nullptr;

  int  m_id;
  int  m_page_size;
  int  m_tlb_entry_size;
  int  m_tlb_hit_latency;
  uint32_t m_page_shift;
  bool m_ideal_tlb = false;

  M2NDPConfig *m_config;

  // NdpUnit의 m_tlb_req
  fifo_pipeline<mem_fetch> *m_to_mem_queue;

  // 완료 원본 mf
  fifo_pipeline<mem_fetch> m_finished_mf;

  // TLB hit latency 모델
  DelayQueue<mem_fetch*> m_tlb_request_queue;

  // (미사용이지만 인터페이스 유지)
  DelayQueue<mem_fetch*> m_dram_tlb_latency_queue;

  // (원형 유지)
  std::set<uint64_t> *m_accessed_tlb_addr;

  // 원래의 cache형 TLB (현 구조에선 MMU path에서 직접 사용 X, 유지)
  CacheConfig m_tlb_config;
  Cache *m_tlb;

  // SW TLB: vpn -> (ppn, LRU iterator)
  size_t m_sw_cap = 1024; // 필요 시 config에서 유도 가능
  std::list<uint64_t> m_sw_lru; // MRU front
  std::unordered_map<uint64_t,
    std::pair<uint64_t, std::list<uint64_t>::iterator>> m_sw_map;
};

} // namespace NDPSim
#endif
#endif // TIMING_SIMULATION
