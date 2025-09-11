#ifdef TIMING_SIMULATION
#ifndef TLB_H
#define TLB_H
#include "cache.h"
#include "common.h"
#include "delay_queue.h"
#include "m2ndp_config.h"

namespace NDPSim { class MMU; }

namespace NDPSim {
class Tlb {
 public:
  Tlb(int id, M2NDPConfig *config, std::string tlb_config, fifo_pipeline<mem_fetch> *to_mem_queue);
  void set_ideal_tlb();
  bool fill_port_free();
  bool data_port_free();

  // MISS 시 SW page walk(네 MMU) 사용하도록 주입
  void set_mmu(MMU* mmu) { m_mmu = mmu; }

  bool full();
  bool full(uint64_t mf_size);

  // 메모리에서 온(혹은 내부에서 만든) TLB 라인을 실제 TLB 캐시에 채움
  void fill(mem_fetch *mf);

  bool waiting_for_fill(mem_fetch *mf);

  // 상위에서 들어온 TLB 조회 요청(원본 mf)
  void access(mem_fetch* mf);

  // 주소변환 완료된 원본 mf
  bool data_ready();
  mem_fetch* get_data();
  void pop_data();

  // 주기 동작
  void cycle();
  void bank_access_cycle();

  CacheStats get_stats();

 private:
  uint64_t get_tlb_addr(uint64_t addr);

 private:
  MMU* m_mmu = nullptr;       // MISS 시 소프트웨어 페이지워크에 사용
  int  m_id;
  int  m_page_size;
  int  m_tlb_entry_size;
  int  m_tlb_hit_latency;
  bool m_ideal_tlb = false;

  M2NDPConfig *m_config;
  fifo_pipeline<mem_fetch> *m_to_mem_queue;

  fifo_pipeline<mem_fetch> m_finished_mf;       // 완료된 원본 mf
  DelayQueue<mem_fetch*>   m_tlb_request_queue; // 원본 mf 조회 대기
  DelayQueue<mem_fetch*>   m_dram_tlb_latency_queue; // (원형 유지; 우린 쓰지 않아도 무방)
  std::set<uint64_t>      *m_accessed_tlb_addr; // (원형 유지)

  CacheConfig m_tlb_config;
  Cache      *m_tlb;
};
}  // namespace NDPSim
#endif
#endif
