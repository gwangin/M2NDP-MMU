#pragma once
#ifdef TIMING_SIMULATION

#include <cstdint>
#include <unordered_map>
#include <queue>
#include "delay_queue.h"   // DelayQueue<T>
#include "mem_fetch.h"     // NDPSim::mem_fetch
#include "m2ndp_config.h"  // NDPSim::M2NDPConfig

namespace NDPSim {

class MemoryMap;
class Tlb;  // forward

// x86-64 4KB, 4-레벨(48-bit canonical) 테이블워크.
// Page table 메모리는 64B 라인 단위(uint8)로 MemoryMap에 존재한다고 가정.
class MMU {
public:
  // pt_base: PML4 물리 베이스 (예: 0x0009_0000_0000_0000)
  // cfg, owner_tlb, ndp_id는 Ramulator 채널 계산과 to-mem push에 필요
  MMU(MemoryMap* mem, uint64_t pt_base,
      M2NDPConfig* cfg = nullptr, Tlb* owner_tlb = nullptr,
      int ndp_id = 0, uint64_t page_size = 4096);

  // 기존 동기 API(남겨둠, 필요시 사용)
  bool Translate(uint64_t va, uint64_t& pa_out, bool is_write);

  // 비동기 PT walk 제출 (TLB miss에서 호출)
  void submit(mem_fetch* orig_mf);

  // Ramulator 경로로 보낸 PTW 메모리 요청이 돌아왔는지 확인하기 위해 TLB가 호출
  bool waiting_for_fill(mem_fetch* mf) const;

  // NdpUnit::from_mem_handle() -> Tlb::fill(mf) -> MMU 로 반납 (데이터는 MemoryMap에서 읽음)
  void on_mem_fill(mem_fetch* mf);

  // TLB가 주기적으로 호출 (발행 queue, 내부 상태 갱신)
  void cycle();

  // 완료된 VA→PA 변환(원본 mf)을 가져감 (TLB가 polling)
  struct Completed {
    mem_fetch* mf;
    uint64_t   va;
    uint64_t   pa;
  };
  bool has_completed() const { return !m_done.empty(); }
  Completed pop_completed();

  // Trace/통계
  struct Stats {
    uint64_t walks=0, walk_reads=0, hits=0, fails=0;
  };
  Stats GetStats() const { return m_stats; }

  // 바인딩/설정자
  void bind_tlb(Tlb* tlb)        { m_owner_tlb = tlb; }
  void bind_config(M2NDPConfig* cfg) { m_cfg = cfg; }

  // (선택) 소규모 issue 지연을 줄 값. 0이면 즉시 발행
  void set_ptw_issue_latency(int cyc) { m_ptw_issue_latency = cyc; }

  // (선택) 최대 동시 워크 제한 (백프레셔/메모리 폭주 방지)
  void set_max_outstanding_walks(int n) { m_max_outstanding_walks = n; }

private:
  struct WalkCtx {
    mem_fetch* orig;      // 원본 요청
    uint64_t   va;
    uint64_t   pa_out = 0;
    int        level = 4; // 4→3→2→1
    uint64_t   next_addr; // 다음 읽을 PTE의 물리주소 (64B 라인 기준 정렬)
    bool       is_write = false;
  };

  // 64B 라인 정렬 후 8B little-endian 로드 (기능적으로 메모리 내용 조회)
  uint64_t read_qword(uint64_t phys_addr);

  // 다음 단계의 PTE 라인을 Ramulator 경로로 읽도록 발행
  void issue_pt_read(WalkCtx* wctx, uint64_t pte_line_addr);

  // VA에서 인덱스/오프셋 계산
  inline uint64_t vpn(uint64_t va) const { return va >> m_page_shift; }
  inline uint64_t page_off(uint64_t va) const { return va & (m_page_size - 1); }
  inline uint64_t idx_pml4(uint64_t va) const { return (va >> 39) & 0x1FF; }
  inline uint64_t idx_pdpt(uint64_t va) const { return (va >> 30) & 0x1FF; }
  inline uint64_t idx_pd (uint64_t va) const { return (va >> 21) & 0x1FF; }
  inline uint64_t idx_pt (uint64_t va) const { return (va >> 12) & 0x1FF; }

private:
  MemoryMap*     m_mem;
  M2NDPConfig*   m_cfg;
  Tlb*           m_owner_tlb;
  int            m_ndp_id;
  uint64_t       m_pt_base;
  uint64_t       m_page_size;
  uint32_t       m_page_shift;

  // 발행 스로틀링용 (지정한 latency 이후 to-mem push 시도)
  DelayQueue<mem_fetch*> m_issue_q;

  // inflight: ptw용 mem_fetch -> WalkCtx*
  std::unordered_map<mem_fetch*, WalkCtx*> m_inflight;

  // 완료된 원본 (VA/PA 함께 보관)
  std::queue<Completed> m_done;

  // 통계
  Stats m_stats;

  // 발행 간격 (선택): 0이면 바로 push, 그 외면 m_issue_q 사용
  int m_ptw_issue_latency = 0; // 기본 0

  // 최대 동시 워크 수 제한 (기본 0 = 무제한)
  int m_max_outstanding_walks = 0;
};

} // namespace NDPSim
#endif // TIMING_SIMULATION
