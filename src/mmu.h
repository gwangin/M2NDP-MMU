#pragma once
#include <cstdint>
#include <sys/types.h>

namespace NDPSim {

class MemoryMap;  // pointer로만 쓰므로 전방 선언

// x86-64 4KB, 4-레벨(48-bit canonical) 테이블워크
// PT 메모리는 64B 라인단위(uint8)로 MemoryMap에 올라와 있다고 가정.
class MMU {
public:
  // pt_base: PML4 물리 베이스 (예: 0x0009_0000_0000_0000)
  MMU(MemoryMap* mem, uint64_t pt_base, uint64_t page_size = 4096);

  // va -> pa 변환 (성공 시 true). 아이덴티티 매핑 유무와 무관하게 존재 확인용으로 사용 가능.
  bool Translate(uint64_t va, uint64_t& pa_out, bool is_write);

  // 트레이싱 제어 (기본 비활성). limit=0이면 무제한
  void EnableTracing(bool en, uint32_t limit = 256) {
    m_trace_enabled = en; m_trace_limit = limit;
  }
  // 통계 노출(원하면 사용)
  struct Stats { uint64_t walks=0, walk_reads=0, hits=0, fails=0; };
  Stats GetStats() const { return m_stats; }

private:
  // MemoryMap이 64B 라인 단위로만 Load 가능하므로, 라인 정렬 후 8B little-endian 로드
  uint64_t read_qword(uint64_t phys_addr);

private:
  MemoryMap* m_mem;
  uint64_t   m_pt_base;
  uint64_t   m_page_size;
  uint32_t   m_page_shift;

  // tracing
  bool       m_trace_enabled = false;
  uint32_t   m_trace_limit   = 256;
  uint32_t   m_trace_count   = 0;
  Stats      m_stats;
};

} // namespace NDPSim
