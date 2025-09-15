#ifdef TIMING_SIMULATION
#include "mmu.h"
#include "memory_map.h"
#include "vector_data.h"
#include "tlb.h"
#include <stdexcept>

namespace NDPSim {

MMU::MMU(MemoryMap* mem, uint64_t pt_base, M2NDPConfig* cfg, Tlb* owner_tlb,
         int ndp_id, uint64_t page_size)
  : m_mem(mem)
  , m_cfg(cfg)
  , m_owner_tlb(owner_tlb)
  , m_ndp_id(ndp_id)
  , m_pt_base(pt_base)
  , m_page_size(page_size)
  , m_issue_q("mmu_issue_q", /*only_latency=*/true, /*max_size=*/-1)
{
  uint64_t ps = page_size;
  uint32_t sh = 0;
  while ((1ULL << sh) < ps) sh++;
  m_page_shift = sh;
}

bool MMU::Translate(uint64_t va, uint64_t& pa_out, bool /*is_write*/) {
  const uint64_t pml4_i = idx_pml4(va);
  const uint64_t pdpt_i = idx_pdpt(va);
  const uint64_t pd_i   = idx_pd(va);
  const uint64_t pt_i   = idx_pt(va);
  const uint64_t off    = page_off(va);

  uint64_t pml4e = read_qword(m_pt_base + pml4_i * 8);
  if (!(pml4e & 0x1)) { m_stats.fails++; return false; }
  uint64_t pdpt_base = pml4e & ~0xFFFULL;

  uint64_t pdpte = read_qword(pdpt_base + pdpt_i * 8);
  if (!(pdpte & 0x1)) { m_stats.fails++; return false; }
  uint64_t pd_base = pdpte & ~0xFFFULL;

  uint64_t pde = read_qword(pd_base + pd_i * 8);
  if (!(pde & 0x1)) { m_stats.fails++; return false; }
  uint64_t pt_base = pde & ~0xFFFULL;

  uint64_t pte = read_qword(pt_base + pt_i * 8);
  if (!(pte & 0x1)) { m_stats.fails++; return false; }
  uint64_t frame = pte & ~0xFFFULL;

  pa_out = frame | off;
  m_stats.hits++;
  return true;
}

void MMU::submit(mem_fetch* orig_mf) {
  // outstanding 제한이 있으면 리턴 (상위에서 다음 사이클 재시도)
  if (m_max_outstanding_walks > 0 &&
      (int)m_inflight.size() >= m_max_outstanding_walks) {
    return;
  }

  // PT walk 컨텍스트 생성
  auto* w = new WalkCtx();
  w->orig     = orig_mf;
  w->va       = orig_mf->get_addr();
  w->is_write = orig_mf->is_write() ||
                orig_mf->get_type() == WRITE_REQUEST ||
                orig_mf->get_access_type() == GLOBAL_ACC_W;

  // 1단계: PML4 엔트리 라인 주소
  uint64_t pml4_line = (m_pt_base + idx_pml4(w->va) * 8) & ~0x3FULL;
  w->level     = 4;
  w->next_addr = pml4_line;

  m_stats.walks++;
  // 첫 PTE line read 발행
  issue_pt_read(w, w->next_addr);
}

bool MMU::waiting_for_fill(mem_fetch* mf) const {
  return m_inflight.find(mf) != m_inflight.end();
}

void MMU::on_mem_fill(mem_fetch* mf) {
  // Ramulator 경로에서 돌아온 PT 라인. 실제 데이터는 MemoryMap에서 로드.
  auto it = m_inflight.find(mf);
  if (it == m_inflight.end()) {
    // 우리가 낸 것이 아니면 무시
    delete mf;
    return;
  }
  WalkCtx* w = it->second;
  m_inflight.erase(it);
  // PT 라인 fetch용 mf는 여기서 소멸
  delete mf;

  // 현재 단계의 엔트리 읽고 다음 단계 진행
  if (w->level == 4) {
    uint64_t pml4e = read_qword(m_pt_base + idx_pml4(w->va) * 8);
    if (!(pml4e & 0x1)) { m_stats.fails++; delete w; return; }
    uint64_t pdpt_base = pml4e & ~0xFFFULL;

    uint64_t pdpt_line = (pdpt_base + idx_pdpt(w->va) * 8) & ~0x3FULL;
    w->level     = 3;
    w->next_addr = pdpt_line;
    issue_pt_read(w, w->next_addr);
    return;
  }

  if (w->level == 3) {
    uint64_t pml4e     = read_qword(m_pt_base + idx_pml4(w->va) * 8);
    uint64_t pdpt_base = pml4e & ~0xFFFULL;
    uint64_t pdpte     = read_qword(pdpt_base + idx_pdpt(w->va) * 8);
    if (!(pdpte & 0x1)) { m_stats.fails++; delete w; return; }
    uint64_t pd_base = pdpte & ~0xFFFULL;

    uint64_t pd_line = (pd_base + idx_pd(w->va) * 8) & ~0x3FULL;
    w->level     = 2;
    w->next_addr = pd_line;
    issue_pt_read(w, w->next_addr);
    return;
  }

  if (w->level == 2) {
    uint64_t pml4e     = read_qword(m_pt_base + idx_pml4(w->va) * 8);
    uint64_t pdpt_base = pml4e & ~0xFFFULL;
    uint64_t pdpte     = read_qword(pdpt_base + idx_pdpt(w->va) * 8);
    uint64_t pd_base   = pdpte & ~0xFFFULL;
    uint64_t pde       = read_qword(pd_base + idx_pd(w->va) * 8);
    if (!(pde & 0x1)) { m_stats.fails++; delete w; return; }
    uint64_t pt_base = pde & ~0xFFFULL;

    uint64_t pt_line = (pt_base + idx_pt(w->va) * 8) & ~0x3FULL;
    w->level     = 1;
    w->next_addr = pt_line;
    issue_pt_read(w, w->next_addr);
    return;
  }

  // level == 1 (PTE)
  if (w->level == 1) {
    uint64_t pml4e     = read_qword(m_pt_base + idx_pml4(w->va) * 8);
    uint64_t pdpt_base = pml4e & ~0xFFFULL;
    uint64_t pdpte     = read_qword(pdpt_base + idx_pdpt(w->va) * 8);
    uint64_t pd_base   = pdpte & ~0xFFFULL;
    uint64_t pde       = read_qword(pd_base + idx_pd(w->va) * 8);
    uint64_t pt_base   = pde & ~0xFFFULL;
    uint64_t pte       = read_qword(pt_base + idx_pt(w->va) * 8);

    if (!(pte & 0x1)) { m_stats.fails++; delete w; return; }

    uint64_t frame = pte & ~0xFFFULL;
    uint64_t pa    = frame | page_off(w->va);

    w->pa_out = pa;

    // 원본 mf를 PA로 교체
    w->orig->set_addr(pa);
    if (m_cfg) w->orig->set_channel(m_cfg->get_channel_index(pa));

    // TLB가 SW 캐시에 설치할 수 있도록 VA/PA 함께 넘김
    m_done.push(Completed{w->orig, w->va, pa});
    delete w;
    return;
  }

  // 여기 오면 안 됨
  delete w;
}

void MMU::cycle() {
  // PTW 발행 딜레이 큐 tick
  m_issue_q.cycle();

  // 발행 준비된 요청이 있으면 TLB의 to-mem queue로 푸시
  while (!m_issue_q.empty()) {
    mem_fetch* mf = m_issue_q.top();
    if (m_owner_tlb && m_owner_tlb->push_mem_req(mf)) {
      m_issue_q.pop();
    } else {
      // to-mem이 풀이라면 잠시 대기 (다음 cycle에 재시도)
      break;
    }
  }
}

MMU::Completed MMU::pop_completed() {
  Completed c = m_done.front();
  m_done.pop();
  return c;
}

uint64_t MMU::read_qword(uint64_t phys_addr) {
  const uint64_t LINE = 64;
  uint64_t base = phys_addr & ~(LINE - 1);
  uint32_t off  = static_cast<uint32_t>(phys_addr - base);
  if (off > (LINE - 8)) {
    throw std::runtime_error("MMU: PTE crosses 64B line (unexpected)");
  }
  VectorData v = m_mem->Load(base);
  m_stats.walk_reads++;
  uint64_t val = 0;
  for (int i = 0; i < 8; ++i)
    val |= (static_cast<uint64_t>(v.GetU8Data(off + i)) << (8 * i));
  return val;
}

void MMU::issue_pt_read(WalkCtx* wctx, uint64_t pte_line_addr) {
  // 실제 Ramulator 경로로 보낼 mem_fetch 생성
  // 64B 라인을 읽어오도록 data_size=64로 설정 (ctrl=CXL_OVERHEAD)
  mem_fetch* mf = new mem_fetch(
      pte_line_addr, TLB_ACC_R, READ_REQUEST,
      /*data_size=*/64, CXL_OVERHEAD,
      /*timestamp=*/m_cfg ? m_cfg->get_ndp_cycle() : 0);
  mf->set_from_ndp(true);
  mf->set_ndp_id(m_ndp_id);
  if (m_cfg) mf->set_channel(m_cfg->get_channel_index(pte_line_addr));

  m_inflight[mf] = wctx;

  if (m_ptw_issue_latency > 0) {
    m_issue_q.push(mf, m_ptw_issue_latency);
  } else {
    // 즉시 발행 시도 (to-mem full이면 다음 cycle에 재시도해야 하므로 issue_q에 0으로 넣자)
    m_issue_q.push(mf, 0);
  }
}

} // namespace NDPSim
#endif // TIMING_SIMULATION
