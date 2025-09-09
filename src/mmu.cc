// mmu.cc
#include "mmu.h"
#include "memory_map.h"
#include "vector_data.h"
#include <stdexcept>

NDPSim::MMU::MMU(NDPSim::MemoryMap* mem, uint64_t pt_base, uint64_t page_size)
  : m_mem(mem), m_pt_base(pt_base), m_page_size(page_size)
{
  uint64_t ps = page_size; 
  uint32_t sh = 0;
  while ((1ULL << sh) < ps) sh++;
  m_page_shift = sh;
}

bool NDPSim::MMU::Translate(uint64_t va, uint64_t& pa_out, bool /*is_write*/) {
  const uint64_t pml4_i   = (va >> 39) & 0x1FF;
  const uint64_t pdpt_i   = (va >> 30) & 0x1FF;
  const uint64_t pd_i     = (va >> 21) & 0x1FF;
  const uint64_t pt_i     = (va >> 12) & 0x1FF;
  const uint64_t page_off = va & (m_page_size - 1);

  uint64_t pml4_addr = m_pt_base + pml4_i * 8;
  uint64_t pml4e     = read_qword(pml4_addr);
  if (!(pml4e & 0x1)) { m_stats.fails++; return false; }

  uint64_t pdpt_base = pml4e & ~0xFFFULL;
  uint64_t pdpte     = read_qword(pdpt_base + pdpt_i * 8);
  if (!(pdpte & 0x1)) { m_stats.fails++; return false; }

  uint64_t pd_base = pdpte & ~0xFFFULL;
  uint64_t pde     = read_qword(pd_base + pd_i * 8);
  if (!(pde & 0x1)) { m_stats.fails++; return false; }

  uint64_t pt_base = pde & ~0xFFFULL;
  uint64_t pte     = read_qword(pt_base + pt_i * 8);
  if (!(pte & 0x1)) { m_stats.fails++; return false; }

  uint64_t frame = pte & ~0xFFFULL;
  pa_out = frame | page_off;
  m_stats.hits++;

  return true;
}

uint64_t NDPSim::MMU::read_qword(uint64_t phys_addr) {
  const uint64_t LINE = 64;
  uint64_t base = phys_addr & ~(LINE - 1);
  uint32_t off  = static_cast<uint32_t>(phys_addr - base);
  if (off > (LINE - 8)) {
    throw std::runtime_error("MMU: PTE crosses 64B line (unexpected)");
  }

  NDPSim::VectorData v = m_mem->Load(base);
  m_stats.walk_reads++;

  uint64_t val = 0;
  for (int i = 0; i < 8; ++i) {
    val |= (static_cast<uint64_t>(v.GetU8Data(off + i)) << (8 * i));
  }
  return val;
}
