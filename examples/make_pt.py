#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
make_pt.py
- addr_log.txt를 읽어 4레벨(x86-64, 4KB) 페이지 테이블을 구성하고,
  메모리맵 형식(uint8, 64B line)으로 <kernel>_pt.data 를 생성합니다.

주요 옵션
--pt_base   : PML4 물리 베이스(CR3). 예) 0x900000000000 또는 0x0009000000000000
--addr_shift: addr_log가 VPN(VA>>shift)이면 그 shift (기본 12), 바이트 주소면 0
"""

import argparse
import os
import re

PAGE_SIZE       = 4096
ENTRIES_PER_PT  = 512
ENTRY_SIZE      = 8
LINE_SIZE       = 64  # 64B 라인

PTE_FLAGS = 0x3       # Present | RW

ADDR_RE = re.compile(r'0x([0-9a-fA-F]+)')

SPECIAL_KERNEL_MAP = {
    'LtINT64Kernel': 'lt_int64',
    'GtEqLtINT64Kernel': 'gteq_lt_int64',
    'GtLtFP32Kernel': 'gt_lt_fp32',
    'TwoColANDKernel': 'two_col_and',
    'ThreeColANDKernel': 'three_col_and',
}

def kernel_to_basename(kname: str) -> str:
    if kname in SPECIAL_KERNEL_MAP:
        return SPECIAL_KERNEL_MAP[kname]
    import re as _re
    s = _re.sub('(.)([A-Z][a-z]+)', r'\1_\2', kname)
    s = _re.sub('([a-z0-9])([A-Z])', r'\1_\2', s).lower()
    if s.endswith('_kernel'):
        s = s[:-7]
    return s

def align_down(x: int, a: int) -> int:
    return x & ~(a - 1)

def line_base(addr: int) -> int:
    return addr & ~(LINE_SIZE - 1)

def write_qword(mem_lines: dict, page_addr: int, index: int, value: int):
    """page_addr 페이지의 index 번째 엔트리에 8B little-endian으로 value 기록."""
    offset = index * ENTRY_SIZE
    lb = line_base(page_addr + offset)
    pos = (page_addr + offset) - lb
    buf = mem_lines.setdefault(lb, bytearray(LINE_SIZE))
    for i in range(8):
        buf[pos + i] = (value >> (8 * i)) & 0xFF

def indices_from_va(va: int):
    """x86-64 4레벨 인덱스 추출 (48-bit canonical 가정)"""
    pml4 = (va >> 39) & 0x1ff
    pdpt = (va >> 30) & 0x1ff
    pd   = (va >> 21) & 0x1ff
    pt   = (va >> 12) & 0x1ff
    return pml4, pdpt, pd, pt

def read_unique_pages(addr_log_path: str, page_size: int, addr_shift: int):
    unique_pages = set()
    total = 0
    with open(addr_log_path, 'r') as f:
        for line in f:
            m = ADDR_RE.search(line)
            if not m:
                continue
            addr = int(m.group(1), 16)
            if addr_shift:
                addr = addr << addr_shift
            page = align_down(addr, page_size)
            unique_pages.add(page)
            total += 1
    return unique_pages, total

def build_pt(unique_pages, pt_base):
    """
    unique_pages: VA 페이지들의 집합 (또는 복원된 VA)
    반환: mem_lines(dict: line_addr -> 64B bytes)
    """
    mem_lines = {}  # 64B line 단위 기록

    # 테이블 페이지 할당 상태
    next_free = pt_base + PAGE_SIZE  # PML4 다음 페이지부터
    pml4_page = pt_base

    # 계층별 이미 할당된 child 테이블
    pdpt_pages = {}  # key: pml4_idx -> phys
    pd_pages   = {}  # key: (pml4_idx, pdpt_idx) -> phys
    pt_pages   = {}  # key: (pml4_idx, pdpt_idx, pd_idx) -> phys

    for va_page in sorted(unique_pages):
        pml4_i, pdpt_i, pd_i, pt_i = indices_from_va(va_page)

        # 1) PML4 -> PDPT
        if pml4_i not in pdpt_pages:
            pdpt_phys = next_free
            next_free += PAGE_SIZE
            pdpt_pages[pml4_i] = pdpt_phys
            write_qword(mem_lines, pml4_page, pml4_i, pdpt_phys | PTE_FLAGS)

        # 2) PDPT -> PD
        key2 = (pml4_i, pdpt_i)
        if key2 not in pd_pages:
            pd_phys = next_free
            next_free += PAGE_SIZE
            pd_pages[key2] = pd_phys
            write_qword(mem_lines, pdpt_pages[pml4_i], pdpt_i, pd_phys | PTE_FLAGS)

        # 3) PD -> PT
        key3 = (pml4_i, pdpt_i, pd_i)
        if key3 not in pt_pages:
            _pt_phys = next_free
            next_free += PAGE_SIZE
            pt_pages[key3] = _pt_phys
            write_qword(mem_lines, pd_pages[key2], pd_i, _pt_phys | PTE_FLAGS)

        # 4) PT leaf: (임시) VA와 동일한 프레임으로 아이덴티티 매핑
        write_qword(mem_lines, pt_pages[key3], pt_i, va_page | PTE_FLAGS)

    return mem_lines

def dump_memmap_uint8(mem_lines: dict, out_path: str):
    with open(out_path, 'w') as f:
        f.write("_META_\nuint8\n_DATA_\n")
        for addr in sorted(mem_lines.keys()):
            bytes64 = mem_lines[addr]
            # 주소 16자리 0패딩
            f.write(f"0x{addr:016x} " + " ".join(str(b) for b in bytes64) + "\n")

def parse_int_auto(s: str) -> int:
    """'0x...' 또는 10진 모두 허용"""
    return int(s, 0)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--kernel', type=str, required=True, help='kernel name (e.g., LtINT64Kernel)')
    ap.add_argument('--input_dir', type=str, required=True, help='dir that contains addr_log.txt')
    ap.add_argument('--output_dir', type=str, required=True, help='where to write <kernel>_pt.data')
    ap.add_argument('--addr_shift', type=int, default=12,
                    help='addr_log가 VPN(VA>>shift)라면 그 shift 값 (기본 12). 바이트 주소면 0.')
    ap.add_argument('--pt_base', type=parse_int_auto, default=parse_int_auto("0x0009000000000000"),
                    help='PML4 물리 베이스(CR3). 예) 0x900000000000 또는 0x0009000000000000')
    ap.add_argument('--page_size', type=int, default=PAGE_SIZE)
    args = ap.parse_args()

    addr_log = os.path.join(args.input_dir, "addr_log.txt")
    if not os.path.isfile(addr_log):
        raise FileNotFoundError(f"addr_log.txt not found in {args.input_dir}")

    base = kernel_to_basename(args.kernel)
    out_name = f"{base}_pt.data"
    os.makedirs(args.output_dir, exist_ok=True)
    out_path = os.path.join(args.output_dir, out_name)

    unique_pages, total_accesses = read_unique_pages(addr_log, args.page_size, args.addr_shift)
    mem_lines = build_pt(unique_pages, args.pt_base)

    # 통계
    lines_emitted   = len(mem_lines)
    pt_pages_alloc  = len({align_down(a, args.page_size) for a in mem_lines.keys()})

    print(f"[{base}] total accesses={total_accesses}, unique 4KB pages={len(unique_pages)}")
    print(f"  PML4 base = 0x{args.pt_base:016x}")
    print(f"  allocated PT pages = {pt_pages_alloc}")
    print(f"  emitted 64B lines  = {lines_emitted}")

    dump_memmap_uint8(mem_lines, out_path)
    print(f"  -> wrote {out_path}")

if __name__ == "__main__":
    main()
