#!/usr/bin/env python3
"""Minimal ELF -> UF2 converter for RP2040 (no picotool needed).

Reads an ARM ELF, extracts loadable FLASH segments (paddr in 0x10000000
XIP range), and writes a UF2 file. Handles both 32-bit (class 1) and
64-bit (class 2) ELFs.
"""
import struct
import sys


def align_up(v, a):
    return (v + a - 1) & ~(a - 1)


def read_elf(path):
    """Extract flash-resident PT_LOAD segments from an ELF.

    Uses program headers (not section headers) so RAM-initialized data
    (.data) whose LMA lives in flash is included, exactly like picotool.
    """
    with open(path, 'rb') as f:
        data = f.read()
    assert data[:4] == b'\x7fELF', 'not an ELF'
    is64 = data[4] == 2
    if is64:
        e_phoff = struct.unpack_from('<Q', data, 0x20)[0]
        e_phentsize = struct.unpack_from('<H', data, 0x36)[0]
        e_phnum = struct.unpack_from('<H', data, 0x38)[0]
        p_type_off, p_offset_off = 0x00, 0x08
        p_paddr_off, p_filesz_off = 0x18, 0x20
    else:
        e_phoff = struct.unpack_from('<I', data, 0x1C)[0]
        e_phentsize = struct.unpack_from('<H', data, 0x2A)[0]
        e_phnum = struct.unpack_from('<H', data, 0x2C)[0]
        p_type_off, p_offset_off = 0x00, 0x04
        p_paddr_off, p_filesz_off = 0x0C, 0x10

    FLASH_BASE = 0x10000000
    FLASH_MAX = 2 * 1024 * 1024
    segments = []
    for i in range(e_phnum):
        off = e_phoff + i * e_phentsize
        p_type = struct.unpack_from('<I', data, off + p_type_off)[0]
        p_offset = struct.unpack_from('<Q' if is64 else '<I', data, off + p_offset_off)[0]
        p_paddr = struct.unpack_from('<Q' if is64 else '<I', data, off + p_paddr_off)[0]
        p_filesz = struct.unpack_from('<Q' if is64 else '<I', data, off + p_filesz_off)[0]
        if p_type != 1:  # PT_LOAD
            continue
        if p_filesz == 0:
            continue  # .bss — nothing to flash
        if not (FLASH_BASE <= p_paddr < FLASH_BASE + FLASH_MAX):
            continue  # not flash-resident (e.g. RAM-only scratch)
        segments.append((p_paddr, data[p_offset:p_offset + p_filesz]))
    segments.sort()
    return segments


def to_uf2(segments, out_path):
    # Merge overlapping/adjacent segments
    merged = []
    for addr, blob in segments:
        if merged and addr <= merged[-1][0] + len(merged[-1][1]):
            prev_addr, prev_blob = merged[-1]
            end = max(prev_addr + len(prev_blob), addr + len(blob))
            merged[-1] = (prev_addr, prev_blob + b'\xff' * (end - (prev_addr + len(prev_blob))))
            # overwrite overlap
            merged[-1] = (prev_addr, merged[-1][1][:addr - prev_addr] + blob + merged[-1][1][addr - prev_addr + len(blob):])
        else:
            merged.append((addr, blob))

    # Extend each region start down to a 256-byte boundary so every block is
    # aligned (picotool pads similarly). Unaligned trailing segments (e.g.
    # RAM-initialized data or flash-resident config) are absorbed into the
    # preceding aligned region.
    aligned = []
    for addr, blob in merged:
        start = addr & ~0xFF
        if start < addr:
            blob = b'\xff' * (addr - start) + blob
            addr = start
        aligned.append((addr, blob))

    with open(out_path, 'wb') as f:
        total_blocks = sum((len(blob) + 255) // 256 for _, blob in aligned)
        blockno = 0
        for addr, blob in aligned:
            if addr % 256 != 0:
                raise ValueError('segment not 256-aligned')
            chunks = [blob[i:i + 256] for i in range(0, len(blob), 256)]
            for i, chunk in enumerate(chunks):
                fam = 0xE48BFF56  # RP2040
                block = struct.pack('<IIIIIIII', 0x0A324655, 0x9E5D5157,
                                    0x00002000, addr + i * 256, len(chunk), blockno,
                                    total_blocks, fam)
                f.write(block)
                f.write(chunk)
                f.write(b'\x00' * (476 - len(chunk)))
                f.write(b'\x00\x00\x00\x00')  # block end marker
                blockno += 1
    print(f'wrote {out_path}: {sum(len(b) for _, b in aligned)} bytes flash, {total_blocks} blocks')


if __name__ == '__main__':
    for src in sys.argv[1:]:
        segs = read_elf(src)
        to_uf2(segs, src.replace('.elf', '.uf2'))
