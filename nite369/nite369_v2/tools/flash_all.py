#!/usr/bin/env python3
"""
flash_all.py — build all three production targets and convert to .uf2.

Usage:
    python tools/flash_all.py                # build + convert (default)
    python tools/flash_all.py --no-build     # only convert existing .elf
    python tools/flash_all.py --picotool PATH  # override picotool location

Requires CMake, Ninja and the Pico SDK (set PICO_SDK_PATH or pass --sdk).
The picotool default matches the known-good v1 location; pass --picotool
if yours differs.
"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / "build"

TARGETS = ["spi_cmd_master_eth", "spi_cmd_slave_s1", "spi_cmd_slave_s2"]

DEFAULT_PICOTOOL = r"C:\Users\aarut\.pico-sdk\picotool\2.2.0-a4\picotool\picotool.exe"
DEFAULT_SDK = r"C:\pico\pico-sdk"


def run(cmd, **kw):
    print("+", " ".join(str(c) for c in cmd))
    subprocess.run(cmd, check=True, **kw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-build", action="store_true", help="only convert existing .elf")
    ap.add_argument("--picotool", default=None, help="path to picotool.exe")
    ap.add_argument("--sdk", default=None, help="PICO_SDK_PATH")
    args = ap.parse_args()

    picotool = args.picotool or os.environ.get("PICOTOOL", DEFAULT_PICOTOOL)
    if not os.path.exists(picotool):
        print(f"ERROR: picotool not found at {picotool}", file=sys.stderr)
        sys.exit(1)

    if not args.no_build:
        if not (BUILD / "CMakeCache.txt").exists():
            sdk = args.sdk or os.environ.get("PICO_SDK_PATH", DEFAULT_SDK)
            run(["cmake", "-B", str(BUILD), "-GNinja",
                 f"-DPICO_SDK_PATH={sdk}", "-DPICO_NO_PICOTOOL=1",
                 "-DCMAKE_BUILD_TYPE=Release"], cwd=ROOT)
        run(["cmake", "--build", str(BUILD), "--target"] + TARGETS)

    for t in TARGETS:
        elf = BUILD / f"{t}.elf"
        uf2 = BUILD / f"{t}.uf2"
        if not elf.exists():
            print(f"SKIP {t}: {elf} missing", file=sys.stderr)
            continue
        run([picotool, "uf2", "convert", str(elf), str(uf2)])
        print(f"  -> {uf2} ({uf2.stat().st_size} bytes)")

    print("DONE. Flash order: Slave 1 -> Slave 2 -> Master.")


if __name__ == "__main__":
    main()
