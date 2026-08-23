"""verify_firmware.py — Exercise each Nite 369 Pico's serial command set and report PASS/FAIL.

Usage:
    python verify_firmware.py                  # verify all mapped roles
    python verify_firmware.py master slave2    # specific roles
    python verify_firmware.py --motion         # also run small MOVEJ tests (motors move!)
    python verify_firmware.py --quick          # short timeouts, one shot

Exit code 0 = all tests passed, 1 = at least one failed.
"""
import argparse
import sys
import time

import pico_dev


def check(ser_port, name, cmd, expect_substr, wait=0.6):
    """Send cmd, assert the reply contains expect_substr. Returns (ok, reply)."""
    reply = pico_dev.send_cmd(ser_port, cmd, wait=wait)
    ok = expect_substr.lower() in reply.lower()
    status = "PASS" if ok else "FAIL"
    print(f"  [{status}] {name:<28} -> {reply.strip()[:100]}")
    return ok


def verify_master(port):
    print(f"\n== MASTER ({port}) ==")
    results = []
    results.append(check(port, "ID", "ID", "NITE-MASTER", wait=0.6))
    # MODE tells us USB or LAN mode; serial parser only active in USB mode.
    results.append(check(port, "MODE", "MODE", "MODE", wait=0.8))
    results.append(check(port, "STATUS", "STATUS", "POS", wait=1.0))
    results.append(check(port, "HELP", "HELP", "Commands:", wait=0.8))
    results.append(check(port, "STOP", "STOP", "OK", wait=0.8))
    results.append(check(port, "config dump", "$$", None, wait=1.0))
    r = pico_dev.send_cmd(port, "$$", wait=1.0)
    ok_cfg = bool(r.strip()) and "ERR" not in r
    print(f"  [{'PASS' if ok_cfg else 'FAIL'}] {'config dump (non-empty)':<28} -> {len(r.strip())} chars")
    results.append(ok_cfg)
    return all(results)


def verify_slave(port, role, motion=False):
    print(f"\n== {role.upper()} ({port}) ==")
    results = []
    tag = "NITE-SLAVE2" if role == "slave2" else "NITE-SLAVE1"
    results.append(check(port, "ID", "ID", tag, wait=0.6))
    results.append(check(port, "STATUS", "STATUS", "POS", wait=1.0))
    results.append(check(port, "STOP", "STOP", "OK", wait=0.8))
    if motion:
        results.append(check(port, "tiny move J1 +10", "MOVEJ 0.01 0 0 0", "OK", wait=1.0))
        results.append(check(port, "STOP after move", "STOP", "OK", wait=0.8))
    return all(results)


def main():
    ap = argparse.ArgumentParser(description="Verify Nite 369 firmware on connected Picos")
    ap.add_argument("roles", nargs="*", help="roles to verify (default: all mapped)")
    ap.add_argument("--motion", action="store_true", help="run small MOVEJ motion tests (motors move)")
    args = ap.parse_args()

    mapping = pico_dev.load_map()
    if args.roles:
        roles = args.roles
    else:
        roles = [r for r in ("master", "slave1", "slave2") if mapping.get(r)]

    if not roles:
        sys.exit("No roles to verify. Run pico_flash.py --learn <role> first.")

    picos = pico_dev.list_picos()
    by_serial = {p["serial"]: p for p in picos}

    all_ok = True
    for role in roles:
        serial = mapping.get(role)
        if not serial:
            print(f"\n== {role.upper()}: no serial mapped, SKIPPED ==")
            all_ok = False
            continue
        p = by_serial.get(serial)
        if not p:
            print(f"\n== {role.upper()}: serial {serial} NOT CONNECTED, SKIPPED ==")
            all_ok = False
            continue
        if role == "master":
            ok = verify_master(p["port"])
        else:
            ok = verify_slave(p["port"], role, motion=args.motion)
        if not ok:
            all_ok = False

    # Cross-check: any connected Pico answering ID with a role that isn't the
    # one we mapped should be flagged (catches wrong-flash mistakes).
    for p in picos:
        got = pico_dev.probe_id(p["port"])
        expected = next((r for r, s in mapping.items() if s == p["serial"]), None)
        if got and expected and got != expected:
            print(f"\n!! {p['port']} answers ID={got.upper()} but is mapped as {expected.upper()} — WRONG FIRMWARE!")
            all_ok = False

    print("\n=== VERIFICATION:", "ALL PASS" if all_ok else "FAILURES FOUND", "===")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
