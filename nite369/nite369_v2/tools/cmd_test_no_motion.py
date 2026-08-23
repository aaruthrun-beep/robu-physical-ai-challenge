#!/usr/bin/env python3
"""
Comprehensive no-motion command test for the v2 firmware.

Tests every command handler that does NOT cause joint movement.
Run:  python cmd_test_no_motion.py COM3
      python cmd_test_no_motion.py          # auto-detect master
"""

import serial, sys, time, json, os

# -- Config --
BAUD = 115200
TIMEOUT = 3.0         # seconds per command reply
SETTLE = 0.10         # seconds between commands
MAP_PATH = os.path.join(os.path.dirname(__file__), "pico_map.json")

# -- Helpers --
class Result:
    def __init__(self, name, passed, detail=""):
        self.name = name
        self.passed = passed
        self.detail = detail
    def __repr__(self):
        mark = "PASS" if self.passed else "FAIL"
        d = "  ({})".format(self.detail) if self.detail else ""
        return "[{}] {}{}".format(mark, self.name, d)

def send_cmd(ser, cmd, timeout=TIMEOUT):
    """Send a command, read until we see the >> response line.
    The master prints both << CMD (echo) and >> RESPONSE. We want the >> line."""
    ser.reset_input_buffer()
    payload = (cmd.strip() + "\n").encode()
    ser.write(payload)
    ser.flush()

    # Read all available data until we see a line starting with ">>" or timeout.
    buf = b""
    deadline = time.time() + timeout
    while time.time() < deadline:
        n = ser.in_waiting
        if n > 0:
            buf += ser.read(n)
            # Check if we have a >> line
            text = buf.decode("utf-8", errors="replace")
            lines = text.split("\n")
            for line in lines:
                line = line.strip()
                if line.startswith(">> "):
                    return line[3:].strip()  # strip ">> " prefix
            # If we see << line and timeout hasn't hit, keep reading
        else:
            time.sleep(0.01)

    # Fallback: return whatever we got
    text = buf.decode("utf-8", errors="replace")
    for line in text.split("\n"):
        line = line.strip()
        if line.startswith(">> "):
            return line[3:].strip()
    return text.strip()

def find_master():
    """Auto-detect master port from pico_map.json."""
    try:
        with open(MAP_PATH) as f:
            devmap = json.load(f)
        for entry in devmap:
            if entry.get("role") == "master":
                return entry["port"]
    except Exception:
        pass
    import serial.tools.list_ports
    for info in serial.tools.list_ports.comports():
        try:
            s = serial.Serial(info.device, BAUD, timeout=1)
            s.reset_input_buffer()
            s.write(b"#V\n")
            time.sleep(0.5)
            r = s.read(s.in_waiting or 64).decode(errors="replace")
            s.close()
            if ">V:" in r or "Nite369" in r:
                return info.device
        except Exception:
            pass
    return None

# -- Test suite --
def run_tests(ser):
    results = []
    R = results.append

    # === SECTION 1: Identity & status (no SPI) ===

    r = send_cmd(ser, "#V")
    ok = ">V:" in r and "Nite369" in r
    R(Result("#V version", ok, r[:60]))

    r = send_cmd(ser, "#ER")
    ok = ">ER:UNKNOWN_CMD" in r
    R(Result("#ER explicit error", ok, r[:40]))

    r = send_cmd(ser, "#PING")
    ok = ">PING:" in r and "S1=OK" in r and "S2=OK" in r
    R(Result("#PING slaves alive", ok, r[:60]))

    r = send_cmd(ser, "#S")
    ok = ">S:" in r and "IDLE" in r
    R(Result("#S status", ok, r[:40]))

    # === SECTION 2: Position & encoder reads ===

    r = send_cmd(ser, "#P")
    ok = ">P:" in r and "|" in r
    R(Result("#P position", ok, r[:80]))

    r = send_cmd(ser, "#E")
    ok = ">E:" in r
    angles = []
    if ok:
        try:
            vals = r.split(":")[1].split(",")
            angles = [float(v) for v in vals]
            ok = len(angles) == 6 and all(0 <= a <= 360 for a in angles)
        except Exception:
            ok = False
    R(Result("#E encoders", ok, "angles={}".format(
        ["{:.1f}".format(a) for a in angles]) if angles else r[:60]))

    r = send_cmd(ser, "#MS")
    ok = ">MS:" in r and ";" in r
    fields = r.split(":")[1].split(";") if ok else []
    ok2 = ok and len(fields) == 6
    if ok2:
        for f in fields:
            if len(f.split(",")) != 3:
                ok2 = False
    R(Result("#MS motion status", ok and ok2, r[:80]))

    r = send_cmd(ser, "#L")
    ok = ">L:" in r
    R(Result("#L limit switches", ok, r[:40]))

    # === SECTION 3: Enable / Disable drivers ===

    r = send_cmd(ser, "#ENFF")
    ok = ">OK" in r
    R(Result("#ENFF enable all", ok, r[:40]))

    r = send_cmd(ser, "#DIFF")
    ok = ">OK" in r
    R(Result("#DIFF disable all", ok, r[:40]))

    r = send_cmd(ser, "#EN07")
    ok = ">OK" in r
    R(Result("#EN07 enable slave1", ok, r[:40]))

    r = send_cmd(ser, "#DI07")
    ok = ">OK" in r
    R(Result("#DI07 disable slave1", ok, r[:40]))

    r = send_cmd(ser, "#EN38")
    ok = ">OK" in r
    R(Result("#EN38 enable slave2", ok, r[:40]))

    r = send_cmd(ser, "#DI38")
    ok = ">OK" in r
    R(Result("#DI38 disable slave2", ok, r[:40]))

    # === SECTION 4: Config read (no writes) ===

    for j in range(1, 7):
        r = send_cmd(ser, "#CR{}".format(j))
        ok = ">CR:" in r and "{},".format(j) in r
        parts = r.split(":")[1].split(",") if ok else []
        ok2 = ok and len(parts) >= 4
        R(Result("#CR{} config read".format(j), ok2, r[:60]))

    for j in range(1, 7):
        r = send_cmd(ser, "#CFG{}".format(j))
        ok = ">CFG:" in r and "{},".format(j) in r
        parts = r.split(":")[1].split(",") if ok else []
        ok2 = ok and len(parts) >= 4
        R(Result("#CFG{} ext config read".format(j), ok2, r[:60]))

    # === SECTION 5: Config write + read round-trip ===

    r = send_cmd(ser, "#CF2,3000,800,600,5000,2000")
    ok = ">OK" in r
    R(Result("#CF2 motion profile write", ok, r[:40]))

    r = send_cmd(ser, "#CR2")
    ok = ">CR:" in r
    parts = r.split(":")[1].split(",") if ok else []
    spd = int(parts[1]) if len(parts) > 1 else 0
    acc = int(parts[2]) if len(parts) > 2 else 0
    dec = int(parts[3]) if len(parts) > 3 else 0
    jd = int(parts[4]) if len(parts) > 4 else 0
    ja = int(parts[5]) if len(parts) > 5 else 0
    ok2 = ok and spd == 3000 and acc == 800 and dec == 600
    R(Result("#CF2->#CR2 profile round-trip", ok2,
             "spd={} acc={} dec={} jd={} ja={}".format(spd, acc, dec, jd, ja)))

    R(Result("#CF2 jog_decel round-trip", jd == 5000,
             "expected 5000 got {}".format(jd)))
    R(Result("#CF2 jog_accel round-trip", ja == 2000,
             "expected 2000 got {}".format(ja)))

    # Homing config round-trip
    r = send_cmd(ser, "#HC2,8000,200,400")
    ok = ">OK" in r
    R(Result("#HC2 homing config write", ok, r[:40]))

    r = send_cmd(ser, "#HG2")
    ok = ">HG:" in r
    parts = r.split(":")[1].split(",") if ok else []
    search = int(parts[1]) if len(parts) > 1 else 0
    creep = int(parts[2]) if len(parts) > 2 else 0
    bo = int(parts[3]) if len(parts) > 3 else 0
    R(Result("#HC2->#HG2 homing round-trip",
             search == 8000 and creep == 200 and bo == 400,
             "search={} creep={} backoff={}".format(search, creep, bo)))

    # Ext config round-trip
    r = send_cmd(ser, "#CFG2,200,200,0")
    ok = ">OK" in r
    R(Result("#CFG2 ext config write", ok, r[:40]))

    r = send_cmd(ser, "#CFG2")
    ok = ">CFG:" in r
    parts = r.split(":")[1].split(",") if ok else []
    spr = int(parts[1]) if len(parts) > 1 else 0
    gr = int(parts[2]) if len(parts) > 2 else 0
    di = int(parts[3]) if len(parts) > 3 else -1
    R(Result("#CFG2->#CFG2 ext round-trip",
             spr == 200 and gr == 200 and di == 0,
             "spr={} gr={} di={}".format(spr, gr, di)))

    # Write all joints
    r = send_cmd(ser, "#CFG0,200,200,0")
    ok = ">OK" in r
    R(Result("#CFG0 write all joints", ok, r[:40]))

    # Save to flash
    r = send_cmd(ser, "#CS")
    ok = ">OK" in r
    R(Result("#CS config save", ok, r[:40]))

    # === SECTION 6: Homing query ===

    for j in range(1, 7):
        r = send_cmd(ser, "#HQ{}".format(j))
        ok = ">HQ:" in r and "{},".format(j) in r
        parts = r.split(":")[1].split(",") if ok else []
        ok2 = ok and len(parts) >= 3
        R(Result("#HQ{} homing query".format(j), ok2, r[:60]))

    # === SECTION 7: Software home set ===

    r = send_cmd(ser, "#SH")
    ok = ">OK" in r
    R(Result("#SH set software home", ok, r[:40]))

    # === SECTION 8: Halt ===

    r = send_cmd(ser, "#H")
    ok = ">OK" in r
    R(Result("#H halt (idempotent)", ok, r[:40]))

    # === SECTION 9: LED ===

    r = send_cmd(ser, "#LED10,20,30")
    ok = ">OK" in r
    R(Result("#LED set color", ok, r[:40]))

    r = send_cmd(ser, "#LED0,0,0")
    ok = ">OK" in r
    R(Result("#LED off", ok, r[:40]))

    # === SECTION 10: TMC reads ===

    # TMC addr routing: addr<4 -> slave2 (TMC2209 UART), addr>=4 -> slave1 (no UART)
    r = send_cmd(ser, "#T0")
    ok_t0 = ">T:0," in r
    R(Result("#T0 TMC DRV_STATUS (slave2)", ok_t0, r[:60]))

    r = send_cmd(ser, "#TR0,00")
    ok_tr0 = ">TR:0," in r
    R(Result("#TR0,00 TMC reg read (slave2)", ok_tr0, r[:60]))

    r = send_cmd(ser, "#T4")
    ok_t4 = "TMC_READ_FAIL" in r or ">T:4," in r
    R(Result("#T4 TMC (slave1, expected fail)", ok_t4, r[:60]))

    # === SECTION 11: Invalid input edge cases ===

    r = send_cmd(ser, "#MV0,100,100")
    R(Result("#MV0 invalid joint", ">ER:" in r, r[:40]))

    r = send_cmd(ser, "#MV7,100,100")
    R(Result("#MV7 out of range", ">ER:" in r, r[:40]))

    r = send_cmd(ser, "#CF0,100,100,100")
    R(Result("#CF0 invalid joint", ">ER:" in r, r[:40]))

    r = send_cmd(ser, "#CF7,100,100,100")
    R(Result("#CF7 out of range", ">ER:" in r, r[:40]))

    r = send_cmd(ser, "#HQ0")
    R(Result("#HQ0 invalid homing query", ">ER:" in r, r[:40]))

    r = send_cmd(ser, "#HG7")
    R(Result("#HG7 out of range", ">ER:" in r, r[:40]))

    r = send_cmd(ser, "#ZYZ")
    R(Result("#ZYZ unknown command", ">ER:UNKNOWN_CMD" in r, r[:40]))

    # === SECTION 12: Position stability (3 reads) ===

    positions = []
    for i in range(3):
        r = send_cmd(ser, "#P")
        if ">P:" in r:
            try:
                vals = r.split(":")[1].split("|")[0].split(",")
                positions.append([float(v) for v in vals])
            except Exception:
                pass
        time.sleep(SETTLE)
    stable = (len(positions) == 3 and
              all(len(p) == 6 for p in positions) and
              all(abs(positions[0][i] - positions[2][i]) < 0.01
                  for i in range(6)))
    R(Result("#P position stability (3 reads)", stable,
             "read0={}".format(
                 ["{:.2f}".format(p) for p in positions[0]])
             if positions else "FAIL"))

    # === SECTION 13: Heartbeat survived ===

    r = send_cmd(ser, "#PING")
    ok = ">PING:" in r and "S1=OK" in r and "S2=OK" in r
    R(Result("#PING after all tests (survivability)", ok, r[:60]))

    return results


# -- Main --
def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_master()
    if not port:
        print("ERROR: No master Pico found. Pass COM port or check pico_map.json.")
        sys.exit(1)

    print("Connecting to {} @ {}...".format(port, BAUD))
    ser = serial.Serial(port, BAUD, timeout=TIMEOUT)
    time.sleep(0.5)

    # Drain any boot banner
    ser.reset_input_buffer()
    time.sleep(0.3)
    banner = ser.read(ser.in_waiting or 256)
    if banner:
        print("Boot banner: {}".format(
            banner.decode(errors="replace").strip()[:80]))

    print("\nRunning no-motion command test on {}...\n".format(port))
    results = run_tests(ser)
    ser.close()

    # Summary
    passed = sum(1 for r in results if r.passed)
    failed = sum(1 for r in results if not r.passed)
    total = len(results)

    for r in results:
        print(r)

    print("\n" + "=" * 60)
    print("TOTAL: {}  PASSED: {}  FAILED: {}".format(total, passed, failed))
    if failed == 0:
        print("ALL TESTS PASSED - v2 protocol verified, no motion executed.")
    else:
        print("{} test(s) FAILED - see details above.".format(failed))
    print("=" * 60)

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
