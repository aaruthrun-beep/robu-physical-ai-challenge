#!/usr/bin/env python3
"""
Comprehensive no-motion command test for the v2 firmware — TCP/Ethernet mode.

Identical test suite as cmd_test_no_motion.py but runs over TCP (port 23).
Run:  python cmd_test_ethernet.py [host] [port]
      python cmd_test_ethernet.py                  # defaults to 192.168.1.50:23
"""

import socket, sys, time

# -- Config --
DEFAULT_HOST = "192.168.1.50"
DEFAULT_PORT = 23
TIMEOUT = 3.0
SETTLE = 0.10


class Result:
    def __init__(self, name, passed, detail=""):
        self.name = name
        self.passed = passed
        self.detail = detail
    def __repr__(self):
        mark = "PASS" if self.passed else "FAIL"
        d = "  ({})".format(self.detail) if self.detail else ""
        return "[{}] {}{}".format(mark, self.name, d)


class TcpClient:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None
        self.connect()

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(TIMEOUT)
        self.sock.connect((self.host, self.port))
        # Drain any banner
        self.sock.settimeout(0.5)
        try:
            self.sock.recv(4096)
        except socket.timeout:
            pass
        self.sock.settimeout(TIMEOUT)

    def send_cmd(self, cmd):
        """Send a command, read until we see the >> response line."""
        self.sock.settimeout(0.05)
        # Clear any stale data
        try:
            while True:
                self.sock.recv(4096)
        except (socket.timeout, ConnectionError):
            pass
        self.sock.settimeout(TIMEOUT)

        payload = (cmd.strip() + "\n").encode()
        self.sock.send(payload)

        buf = b""
        deadline = time.time() + TIMEOUT
        while time.time() < deadline:
            try:
                chunk = self.sock.recv(4096)
                if chunk:
                    buf += chunk
                    text = buf.decode("utf-8", errors="replace")
                    for line in text.split("\n"):
                        line = line.strip()
                        if line.startswith(">> "):
                            return line[3:].strip()
                else:
                    break  # connection closed
            except socket.timeout:
                continue

        text = buf.decode("utf-8", errors="replace")
        for line in text.split("\n"):
            line = line.strip()
            if line.startswith(">> "):
                return line[3:].strip()
        return text.strip()

    def close(self):
        if self.sock:
            self.sock.close()


def run_tests(c):
    results = []
    R = results.append

    # === SECTION 1: Identity & status ===

    r = c.send_cmd("#V")
    ok = ">V:" in r and "Nite369" in r
    R(Result("#V version", ok, r[:60]))

    r = c.send_cmd("#ER")
    ok = ">ER:UNKNOWN_CMD" in r
    R(Result("#ER explicit error", ok, r[:40]))

    r = c.send_cmd("#PING")
    ok = ">PING:" in r and "S1=OK" in r and "S2=OK" in r
    R(Result("#PING slaves alive", ok, r[:60]))

    r = c.send_cmd("#S")
    ok = ">S:" in r and "IDLE" in r
    R(Result("#S status", ok, r[:40]))

    # === SECTION 2: Position & encoder reads ===

    r = c.send_cmd("#P")
    ok = ">P:" in r and "|" in r
    R(Result("#P position", ok, r[:80]))

    r = c.send_cmd("#E")
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

    r = c.send_cmd("#MS")
    ok = ">MS:" in r and ";" in r
    fields = r.split(":")[1].split(";") if ok else []
    ok2 = ok and len(fields) == 6
    if ok2:
        for f in fields:
            if len(f.split(",")) != 3:
                ok2 = False
    R(Result("#MS motion status", ok and ok2, r[:80]))

    r = c.send_cmd("#L")
    ok = ">L:" in r
    R(Result("#L limit switches", ok, r[:40]))

    # === SECTION 3: Enable / Disable drivers ===

    r = c.send_cmd("#ENFF")
    ok = ">OK" in r
    R(Result("#ENFF enable all", ok, r[:40]))

    r = c.send_cmd("#DIFF")
    ok = ">OK" in r
    R(Result("#DIFF disable all", ok, r[:40]))

    r = c.send_cmd("#EN07")
    ok = ">OK" in r
    R(Result("#EN07 enable slave1", ok, r[:40]))

    r = c.send_cmd("#DI07")
    ok = ">OK" in r
    R(Result("#DI07 disable slave1", ok, r[:40]))

    r = c.send_cmd("#EN38")
    ok = ">OK" in r
    R(Result("#EN38 enable slave2", ok, r[:40]))

    r = c.send_cmd("#DI38")
    ok = ">OK" in r
    R(Result("#DI38 disable slave2", ok, r[:40]))

    # === SECTION 4: Config read ===

    for j in range(1, 7):
        r = c.send_cmd("#CR{}".format(j))
        ok = ">CR:" in r and "{},".format(j) in r
        parts = r.split(":")[1].split(",") if ok else []
        ok2 = ok and len(parts) >= 4
        R(Result("#CR{} config read".format(j), ok2, r[:60]))

    for j in range(1, 7):
        r = c.send_cmd("#CFG{}".format(j))
        ok = ">CFG:" in r and "{},".format(j) in r
        parts = r.split(":")[1].split(",") if ok else []
        ok2 = ok and len(parts) >= 4
        R(Result("#CFG{} ext config read".format(j), ok2, r[:60]))

    # === SECTION 5: Config write + read round-trip ===

    r = c.send_cmd("#CF2,3000,800,600,5000,2000")
    ok = ">OK" in r
    R(Result("#CF2 motion profile write", ok, r[:40]))

    r = c.send_cmd("#CR2")
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

    # Homing config
    r = c.send_cmd("#HC2,8000,200,400")
    ok = ">OK" in r
    R(Result("#HC2 homing config write", ok, r[:40]))

    r = c.send_cmd("#HG2")
    ok = ">HG:" in r
    parts = r.split(":")[1].split(",") if ok else []
    search = int(parts[1]) if len(parts) > 1 else 0
    creep = int(parts[2]) if len(parts) > 2 else 0
    bo = int(parts[3]) if len(parts) > 3 else 0
    R(Result("#HC2->#HG2 homing round-trip",
             search == 8000 and creep == 200 and bo == 400,
             "search={} creep={} backoff={}".format(search, creep, bo)))

    # Ext config
    r = c.send_cmd("#CFG2,200,200,0")
    ok = ">OK" in r
    R(Result("#CFG2 ext config write", ok, r[:40]))

    r = c.send_cmd("#CFG2")
    ok = ">CFG:" in r
    parts = r.split(":")[1].split(",") if ok else []
    spr = int(parts[1]) if len(parts) > 1 else 0
    gr = int(parts[2]) if len(parts) > 2 else 0
    di = int(parts[3]) if len(parts) > 3 else -1
    R(Result("#CFG2->#CFG2 ext round-trip",
             spr == 200 and gr == 200 and di == 0,
             "spr={} gr={} di={}".format(spr, gr, di)))

    r = c.send_cmd("#CFG0,200,200,0")
    ok = ">OK" in r
    R(Result("#CFG0 write all joints", ok, r[:40]))

    r = c.send_cmd("#CS")
    ok = ">OK" in r
    R(Result("#CS config save", ok, r[:40]))

    # === SECTION 6: Homing query ===

    for j in range(1, 7):
        r = c.send_cmd("#HQ{}".format(j))
        ok = ">HQ:" in r and "{},".format(j) in r
        parts = r.split(":")[1].split(",") if ok else []
        ok2 = ok and len(parts) >= 3
        R(Result("#HQ{} homing query".format(j), ok2, r[:60]))

    # === SECTION 7: Software home set ===

    r = c.send_cmd("#SH")
    ok = ">OK" in r
    R(Result("#SH set software home", ok, r[:40]))

    # === SECTION 8: Halt ===

    r = c.send_cmd("#H")
    ok = ">OK" in r
    R(Result("#H halt (idempotent)", ok, r[:40]))

    # === SECTION 9: LED ===

    r = c.send_cmd("#LED10,20,30")
    ok = ">OK" in r
    R(Result("#LED set color", ok, r[:40]))

    r = c.send_cmd("#LED0,0,0")
    ok = ">OK" in r
    R(Result("#LED off", ok, r[:40]))

    # === SECTION 10: TMC reads ===

    r = c.send_cmd("#T0")
    ok_t0 = ">T:0," in r
    R(Result("#T0 TMC DRV_STATUS (slave2)", ok_t0, r[:60]))

    r = c.send_cmd("#TR0,00")
    ok_tr0 = ">TR:0," in r
    R(Result("#TR0,00 TMC reg read (slave2)", ok_tr0, r[:60]))

    r = c.send_cmd("#T4")
    ok_t4 = "TMC_READ_FAIL" in r or ">T:4," in r
    R(Result("#T4 TMC (slave1, expected fail)", ok_t4, r[:60]))

    # === SECTION 11: Invalid input edge cases ===

    r = c.send_cmd("#MV0,100,100")
    R(Result("#MV0 invalid joint", ">ER:" in r, r[:40]))

    r = c.send_cmd("#MV7,100,100")
    R(Result("#MV7 out of range", ">ER:" in r, r[:40]))

    r = c.send_cmd("#CF0,100,100,100")
    R(Result("#CF0 invalid joint", ">ER:" in r, r[:40]))

    r = c.send_cmd("#CF7,100,100,100")
    R(Result("#CF7 out of range", ">ER:" in r, r[:40]))

    r = c.send_cmd("#HQ0")
    R(Result("#HQ0 invalid homing query", ">ER:" in r, r[:40]))

    r = c.send_cmd("#HG7")
    R(Result("#HG7 out of range", ">ER:" in r, r[:40]))

    r = c.send_cmd("#ZYZ")
    R(Result("#ZYZ unknown command", ">ER:UNKNOWN_CMD" in r, r[:40]))

    # === SECTION 12: Position stability (3 reads) ===

    positions = []
    for i in range(3):
        r = c.send_cmd("#P")
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

    r = c.send_cmd("#PING")
    ok = ">PING:" in r and "S1=OK" in r and "S2=OK" in r
    R(Result("#PING after all tests (survivability)", ok, r[:60]))

    # === SECTION 14: Concurrent connections (second client) ===

    try:
        c2 = TcpClient(c.host, c.port)
        r2 = c2.send_cmd("#V")
        R(Result("#V from 2nd TCP client", ">V:" in r2 and "Nite369" in r2, r2[:60]))
        c2.close()
    except Exception as e:
        R(Result("#V from 2nd TCP client", False, str(e)[:60]))

    # Reconnect first client (may have been displaced)
    try:
        c.close()
        c.connect()
        r = c.send_cmd("#PING")
        R(Result("#PING after 2nd client disconnect",
                 ">PING:" in r and "S1=OK" in r and "S2=OK" in r, r[:60]))
    except Exception as e:
        R(Result("#PING after 2nd client disconnect", False, str(e)[:60]))

    return results


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_HOST
    port = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_PORT

    print("Connecting to {}:{} (TCP)...".format(host, port))
    try:
        c = TcpClient(host, port)
    except Exception as e:
        print("ERROR: Cannot connect to {}:{} - {}".format(host, port, e))
        sys.exit(1)
    print("Connected.\n")

    print("Running no-motion command test over Ethernet...\n")
    results = run_tests(c)
    c.close()

    # Summary
    passed = sum(1 for r in results if r.passed)
    failed = sum(1 for r in results if not r.passed)
    total = len(results)

    for r in results:
        print(r)

    print("\n" + "=" * 60)
    print("TOTAL: {}  PASSED: {}  FAILED: {}".format(total, passed, failed))
    if failed == 0:
        print("ALL TESTS PASSED - Ethernet v2 protocol verified, no motion.")
    else:
        print("{} test(s) FAILED - see details above.".format(failed))
    print("=" * 60)

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
