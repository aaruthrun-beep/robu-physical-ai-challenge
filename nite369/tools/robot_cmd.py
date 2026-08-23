"""Nite369 fast robot control - one command per invocation, instant reply.

Usage:
    python robot_cmd.py PING                 # check both slaves
    python robot_cmd.py V                    # version
    python robot_cmd.py EN3F                 # enable motors
    python robot_cmd.py MV1,500,200          # raw command (auto-# prefix)
    python robot_cmd.py move 1 200           # joint 1, +200 steps @ 500/s
    python robot_cmd.py move 1 -200          # joint 1, -200 steps
    python robot_cmd.py move 2 100 1000      # joint 2, 100 steps @ 1000/s
    python robot_cmd.py grip 500             # gripper +500 steps
    python robot_cmd.py grip -500            # gripper -500 steps
    python robot_cmd.py halt                 # emergency stop
    python robot_cmd.py pos                  # positions
    python robot_cmd.py lim                  # limit switches
    echo "#PING" | python robot_cmd.py       # pipe a raw command
"""

import socket
import sys
import time

HOST = "192.168.1.50"
PORT = 23


def raw_to_cmd(raw):
    raw = raw.strip()
    if not raw:
        return None
    if not raw.startswith("#"):
        raw = "#" + raw
    return raw


def translate(arg):
    a = [x for x in arg.split() if x]
    if not a:
        return None
    head = a[0].lower()
    presets = {
        "ping": "#PING", "p": "#PING", "v": "#V", "version": "#V",
        "halt": "#H", "stop": "#H", "h": "#H", "en": "#EN3F",
        "di": "#DI3F", "pos": "#P", "lim": "#L", "stat": "#S",
        "ms": "#MS", "led": "#LED0,255,0",
    }
    if head in presets:
        return presets[head]
    if head == "move":
        j = a[1] if len(a) > 1 else "1"
        steps = a[2] if len(a) > 2 else "200"
        speed = a[3] if len(a) > 3 else "500"
        return f"#MV{j},{speed},{steps}"
    if head == "grip" or head == "gripper" or head == "g":
        steps = a[1] if len(a) > 1 else "500"
        return f"#G{steps}"
    if head == "mv":
        return "#" + arg.replace("mv ", "MV", 1)
    return raw_to_cmd(arg)


def main():
    cmd = None
    if not sys.stdin.isatty():
        data = sys.stdin.read().strip()
        if data:
            cmd = raw_to_cmd(data.splitlines()[0])
    if cmd is None and len(sys.argv) > 1:
        cmd = translate(" ".join(sys.argv[1:]))
    if cmd is None:
        print(__doc__)
        sys.exit(2)

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(3.0)
    try:
        s.connect((HOST, PORT))
    except OSError as e:
        print(f"CONNECT FAIL: {e}")
        sys.exit(1)
    s.settimeout(0.8)
    t0 = time.time()
    try:
        s.sendall((cmd + "\n").encode("ascii"))
    except OSError as e:
        print(f"SEND FAIL: {e}")
        sys.exit(1)
    buf = b""
    while time.time() - t0 < 2.5:
        try:
            chunk = s.recv(4096)
            if not chunk:
                break
            buf += chunk
        except socket.timeout:
            break
        except OSError:
            break
    s.close()
    if buf:
        print(buf.decode("ascii", errors="replace").strip())
    else:
        print("(no reply)")


if __name__ == "__main__":
    main()
