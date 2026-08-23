"""
Nite369 TCP Client — Connect to robot via Ethernet (W5500).

Usage:
    python nite369_tcp.py                    # default 192.168.1.50:23
    python nite369_tcp.py 192.168.1.50       # custom IP
    python nite369_tcp.py 192.168.1.50 23    # custom IP + port
"""

import sys
import socket
import threading
import time
import argparse


def recv_loop(sock):
    """Background thread: receives data from robot."""
    sock.settimeout(1.0)
    while True:
        try:
            data = sock.recv(4096)
            if not data:
                print("\n[Disconnected]")
                break
            text = data.decode("ascii", errors="replace").strip()
            if text:
                for line in text.split("\n"):
                    if line.strip():
                        print(f"< {line.strip()}")
        except socket.timeout:
            continue
        except OSError:
            break


def main():
    parser = argparse.ArgumentParser(description="Nite369 TCP Client")
    parser.add_argument("host", nargs="?", default="192.168.1.50")
    parser.add_argument("port", nargs="?", type=int, default=23)
    args = parser.parse_args()

    print(f"Connecting to {args.host}:{args.port}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    try:
        sock.connect((args.host, args.port))
    except (socket.timeout, ConnectionRefusedError) as e:
        print(f"Connection failed: {e}")
        sys.exit(1)

    print(f"Connected! Type commands or 'quit' to exit.\n")

    # Start receiver thread
    t = threading.Thread(target=recv_loop, args=(sock,), daemon=True)
    t.start()

    # Preset commands
    presets = {
        "v":    "#V",
        "pos":  "#P",
        "enc":  "#E",
        "lim":  "#L",
        "stat": "#S",
        "en":   "#EN3F",
        "di":   "#DI3F",
        "en1":  "#EN07",
        "halt": "#H",
        "ms":   "#MS",
        "home": "#HM0",
        "led":  "#LED255,0,0",
        "ledoff": "#LED0,0,0,255",
    }

    try:
        while True:
            try:
                cmd = input(">>> ").strip()
            except EOFError:
                break

            if not cmd:
                continue
            if cmd.lower() in ("quit", "exit", "q"):
                break

            # Expand presets
            if cmd.lower() in presets:
                cmd = presets[cmd.lower()]
                print(f"  [{cmd}]")

            # Ensure # prefix
            if not cmd.startswith("#"):
                cmd = "#" + cmd

            try:
                sock.sendall((cmd + "\n").encode("ascii"))
            except OSError as e:
                print(f"Send error: {e}")
                break

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[Interrupted]")
    finally:
        sock.close()
        print("Disconnected.")


if __name__ == "__main__":
    main()
