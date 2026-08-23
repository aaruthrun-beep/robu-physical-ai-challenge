"""nite369_joy_serial.py
=======================
Standalone USB-joystick jog controller for the Nite369 6-DOF robot arm,
running on an "Uno Q" mini-PC and sending commands to the MASTER Pico over
USB serial (COM port). No Astra Studio, no Qt — just Python + pygame + pyserial.

This mirrors the joystick mapping used by Astra Studio:
    left stick X   -> J1   (inverted: stick right = J1 -)
    left stick Y   -> J2   (stick up = J2 +)
    right pad 0-3  -> J3 (up/down), J4 (left/right)
    buttons 4,5,6,7 -> J5 / J6 wrist combos:
        btn 4 or 6        -> J6 LEFT  (inverted)
        btn 5 or 7        -> J6 RIGHT (inverted)
        btn 4 + 5         -> J5 DOWN   (inverted)
        btn 6 + 7         -> J5 UP     (inverted)

Multi-axis is fully supported: hold the left stick diagonally AND press a
pad button — every held direction is jogged at once.

Protocol (same as robot_serial.py):
    #JC<joint>,<dir>,<speed>   start continuous jog (joint 1-6, dir +1/-1)
    #H                         halt ALL motion

Usage:
    python nite369_joy_serial.py                 # auto-detect COM port
    python nite369_joy_serial.py COM7            # force a COM port
    python nite369_joy_serial.py COM7 1500       # set jog speed (steps/s)
    python nite369_joy_serial.py --list          # list available COM ports
Press Ctrl+C to stop and halt the robot.
"""

import sys
import time

try:
    import serial
except ImportError:
    print("Need pyserial: pip install pyserial")
    sys.exit(1)

try:
    import pygame
except ImportError:
    print("Need pygame: pip install pygame")
    sys.exit(1)

# Default jog speed (steps/sec) sent in #JC. The master firmware scales
# per-joint; 2000 is a sane middle ground. Override on the command line.
DEFAULT_SPEED = 2000

# Debounce: require the joystick state to be stable for N consecutive polls
# (~60 ms at 20 ms/poll) before issuing #JC/#H. This stops stick jitter from
# flapping the SPI bus and corrupting the slave link (a real failure mode).
DEBOUNCE_POLLS = 3
POLL_DELAY = 0.02  # 50 Hz


def list_ports():
    """Return a list of available serial ports (or [] on failure)."""
    try:
        from serial.tools import list_ports
        return [p.device for p in list_ports.comports()]
    except Exception:
        return []


def open_serial(port, baud=115200):
    """Open the serial port. Tries the given port, else auto-detects."""
    if port is None:
        ports = list_ports()
        if not ports:
            print("No serial ports found. Pass a port: python nite369_joy_serial.py COMx")
            sys.exit(1)
        # Prefer something that looks like the Pico (CP210x / usbserial).
        for p in ports:
            if "CP210" in p.upper() or "USB" in p.upper() or "COM" in p.upper():
                port = p
                break
        else:
            port = ports[0]
        print(f"Auto-detected port: {port}")
    ser = serial.Serial(port, baud, timeout=0.05, write_timeout=1.0)
    time.sleep(0.3)          # let the master finish its boot banner
    ser.reset_input_buffer()
    return ser


def send(ser, cmd):
    """Write a single command line (with trailing newline)."""
    try:
        ser.write((cmd + "\n").encode())
    except serial.SerialException as e:
        print(f"!! serial error: {e}", flush=True)


def halt(ser):
    """Halt all motion (#H)."""
    send(ser, "H")


def jog_joint(ser, joint, direction, speed):
    """Start continuous jog of one joint (#JC)."""
    send(ser, f"JC{joint},{direction},{speed}")


def current_jogs(joy):
    """Return a SET of (joint_1based, direction) for ALL held directions.

    Both left-stick axes and every button are independent, so the result is
    a set — the caller jogs every active joint simultaneously.
    """
    axes = [joy.get_axis(i) for i in range(min(joy.get_numaxes(), 4))]
    buttons = [joy.get_button(i) for i in range(joy.get_numbuttons())]
    out = set()

    # Left stick: axis0 -> J1 (inverted), axis1 -> J2 (both can be active)
    if abs(axes[0]) > 0.5:
        out.add((1, -1 if axes[0] > 0 else 1))
    if abs(axes[1]) > 0.5:
        out.add((2, 1 if axes[1] > 0 else -1))

    # Right pad: 0=up, 1=right, 2=down, 3=left -> J3 (up/down), J4 (l/r)
    up = buttons[0] and not buttons[2]
    down = buttons[2] and not buttons[0]
    left = buttons[3] and not buttons[1]
    right = buttons[1] and not buttons[3]
    if up:
        out.add((3, -1))        # J3 DOWN (inverted)
    if down:
        out.add((3, 1))         # J3 UP (inverted)
    if left:
        out.add((4, -1))        # J4 left
    if right:
        out.add((4, 1))         # J4 right

    # Wrist: 4+5 -> J5 DOWN, 6+7 -> J5 UP, 4/6 -> J6 LEFT, 5/7 -> J6 RIGHT
    b4, b5, b6, b7 = buttons[4], buttons[5], buttons[6], buttons[7]
    if b4 and b5:
        out.add((5, -1))        # J5 DOWN (inverted)
    elif b6 and b7:
        out.add((5, 1))         # J5 UP (inverted)
    else:
        if b4 or b6:
            out.add((6, -1))    # J6 LEFT (inverted)
        if b5 or b7:
            out.add((6, 1))     # J6 RIGHT (inverted)
    return out


def main():
    args = sys.argv[1:]
    if "--list" in args:
        print("Available ports:", list_ports() or "none")
        return

    port = None
    speed = DEFAULT_SPEED
    for a in args:
        if a.upper().startswith("COM") or "USB" in a.upper():
            port = a
        elif a.isdigit():
            speed = int(a)

    ser = open_serial(port)
    print(f"Serial OK on {ser.port} @ {ser.baudrate}  (Ctrl+C to quit)")

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick found — connect one and re-run.")
        ser.close()
        sys.exit(1)
    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"Joystick: {joy.get_name()}  jog speed={speed}")

    active = set()          # currently jogging (joint, dir) set
    debounce_state = [None, 0]  # [last stable jogs, consecutive count]

    print("Jogging live — left stick = J1/J2, pad = J3/J4, wrist = J5/J6.")
    print("Hold multiple controls to jog multiple axes at once.")

    try:
        while True:
            pygame.event.pump()
            jogs = current_jogs(joy)

            # Debounce stick jitter.
            if jogs == debounce_state[0]:
                debounce_state[1] += 1
            else:
                debounce_state[0] = jogs
                debounce_state[1] = 1
            if debounce_state[1] < DEBOUNCE_POLLS:
                time.sleep(POLL_DELAY)
                continue

            if jogs != active:
                # Edge: halt everything, then start the new set. Sending #H
                # first (single-shot) clears any stacked per-joint #JC so the
                # new set runs clean — same trick as the firmware's #JC/#H.
                if active:
                    halt(ser)
                active = jogs
                for (j, d) in sorted(active):
                    print(f">> JC{j},{d} @ {speed}", flush=True)
                    jog_joint(ser, j, d, speed)
            time.sleep(POLL_DELAY)
    except KeyboardInterrupt:
        pass
    finally:
        halt(ser)
        print("\nHalted.")
        try:
            ser.close()
        except Exception:
            pass
        pygame.quit()


if __name__ == "__main__":
    main()
