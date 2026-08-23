"""JoystickController — polls a USB joystick and maps it to robot joint
jog commands for the Astra Studio.

Device: "USB Joystick" (VID 0079) — digital/hybrid.
  axis 0 (left stick X)  -> joint 1  (left/right)
  axis 1 (left stick Y)  -> joint 2  (up/down)
  right pad buttons:
    0 = up, 1 = right, 2 = down, 3 = left
  buttons 4,5,6,7 -> wrist (J5 / J6) with combos:
    btn 4        -> axis 6 turn LEFT
    btn 6        -> axis 6 turn RIGHT
    btn 4 + btn5 -> axis 5 UP
    btn 5 + btn6 -> axis 5 DOWN

Every direction press produces a fixed step (default 5 deg), repeated while
held (auto-repeat).
"""

import threading
import time

import pygame


class JoystickController:
    def __init__(self, step_deg=2.0, repeat_interval=0.12):
        # Smaller step + slower repeat for smooth, controlled jogging.
        self.step_deg = step_deg
        self.repeat_interval = repeat_interval
        self.joystick = None
        self.running = False
        self._thread = None
        self._lock = threading.Lock()
        # state
        self.connected = False
        self.error = None
        self.axes = [0.0] * 8
        self.buttons = [0] * 32
        self._active_jogs = set()
        self._debounce_candidate = None
        self._debounce_count = 0
        # continuous jog callbacks (hold-to-run)
        self.on_jog_start = None  # callback(joint_index, direction), per active jog
        self.on_jog_stop = None   # callback()

    # ── lifecycle ──────────────────────────────────────────────────────

    def start(self):
        if self.running:
            return
        self.running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

    def connect(self):
        """(Re)init pygame joystick. Returns True if connected."""
        try:
            pygame.joystick.quit()
            pygame.joystick.init()
            n = pygame.joystick.get_count()
            if n > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                with self._lock:
                    self.connected = True
                    self.error = None
                return True
            with self._lock:
                self.connected = False
                self.error = "no joystick found"
            return False
        except Exception as e:
            with self._lock:
                self.connected = False
                self.error = str(e)
            return False

    def disconnect(self):
        with self._lock:
            self.connected = False
        if self.joystick:
            try:
                self.joystick.quit()
            except Exception:
                pass
            self.joystick = None

    # ── polling ────────────────────────────────────────────────────────

    def _run(self):
        pygame.init()
        self.connect()
        while self.running:
            self._poll_once()
            time.sleep(0.02)  # 50 Hz

    def _poll_once(self):
        if not self.joystick:
            self.connect()
            return
        with self._lock:
            connected = self.connected
        if not connected:
            return
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                if event.axis < len(self.axes):
                    self.axes[event.axis] = event.value
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button < len(self.buttons):
                    self.buttons[event.button] = 1
            elif event.type == pygame.JOYBUTTONUP:
                if event.button < len(self.buttons):
                    self.buttons[event.button] = 0

        # Determine the set of currently-active continuous jogs
        # (multiple joints can be jogged at once, e.g. both sticks).
        active = self._current_jogs()
        # Debounce: only act when the state is stable for N consecutive
        # polls (~60ms), so stick jitter doesn't flap #JC/#H rapidly and
        # hammer the SPI bus (that corrupted the slave link).
        if active == self._debounce_candidate:
            self._debounce_count += 1
        else:
            self._debounce_candidate = active
            self._debounce_count = 1
        if self._debounce_count < 3:
            return
        if active != self._active_jogs:
            # Edge: the set of active joints changed.
            if self._active_jogs and self.on_jog_stop:
                try:
                    self.on_jog_stop()
                except Exception:
                    pass
            self._active_jogs = active
            if active and self.on_jog_start:
                # Start every active jog; the firmware #JC stacks per-joint
                # so all of them run until #H.
                for (joint, direction) in sorted(active):
                    try:
                        self.on_jog_start(joint, direction)
                    except Exception:
                        pass

    def _current_jogs(self):
        """Return a set of (joint_index, direction) for all held controls.

        Multiple controls may be held at once (left stick + right pad + wrist
        buttons), giving true multi-axis jogging.
        """
        active = set()
        # Left stick axes: axis0 -> J1, axis1 -> J2 (deadzone 0.5)
        ax0 = self.axes[0]
        ax1 = self.axes[1]
        if abs(ax0) > 0.5:
            active.add((0, -1 if ax0 > 0 else 1))   # J1 inverted
        if abs(ax1) > 0.5:
            active.add((1, 1 if ax1 > 0 else -1))
        active |= self._map_buttons_dirs(self.buttons)
        return active

    def _map_buttons_dirs(self, b):
        """Map held buttons to a set of (joint_index, direction) jogs.

        The right pad / wrist buttons behave like a digital joystick:
          btn 0 = up   -> J3 DOWN (inverted)
          btn 1 = right-> J4 right
          btn 2 = down -> J3 UP (inverted)
          btn 3 = left -> J4 left
          btn 4        -> J6 LEFT (inverted)
          btn 5        -> J6 RIGHT (inverted)
          btn 4 + 5    -> J5 DOWN (inverted)  (priority over single)
          btn 6        -> J6 LEFT (inverted)
          btn 7        -> J6 RIGHT (inverted)
          btn 6 + 7    -> J5 UP (inverted)    (priority over single)
        """
        active = set()
        b0, b1, b2, b3 = bool(b[0]), bool(b[1]), bool(b[2]), bool(b[3])
        # J3 vertical (pad up/down)
        if b0 and not b2:
            active.add((2, -1))          # J3 DOWN (inverted)
        if b2 and not b0:
            active.add((2, 1))           # J3 UP (inverted)
        # J4 horizontal (pad left/right)
        if b1 and not b3:
            active.add((3, 1))           # J4 right
        if b3 and not b1:
            active.add((3, -1))          # J4 left
        # Wrist (4,5,6,7)
        b4, b5, b6, b7 = bool(b[4]), bool(b[5]), bool(b[6]), bool(b[7])
        if b4 and b5:
            active.add((4, -1))          # J5 DOWN (inverted)
        if b6 and b7:
            active.add((4, 1))           # J5 UP (inverted)
        if (b4 or b6) and not (b4 and b5) and not (b6 and b7):
            active.add((5, -1))          # J6 LEFT (inverted)
        if (b5 or b7) and not (b4 and b5) and not (b6 and b7):
            active.add((5, 1))           # J6 RIGHT (inverted)
        return active

    # ── status ─────────────────────────────────────────────────────────

    def status(self):
        with self._lock:
            return {
                "connected": self.connected,
                "error": self.error,
                "name": (self.joystick.get_name()
                         if self.joystick and self.connected else None),
                "axes": list(self.axes),
                "buttons": list(self.buttons),
            }
