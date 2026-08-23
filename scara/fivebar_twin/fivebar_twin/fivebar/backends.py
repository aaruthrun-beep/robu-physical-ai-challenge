"""
Command backends behind a single interface, plus the CAN message format.
SimBackend integrates motion in software. CanBackend talks to real RS01 motors
over any python-can adapter (Windows: slcan/pcan; Linux: socketcan).
"""
import struct, time, math


# ---------------- CAN message format (ADJUST to your motor drivers) ---------
# Command frame  (host -> motor):  8 bytes
#     float32 target_position_rad  (little-endian)
#     float32 target_velocity_rad_s
# Feedback frame (motor -> host):  8 bytes
#     float32 actual_position_rad
#     int16   status_bits
#     int16   temperature_c
def pack_command(pos_rad, vel_rad_s):
    return struct.pack("<ff", pos_rad, vel_rad_s)

def unpack_command(data):
    return struct.unpack("<ff", data[:8])

def pack_feedback(pos_rad, status, temp):
    return struct.pack("<fhh", pos_rad, status, temp)

def unpack_feedback(data):
    pos, status, temp = struct.unpack("<fhh", data[:8])
    return pos, status, temp


class SimBackend:
    """Pure-software twin. Integrates toward the last commanded angles."""
    def __init__(self, cfg, start=(math.pi/2, math.pi/2)):
        self.cfg = cfg
        self.cur = list(start)
        self.target = list(start)
        self.traj = []
        self.connected = True
        self.mode = "SIM"

    def send_angles(self, t1, t2):
        self.target = [t1, t2]
        self.traj = []

    def send_trajectory(self, traj):
        if traj:
            self.traj = list(traj)
            self.target = list(traj[-1])
        else:
            self.traj = []

    def step(self, dt):
        if self.traj:
            step_limit = self.cfg.max_vel * dt
            while self.traj:
                wp = self.traj[0]
                d1 = wp[0] - self.cur[0]
                d2 = wp[1] - self.cur[1]
                dist = math.hypot(d1, d2)
                if dist <= step_limit:
                    self.cur = list(wp)
                    self.traj.pop(0)
                    step_limit -= dist
                else:
                    self.cur[0] += (d1 / dist) * step_limit
                    self.cur[1] += (d2 / dist) * step_limit
                    break
        else:
            for i in range(2):
                err = self.target[i] - self.cur[i]
                step = self.cfg.max_vel * dt
                self.cur[i] += max(-step, min(step, err))
        return tuple(self.cur)

    def read_angles(self):
        return tuple(self.cur)

    def close(self):
        self.connected = False


class CanBackend:
    """Real hardware driver using RobStride Dynamics SDK for RobStride RS01 motors."""
    def __init__(self, cfg, is_monitor: bool = False):
        self.cfg = cfg
        self.is_monitor = is_monitor
        self.mode = "MONITOR" if is_monitor else "LIVE"
        self.connected = False
        self._bus = None
        self._is_robstride = False
        self._last = (math.pi/2, math.pi/2)
        self._stream_thread = None
        self._stop_stream = False

    def connect(self):
        import sys, os, threading
        # Ensure Python_Sample-main is in sys.path
        sdk_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "Python_Sample-main"))
        if sdk_path not in sys.path:
            sys.path.insert(0, sdk_path)

        try:
            from robstride_dynamics import RobstrideBus, Motor, ParameterType
            self._ParameterType = ParameterType
            
            motors = {
                "left": Motor(id=self.cfg.can_id_left, model=self.cfg.motor_model),
                "right": Motor(id=self.cfg.can_id_right, model=self.cfg.motor_model),
            }
            
            self._bus = RobstrideBus(
                channel=self.cfg.can_channel,
                motors=motors,
                bitrate=self.cfg.can_bitrate,
                interface=self.cfg.can_interface
            )
            self._bus.connect()
            self._is_robstride = True
            
            # Switch modes while disabled (as required by RobStride manual)
            for mname in ("left", "right"):
                try:
                    self._bus.disable(mname)
                except Exception:
                    pass
                if not self.is_monitor:
                    run_mode = getattr(self.cfg, "run_mode", 1) # 1 = PP mode default
                    try:
                        self._bus.set_run_mode(mname, run_mode)
                    except Exception as me:
                        print(f"Warning setting run mode for {mname}: {me}")
                    try:
                        self._bus.enable(mname)
                    except Exception as ee:
                        print(f"Warning enabling motor {mname}: {ee}")
                
            self.connected = True
            st = "MONITOR ONLY (Unpowered)" if self.is_monitor else "ENABLED"
            print(f"RobStride CAN Bus connected on {self.cfg.can_channel} ({self.cfg.can_interface}). Status: {st}.")
            
        except Exception as e:
            print(f"RobStride SDK connect failed/fallback ({e}). Falling back to raw python-can.")
            import can
            self._bus = can.interface.Bus(
                interface=self.cfg.can_interface,
                channel=self.cfg.can_channel,
                bitrate=self.cfg.can_bitrate)
            self._is_robstride = False
            self.connected = True

    def send_angles(self, t1, t2):
        if not self.connected or self._bus is None or self.is_monitor:
            return

        self._stop_stream = True

        if self._is_robstride:
            run_mode = getattr(self.cfg, "run_mode", 1)
            if run_mode == 1:  # PP Mode (Profile Position)
                try:
                    self._bus.move_to_position_pp("left", position=t1, velocity_max=self.cfg.max_vel, acceleration=self.cfg.max_acc)
                    self._bus.move_to_position_pp("right", position=t2, velocity_max=self.cfg.max_vel, acceleration=self.cfg.max_acc)
                except Exception as e:
                    print(f"Error sending PP move: {e}")
            elif run_mode == 5:  # CSP Mode (Cyclic Synchronous Position)
                try:
                    self._bus.move_to_position_csp("left", position=t1, velocity_limit=self.cfg.max_vel)
                    self._bus.move_to_position_csp("right", position=t2, velocity_limit=self.cfg.max_vel)
                except Exception as e:
                    print(f"Error sending CSP move: {e}")
        else:
            import can
            for cid, ang in ((self.cfg.can_id_left, t1), (self.cfg.can_id_right, t2)):
                msg = can.Message(arbitration_id=cid,
                                  data=pack_command(ang, self.cfg.max_vel),
                                  is_extended_id=False)
                self._bus.send(msg)

    def send_trajectory(self, traj):
        if not self.connected or self._bus is None or self.is_monitor or not traj:
            return

        run_mode = getattr(self.cfg, "run_mode", 1)
        if run_mode == 1 or not self._is_robstride:
            # PP mode: send final target position
            t1, t2 = traj[-1]
            self.send_angles(t1, t2)
        else:
            # CSP mode: stream waypoints along trajectory in background thread
            import threading
            self._stop_stream = True
            time.sleep(0.01)
            self._stop_stream = False
            
            def stream_loop():
                for (w1, w2) in traj:
                    if self._stop_stream:
                        break
                    try:
                        self._bus.move_to_position_csp("left", position=w1, velocity_limit=self.cfg.max_vel)
                        self._bus.move_to_position_csp("right", position=w2, velocity_limit=self.cfg.max_vel)
                    except Exception:
                        pass
                    time.sleep(0.02)

            self._stream_thread = threading.Thread(target=stream_loop, daemon=True)
            self._stream_thread.start()

    def read_angles(self):
        """Poll feedback; returns last known actual mechanical angles."""
        if not self.connected or self._bus is None:
            return self._last

        if self._is_robstride:
            try:
                t1 = self._bus.read("left", self._ParameterType.MECHANICAL_POSITION)
                t2 = self._bus.read("right", self._ParameterType.MECHANICAL_POSITION)
                if t1 is not None and t2 is not None:
                    self._last = (t1, t2)
            except Exception:
                pass
            return self._last
        else:
            got = {}
            msg = self._bus.recv(timeout=0.0)
            while msg is not None:
                if msg.arbitration_id == self.cfg.can_id_left:
                    got[0] = unpack_feedback(msg.data)[0]
                elif msg.arbitration_id == self.cfg.can_id_right:
                    got[1] = unpack_feedback(msg.data)[0]
                msg = self._bus.recv(timeout=0.0)
            self._last = (got.get(0, self._last[0]), got.get(1, self._last[1]))
            return self._last

    def close(self):
        self._stop_stream = True
        if self.connected and self._bus is not None:
            if self._is_robstride:
                try:
                    self._bus.disable("left")
                    self._bus.disable("right")
                except Exception:
                    pass
                try:
                    self._bus.disconnect(disable_torque=True)
                except Exception:
                    pass
            else:
                self._bus.shutdown()
        self.connected = False
