import json
import socket
import threading
import traceback
import logging
import numpy as np

log = logging.getLogger("astra_studio.server")


class CommandServer:
    """TCP command server for remote robot control (port 8765)."""

    def __init__(self, host="0.0.0.0", port=8765):
        self.host = host
        self.port = port
        self.server = None
        self.clients = []
        self.running = False
        self._handlers = {}
        self.simulation = None

    def set_simulation(self, sim_engine):
        self.simulation = sim_engine

    def register_handler(self, command, handler):
        self._handlers[command] = handler

    def start(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.host, self.port))
        self.server.listen(5)
        self.running = True
        thread = threading.Thread(target=self._accept_loop, daemon=True)
        thread.start()
        log.info("Command server listening on port %s", self.port)
        return self

    def stop(self):
        self.running = False
        if self.server:
            try:
                self.server.close()
            except Exception as e:
                log.debug("Server close error: %s", e)
        for c in self.clients:
            try:
                c.close()
            except Exception as e:
                log.debug("Client close error: %s", e)

    def _accept_loop(self):
        while self.running:
            try:
                client, addr = self.server.accept()
                self.clients.append(client)
                t = threading.Thread(target=self._handle_client, args=(client, addr), daemon=True)
                t.start()
            except Exception as e:
                if self.running:
                    log.warning("Accept loop error: %s", e)
                break

    def _handle_client(self, client, addr):
        buffer = b""
        while self.running:
            try:
                data = client.recv(4096)
                if not data:
                    break
                buffer += data
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    if line.strip():
                        self._process_command(client, line.decode().strip())
            except Exception as e:
                if self.running:
                    log.warning("Client handler error: %s", e)
                break
        try:
            client.close()
        except Exception as e:
            log.debug("Client close error: %s", e)

    def _process_command(self, client, cmd_str):
        try:
            cmd = json.loads(cmd_str)
            action = cmd.get("action", "")
            params = cmd.get("params", {})
            handler = self._handlers.get(action)
            if handler:
                result = handler(params)
            else:
                result = {"error": f"Unknown command: {action}"}
            response = json.dumps(result) + "\n"
            client.sendall(response.encode())
        except json.JSONDecodeError:
            client.sendall(json.dumps({"error": "Invalid JSON"}).encode() + b"\n")
        except Exception as e:
            log.warning("Command failed: %s", e, exc_info=True)
            client.sendall(json.dumps({"error": f"Internal error: {e}"}).encode() + b"\n")


def create_default_handlers(sim_engine=None):
    """Create default command handlers for the server."""
    handlers = {}

    def move_joints(params):
        if sim_engine and "astra" in sim_engine.robots:
            positions = params.get("positions", [])
            sim_engine.set_joint_positions("astra", positions)
            return {"status": "ok", "positions": positions}
        return {"error": "No robot loaded"}

    def get_joints(params):
        if sim_engine and "astra" in sim_engine.robots:
            pos = sim_engine.get_joint_positions("astra")
            return {"status": "ok", "positions": pos}
        return {"error": "No robot loaded"}

    def get_pose(params):
        if sim_engine and "astra" in sim_engine.robots:
            pose = sim_engine.get_endeffector_pose("astra")
            return {"status": "ok", "pose": pose}
        return {"error": "No robot loaded"}

    def add_object(params):
        if not sim_engine:
            return {"error": "No simulation"}
        obj_type = params.get("type", "box")
        name = params.get("name", f"obj_{len(sim_engine.bodies)}")
        pos = params.get("position", [0, 0, 0])
        size = params.get("size", [0.05, 0.05, 0.05])
        color = params.get("color", [0.5, 0.5, 0.5, 1.0])
        if obj_type == "box":
            sim_engine.add_box(name, size, pos, color)
        elif obj_type == "cylinder":
            sim_engine.add_cylinder(name, size[0], size[1] if len(size) > 1 else 0.1, pos, color)
        elif obj_type == "sphere":
            sim_engine.add_sphere(name, size[0], pos, color)
        return {"status": "ok", "name": name}

    def remove_object(params):
        name = params.get("name", "")
        sim_engine.remove_body(name)
        return {"status": "ok"}

    def list_objects(params):
        return {"status": "ok", "objects": list(sim_engine.bodies.keys())}

    def reset_simulation(params):
        sim_engine.reset()
        return {"status": "ok"}

    handlers["move_joints"] = move_joints
    handlers["get_joints"] = get_joints
    handlers["get_pose"] = get_pose
    handlers["add_object"] = add_object
    handlers["remove_object"] = remove_object
    handlers["list_objects"] = list_objects
    handlers["reset"] = reset_simulation
    return handlers
