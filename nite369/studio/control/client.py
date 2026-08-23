import json
import socket
import logging

log = logging.getLogger("astra_studio.client")


class RobotClient:
    """Client library for connecting to the Astra CommandServer."""

    def __init__(self, host="127.0.0.1", port=8765, timeout=10):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None
        self.buffer = b""

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect((self.host, self.port))
        return self

    def disconnect(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception as e:
                log.debug("Client close error: %s", e)
            self.sock = None

    def send_command(self, action, params=None):
        if not self.sock:
            raise ConnectionError("Not connected")
        cmd = {"action": action, "params": params or {}}
        self.sock.sendall((json.dumps(cmd) + "\n").encode())
        return self._recv_response()

    def _recv_response(self):
        while b"\n" not in self.buffer:
            data = self.sock.recv(4096)
            if not data:
                raise ConnectionError("Server disconnected")
            self.buffer += data
        line, self.buffer = self.buffer.split(b"\n", 1)
        return json.loads(line.decode())

    def move_joints(self, positions):
        return self.send_command("move_joints", {"positions": positions})

    def get_joints(self):
        return self.send_command("get_joints")

    def get_pose(self):
        return self.send_command("get_pose")

    def add_box(self, name, size, position, color=None):
        params = {"type": "box", "name": name, "size": size, "position": position}
        if color:
            params["color"] = color
        return self.send_command("add_object", params)

    def add_cylinder(self, name, radius, height, position, color=None):
        params = {"type": "cylinder", "name": name, "size": [radius, height], "position": position}
        if color:
            params["color"] = color
        return self.send_command("add_object", params)

    def add_sphere(self, name, radius, position, color=None):
        params = {"type": "sphere", "name": name, "size": [radius], "position": position}
        if color:
            params["color"] = color
        return self.send_command("add_object", params)

    def remove_object(self, name):
        return self.send_command("remove_object", {"name": name})

    def list_objects(self):
        return self.send_command("list_objects")

    def reset(self):
        return self.send_command("reset")

    def __enter__(self):
        return self.connect()

    def __exit__(self, *args):
        self.disconnect()
