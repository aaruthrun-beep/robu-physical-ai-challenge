import json


class Scene:
    def __init__(self, name="Untitled"):
        self.name = name
        self.robots = {}
        self.objects = {}
        self.camera = {"position": [2, 2, 2], "target": [0, 0, 0]}
        self.grid = True

    def to_dict(self):
        return {
            "name": self.name,
            "camera": self.camera,
            "grid": self.grid,
            "robots": {k: {"joints": v} for k, v in self.robots.items()},
            "objects": self.objects,
        }

    def save(self, path):
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            data = json.load(f)
        scene = cls(data.get("name", "Imported"))
        scene.camera = data.get("camera", scene.camera)
        scene.grid = data.get("grid", True)
        scene.robots = data.get("robots", {})
        scene.objects = data.get("objects", {})
        return scene
