from fivebar.config import default_config
from fivebar import workspace as ws


def test_workspace_has_safe_and_some_structure():
    cfg = default_config()
    xs, ys, states = ws.grid_map(cfg, res=8.0)
    flat = [s for row in states for s in row]
    assert ws.SAFE in flat, "there must be a safe region"
    assert ws.UNREACHABLE in flat, "corners must be unreachable"


def test_center_front_is_safe():
    cfg = default_config()
    state, assembly, rep = ws.classify(cfg, 0, 220)
    assert state == ws.SAFE
