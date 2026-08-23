"""Generate a PNG of the reachable/forbidden workspace for the current config."""
from fivebar.config import default_config
from fivebar import workspace as ws

if __name__ == "__main__":
    cfg = default_config()
    out = ws.render_png(cfg, "workspace_map.png", res=2.5)
    print("wrote", out)
