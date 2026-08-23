"""
Workspace map: classify every (x,y) as SAFE, FORBIDDEN (IK solvable but every
assembly collides) or UNREACHABLE (no IK). Also renders a PNG for inspection.
"""
import math
from . import kinematics as kin
from . import collision as col

SAFE, FORBIDDEN, UNREACHABLE = 2, 1, 0


def classify(cfg, x, y):
    """Return (state, assembly, report). Prefers a collision-free assembly."""
    reachable_any = False
    best = None
    for assembly in (+1, -1):
        sol = kin.ik(cfg, x, y, assembly)
        if sol is None:
            continue
        reachable_any = True
        rep = col.check_pose(cfg, sol[0], sol[1], (x, y))
        if rep.ok:
            return SAFE, assembly, rep
        if best is None:
            best = (assembly, rep)
    if not reachable_any:
        return UNREACHABLE, None, None
    return FORBIDDEN, best[0], best[1]


def bounds(cfg, pad=20):
    R = max(cfg.L1a + cfg.L2a, cfg.L1b + cfg.L2b) + pad
    return -(cfg.d / 2 + R), (cfg.d / 2 + R), -R, R


def grid_map(cfg, res=3.0):
    """Compute a classification grid. Returns (xs, ys, states[y][x])."""
    x0, x1, y0, y1 = bounds(cfg)
    xs = [x0 + i * res for i in range(int((x1 - x0) / res) + 1)]
    ys = [y0 + i * res for i in range(int((y1 - y0) / res) + 1)]
    states = [[classify(cfg, x, y)[0] for x in xs] for y in ys]
    return xs, ys, states


def render_png(cfg, path, res=3.0):
    """Render the workspace map to a PNG (safe=green, forbidden=red, else dark)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    xs, ys, states = grid_map(cfg, res)
    x0, x1, y0, y1 = xs[0], xs[-1], ys[0], ys[-1]
    cmap = ListedColormap(["#0d1216", "#ff5a5f", "#35e0c8"])
    fig, ax = plt.subplots(figsize=(7, 7), facecolor="#0d1216")
    ax.imshow(states, origin="lower", extent=[x0, x1, y0, y1],
              cmap=cmap, vmin=0, vmax=2, interpolation="nearest")
    (A1x, A1y), (A2x, A2y) = cfg.bases()
    ax.plot([A1x, A2x], [A1y, A2y], "-", color="#8fa4ad", lw=2)
    ax.scatter([A1x, A2x], [A1y, A2y], c="#35e0c8", s=60, zorder=5, edgecolors="k")
    for ob in cfg.obstacles:
        ax.add_patch(plt.Circle((ob.x, ob.y), ob.radius, color="#f5b34a", alpha=0.6))
    ax.set_facecolor("#0d1216")
    ax.set_title("5-bar reachable workspace  (green=safe, red=forbidden)",
                 color="#e6f0f2", fontsize=11)
    ax.tick_params(colors="#7f929b")
    for s in ax.spines.values():
        s.set_color("#26333c")
    ax.set_xlabel("X (mm)", color="#7f929b"); ax.set_ylabel("Y (mm)", color="#7f929b")
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(path, dpi=110, facecolor="#0d1216")
    plt.close(fig)
    return path
