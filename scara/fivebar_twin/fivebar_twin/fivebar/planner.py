import math
import heapq
from . import kinematics as kin
from . import collision as col


def plan(cfg, start_angles, target_angles, assembly):
    """
    Build a validated joint-space trajectory from start to target.
    Returns (trajectory, report). trajectory is a list of (t1, t2); if any sample
    collides and no detour path exists, trajectory is None and report explains where it failed.
    """
    n = max(2, cfg.path_check_steps)
    s1, s2 = start_angles
    g1, g2 = target_angles

    # 1. Try direct joint-space interpolation first
    traj = []
    direct_ok = True
    first_fail_rep = None
    first_fail_frac = 0.0

    for i in range(n + 1):
        f = i / n
        t1 = s1 + (g1 - s1) * f
        t2 = s2 + (g2 - s2) * f
        rep = col.check_angles(cfg, t1, t2, assembly)
        if not rep.ok:
            if direct_ok:
                direct_ok = False
                first_fail_rep = rep
                first_fail_frac = f
        traj.append((t1, t2))

    if direct_ok:
        return traj, None

    # 2. Direct path collided; attempt A* detour in joint space (theta1, theta2)
    start_rep = col.check_angles(cfg, s1, s2, assembly)
    target_rep = col.check_angles(cfg, g1, g2, assembly)
    if not start_rep.ok:
        return None, (0.0, start_rep)
    if not target_rep.ok:
        return None, (1.0, target_rep)

    step_deg = 2.0
    step_rad = math.radians(step_deg)

    def to_grid(angle):
        return int(round(angle / step_rad))

    def to_angle(grid_idx):
        return grid_idx * step_rad

    start_node = (to_grid(s1), to_grid(s2))
    goal_node = (to_grid(g1), to_grid(g2))

    def dist_h(node1, node2):
        d1 = (node1[0] - node2[0]) * step_rad
        d2 = (node1[1] - node2[1]) * step_rad
        return math.hypot(d1, d2)

    open_set = []
    heapq.heappush(open_set, (dist_h(start_node, goal_node), 0.0, start_node))
    came_from = {}
    g_score = {start_node: 0.0}

    neighbors = [
        (-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1)
    ]

    max_evals = 3500
    evals = 0
    found_node = None

    while open_set and evals < max_evals:
        _, current_g, current = heapq.heappop(open_set)
        evals += 1

        if current == goal_node or dist_h(current, goal_node) <= step_rad * 1.5:
            found_node = current
            break

        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)
            t1_cand = to_angle(neighbor[0])
            t2_cand = to_angle(neighbor[1])

            if cfg.theta_min is not None and (t1_cand < cfg.theta_min or t2_cand < cfg.theta_min):
                continue
            if cfg.theta_max is not None and (t1_cand > cfg.theta_max or t2_cand > cfg.theta_max):
                continue

            rep = col.check_angles(cfg, t1_cand, t2_cand, assembly)
            if not rep.ok:
                continue

            step_cost = math.hypot(dx, dy) * step_rad
            tentative_g = current_g + step_cost

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                f_score = tentative_g + dist_h(neighbor, goal_node)
                heapq.heappush(open_set, (f_score, tentative_g, neighbor))

    if found_node is None:
        return None, (first_fail_frac, first_fail_rep)

    grid_path = []
    curr = found_node
    while curr in came_from:
        grid_path.append(curr)
        curr = came_from[curr]
    grid_path.append(start_node)
    grid_path.reverse()

    raw_traj = [(s1, s2)]
    for n_idx in range(1, len(grid_path) - 1):
        raw_traj.append((to_angle(grid_path[n_idx][0]), to_angle(grid_path[n_idx][1])))
    raw_traj.append((g1, g2))

    dense_traj = []
    for k in range(len(raw_traj) - 1):
        pA = raw_traj[k]
        pB = raw_traj[k+1]
        segment_dist = math.hypot(pB[0] - pA[0], pB[1] - pA[1])
        sub_steps = max(1, int(math.ceil(segment_dist / (step_rad / 2))))
        for s_idx in range(sub_steps):
            frac = s_idx / sub_steps
            t1_sub = pA[0] + (pB[0] - pA[0]) * frac
            t2_sub = pA[1] + (pB[1] - pA[1]) * frac
            dense_traj.append((t1_sub, t2_sub))
    dense_traj.append((g1, g2))

    return dense_traj, None

