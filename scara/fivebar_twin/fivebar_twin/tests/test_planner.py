import unittest
import math
from fivebar.config import RobotConfig, Obstacle
from fivebar import kinematics as kin
from fivebar import planner


class TestPlanner(unittest.TestCase):
    def test_direct_plan(self):
        cfg = RobotConfig()
        start = kin.ik(cfg, 0, 220, +1)
        target = kin.ik(cfg, 50, 200, +1)
        traj, fail = planner.plan(cfg, start, target, +1)
        self.assertIsNotNone(traj)
        self.assertIsNone(fail)
        self.assertGreater(len(traj), 2)

    def test_detour_plan(self):
        cfg = RobotConfig()
        start = kin.ik(cfg, -80, 250, +1)
        target = kin.ik(cfg, 80, 250, +1)
        # Put an obstacle between start and target
        cfg.obstacles.append(Obstacle(x=0.0, y=230.0, radius=10.0, name="center_block"))
        
        traj, fail = planner.plan(cfg, start, target, +1)
        self.assertIsNotNone(traj)
        self.assertIsNone(fail)
        self.assertGreater(len(traj), 5)



if __name__ == "__main__":
    unittest.main()
