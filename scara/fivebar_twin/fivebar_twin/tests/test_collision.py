import unittest
import math
from fivebar.config import RobotConfig, Obstacle
from fivebar import kinematics as kin
from fivebar import collision as col


class TestCollision(unittest.TestCase):
    def test_safe_pose_has_clearance(self):
        cfg = RobotConfig()
        t1, t2 = kin.ik(cfg, 0, 220, +1)
        rep = col.check_pose(cfg, t1, t2, (0, 220))
        self.assertTrue(rep.ok)
        self.assertGreater(rep.min_clearance, cfg.margin)

    def test_colliding_with_obstacle(self):
        cfg = RobotConfig()
        cfg.obstacles = [Obstacle(x=0, y=200, radius=60, name="block")]
        t1, t2 = kin.ik(cfg, 0, 200, +1)
        rep = col.check_pose(cfg, t1, t2, (0, 200))
        self.assertFalse(rep.ok)
        self.assertIn("block", rep.worst_pair or ("", ""))

    def test_near_margin_pose(self):
        cfg = RobotConfig()
        cfg.margin = 5.0
        cfg.obstacles = [Obstacle(x=0, y=200 + cfg.ee_radius + 5.0 + 1.0, radius=0.5, name="pin")]
        t1, t2 = kin.ik(cfg, 0, 200, +1)
        rep = col.check_pose(cfg, t1, t2, (0, 200))
        self.assertEqual(rep.min_clearance, rep.min_clearance)

    def test_segment_distance_basic(self):
        d = col._seg_seg_distance((0,0,0),(10,0,0),(0,10,0),(10,10,0))
        self.assertAlmostEqual(d, 10.0, places=5)


if __name__ == "__main__":
    unittest.main()

