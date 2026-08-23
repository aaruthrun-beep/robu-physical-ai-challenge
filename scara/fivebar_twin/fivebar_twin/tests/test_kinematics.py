import unittest
import math
from fivebar.config import RobotConfig
from fivebar import kinematics as kin


class TestKinematics(unittest.TestCase):
    def test_ik_fk_roundtrip_both_branches(self):
        cfg = RobotConfig()
        for assembly in (+1, -1):
            for (x, y) in [(0, 180), (60, 160), (-60, 160), (0, 250), (40, 120)]:
                sol = kin.ik(cfg, x, y, assembly)
                self.assertIsNotNone(sol, f"{(x,y)} should be reachable")
                fx, fy = kin.fk(cfg, sol[0], sol[1], assembly)
                self.assertAlmostEqual(fx, x, places=5)
                self.assertAlmostEqual(fy, y, places=5)

    def test_branches_differ(self):
        cfg = RobotConfig()
        up = kin.ik(cfg, 40, 200, +1)
        dn = kin.ik(cfg, 40, 200, -1)
        self.assertIsNotNone(up)
        self.assertIsNotNone(dn)
        self.assertTrue(abs(up[0] - dn[0]) > 1e-3 or abs(up[1] - dn[1]) > 1e-3)

    def test_unreachable_returns_none(self):
        cfg = RobotConfig()
        self.assertIsNone(kin.ik(cfg, 0, 5000, +1))


if __name__ == "__main__":
    unittest.main()

