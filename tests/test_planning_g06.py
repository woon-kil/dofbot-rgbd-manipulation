import sys, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import numpy as np
from kinematics.urdf_fk import UrdfFK, DEFAULT_URDF
from planning.collision_3a import Hull, Box, clearances, finger_capsules, capsule_lowest_z
from planning.reach import Model, solve_ik, tip_state, tilt_deg, URDF_MODEL
from planning import observe


class HullTests(unittest.TestCase):
    def setUp(self):
        c = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], float)
        self.h = Hull(c)

    def test_inside_negative_outside_positive(self):
        d = self.h.distance_lb(np.array([[0, 0, 0], [3, 0, 0], [0, 0, 1.5]]))
        self.assertAlmostEqual(d[0], -1.0); self.assertAlmostEqual(d[1], 2.0); self.assertAlmostEqual(d[2], 0.5)

    def test_lower_bound_never_exceeds_true_distance(self):
        p = np.array([[2.0, 2.0, 0.0]])       # 진짜 거리 sqrt(2) 대 하한 1
        self.assertLessEqual(self.h.distance_lb(p)[0], np.sqrt(2) + 1e-12)
        self.assertAlmostEqual(self.h.distance_lb(p)[0], 1.0)


class BoxTests(unittest.TestCase):
    def setUp(self):
        self.b = Box([0, 0, 0], [0.015, 0.03, 0.025])

    def test_outside_face_edge_corner(self):
        d = self.b.signed_distance(np.array([[0.025, 0, 0], [0.025, 0.04, 0], [0.025, 0.04, 0.035]]))
        self.assertAlmostEqual(d[0], 0.010); self.assertAlmostEqual(d[1], np.hypot(0.010, 0.010)); self.assertAlmostEqual(d[2], np.sqrt(3) * 0.010)

    def test_inside_negative(self):
        d = self.b.signed_distance(np.array([[0.0, 0.0, 0.0], [0.014, 0.0, 0.0]]))
        self.assertAlmostEqual(d[0], -0.015); self.assertAlmostEqual(d[1], -0.001)


class CapsuleTests(unittest.TestCase):
    def test_lowest_z_vertical_down(self):
        caps = finger_capsules(0.086)
        R5 = np.array([[1.0, 0, 0], [0, 0, 1], [0, -1, 0]])   # 손가락 축(arm5 y)이 월드 -z를 향함이 아니라 +z... 아래 확인
        p5 = np.array([0.0, 0.0, 0.2])
        z = capsule_lowest_z(caps, p5, R5)
        # R5의 y 열(=월드로 간 arm5 y)은 [0,0,-1]이므로 끝(y=0.086)이 아래: 0.2 - 0.086 - r
        self.assertAlmostEqual(z, 0.2 - 0.086 - 0.005, places=9)


@unittest.skipUnless(DEFAULT_URDF.exists(), "URDF 없음")
class ReachTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fk = UrdfFK()

    def test_round_trip_known_pose(self):
        q = [89.0, 91.0, 13.0, 11.0]
        tip, a, _ = tip_state(self.fk, q, URDF_MODEL)
        r = float(np.hypot(tip[0], tip[1])); rhat = np.array([tip[0] / r, tip[1] / r, 0.0])
        t = tilt_deg(a, rhat)
        sol = solve_ik(self.fk, tip, t, URDF_MODEL)
        self.assertTrue(sol["ok"], sol)
        tip2, a2, _ = tip_state(self.fk, sol["q"], URDF_MODEL)
        np.testing.assert_allclose(tip2, tip, atol=2e-5)

    def test_vertical_unreachable_at_known_point(self):
        # 무차별 격자 탐색(3° 간격): (r 0.15, z 0.10)의 위치 해는 tilt 30~39°뿐이라 tilt ≤ 15°로는 도달 불가
        for tl in (-15.0, 0.0, 15.0):
            self.assertFalse(solve_ik(self.fk, np.array([0.0, 0.15, 0.10]), tl, URDF_MODEL)["ok"])

    def test_unreachable_far_target_rejected(self):
        sol = solve_ik(self.fk, np.array([0.0, 0.6, 0.05]), 0.0, URDF_MODEL)
        self.assertFalse(sol["ok"])

    def test_model_height_offset_shifts_solution(self):
        tgt = np.array([0.0, 0.15, 0.03])   # 무차별 격자 탐색에서 tilt 6~18°로 닿는 점
        a = solve_ik(self.fk, tgt, 12.0, Model(dz=0.0, tip_y=0.0))
        b = solve_ik(self.fk, tgt, 12.0, Model(dz=-0.0104, tip_y=0.0))
        self.assertTrue(a["ok"] and b["ok"])
        ta = tip_state(self.fk, a["q"], Model(dz=-0.0104, tip_y=0.0))[0]
        self.assertAlmostEqual(ta[2], 0.03 - 0.0104, places=3)   # a의 관절을 dz 세계에 넣으면 10.4 mm 낮다


class ObserveTests(unittest.TestCase):
    def test_projection_of_measured_tcp(self):
        # S4-13: 카메라 프레임 TCP (10.8, 52.8, 95.9) mm -> 픽셀 (369, 501), 영상 밖
        cp = np.zeros(3); Rc = np.eye(3)
        u, v, d = observe.project(np.array([0.0108, 0.0528, 0.0959]), cp, Rc)
        self.assertAlmostEqual(u, 369.0, delta=1.0); self.assertAlmostEqual(v, 501.0, delta=1.0)
        ok, _ = observe.visible(np.array([0.0108, 0.0528, 0.0959]), cp, Rc, dmin=0.0)
        self.assertFalse(ok)

    def test_depth_minimum_rejects_close_point(self):
        ok, _ = observe.visible(np.array([0.0, 0.0, 0.15]), np.zeros(3), np.eye(3), dmin=0.2)
        self.assertFalse(ok)
        ok, _ = observe.visible(np.array([0.0, 0.0, 0.25]), np.zeros(3), np.eye(3), dmin=0.2)
        self.assertTrue(ok)


if __name__ == '__main__':
    unittest.main()


@unittest.skipUnless(DEFAULT_URDF.exists(), "URDF 없음")
class PoseGridTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fk = UrdfFK()
        cls.model = Model(gains=(1, 0.913, 1, 1), dz=-0.0104, tip_y=-0.004)
        cls.grid = observe.PoseGrid(cls.fk, cls.model, step=4.0)

    def test_returned_pose_is_visible_and_distance_matches(self):
        target = np.array([0.0, 0.15, 0.019])
        res = self.grid.nearest_observations(target, dmin=0.0, k=3)
        self.assertTrue(res)
        for idx, dist, (u, v, d) in res:
            self.assertGreater(d, 0)
            self.assertTrue(40 <= u <= 600 and 40 <= v <= 440)
            self.assertAlmostEqual(dist, float(np.linalg.norm(self.grid.tip[idx] - target)), places=9)
        self.assertEqual(res, sorted(res, key=lambda r: r[1]))

    def test_larger_depth_minimum_never_gets_closer(self):
        target = np.array([0.0, 0.17, 0.0])
        d0 = self.grid.nearest_observations(target, 0.0)[0][1]
        d15 = self.grid.nearest_observations(target, 0.15)[0][1]
        self.assertGreaterEqual(d15, d0)
