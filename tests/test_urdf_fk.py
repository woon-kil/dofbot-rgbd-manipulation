import sys, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import numpy as np
from kinematics.urdf_fk import UrdfFK, DEFAULT_URDF

# 기준값: scripts/g01_pose_generator.py의 fk()가 2026-11-14에 낸 값(파일럿 12자세 readback J1~J4, J5=90). 단위 m.
# (q, DaBai 위치, TCP 위치, Arm5 위치)
REF = [
    ([89.0, 91.0, 13.0, 11.0], [0.001886407, 0.15391116, 0.184320488], [0.002052063, 0.151941797, 0.073894492], [0.001388249, 0.113911946, 0.155462193]),
    ([85.0, 93.0, 13.0, 11.0], [0.012421901, 0.15116195, 0.189654582], [0.012785763, 0.153026168, 0.079227235], [0.009224654, 0.112322505, 0.159417823]),
    ([80.0, 99.0, 10.0, 11.0], [0.024156107, 0.141603109, 0.19693415], [0.025684181, 0.149117492, 0.086756995], [0.017870013, 0.104801143, 0.164699281]),
    ([75.0, 99.0, 10.0, 11.0], [0.03640571, 0.138958923, 0.19693415], [0.038582891, 0.146311531, 0.086756995], [0.026936034, 0.102844869, 0.164699281]),
    ([89.0, 104.0, 10.0, 11.0], [0.001584339, 0.136605677, 0.209170524], [0.00208517, 0.153838532, 0.100080724], [0.001184244, 0.102224515, 0.173804408]),
    ([99.0, 95.0, 10.0, 11.0], [-0.024018925, 0.14653556, 0.186732559], [-0.023815337, 0.14652865, 0.076289066], [-0.017643465, 0.107560984, 0.157180531]),
    ([104.0, 90.0, 14.0, 11.0], [-0.038362812, 0.150557984, 0.184333107], [-0.037693093, 0.1486986, 0.07390711], [-0.028491438, 0.111792778, 0.155474811]),
    ([85.0, 91.0, 10.0, 11.0], [0.012503688, 0.152096784, 0.176218033], [0.012028104, 0.144366088, 0.066046273], [0.009089684, 0.110779789, 0.14949282]),
    ([99.0, 80.0, 19.0, 11.0], [-0.026966387, 0.165145103, 0.169613315], [-0.024956877, 0.153736045, 0.059778887], [-0.02014154, 0.123333207, 0.144351222]),
    ([95.0, 80.0, 19.0, 15.0], [-0.015654071, 0.169747885, 0.17468951], [-0.015115529, 0.165887058, 0.064314646], [-0.01192508, 0.129420058, 0.146533738]),
    ([85.0, 80.0, 19.0, 15.0], [0.01406016, 0.169887334, 0.17468951], [0.013920095, 0.165991645, 0.064314646], [0.010729646, 0.129524645, 0.146533738]),
    ([80.0, 80.0, 19.0, 11.0], [0.028268767, 0.164927159, 0.169613315], [0.02645437, 0.15348545, 0.059778887], [0.021109164, 0.123171282, 0.144351222]),
]


class UrdfFkRegression(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fk = UrdfFK(DEFAULT_URDF)

    def test_matches_reference_fk(self):
        for q, dabai, tcp, arm5 in REF:
            f = self.fk.fk(np.array([q + [90.0]]))
            np.testing.assert_allclose(f["dabai"][0][0], dabai, atol=1e-6)
            np.testing.assert_allclose(f["tcp"][0][0], tcp, atol=1e-6)
            np.testing.assert_allclose(f["arm5"][0][0], arm5, atol=1e-6)

    def test_batch_equals_single(self):
        qs = np.array([r[0] + [90.0] for r in REF])
        batch = self.fk.fk(qs)["tcp"][0]
        for i, r in enumerate(REF):
            np.testing.assert_allclose(batch[i], self.fk.fk(np.array([r[0] + [90.0]]))["tcp"][0][0], atol=1e-12)

    def test_upright_tcp_height(self):
        # 도달 포락선 보고서: 모든 관절 0 rad(SDK 90°) TCP 높이 456.3 mm
        f = self.fk.fk(np.array([[90.0, 90.0, 90.0, 90.0, 90.0]]))
        self.assertAlmostEqual(f["tcp"][0][0][2] * 1000, 456.3, delta=0.1)

    def test_default_correction_is_nominal(self):
        q = np.array([REF[0][0] + [90.0]])
        a = self.fk.fk(q)["tcp"][0]
        b = self.fk.fk(q, gains=[1, 1, 1, 1], zeros=[0, 0, 0, 0], tool_offset=[0, 0, 0])["tcp"][0]
        np.testing.assert_allclose(a, b, atol=1e-12)

    def test_tool_offset_moves_along_arm5_axes(self):
        q = np.array([REF[0][0] + [90.0]])
        f0 = self.fk.fk(q); f1 = self.fk.fk(q, tool_offset=[0, -0.004, 0])
        d = f1["tcp"][0][0] - f0["tcp"][0][0]
        expect = f0["tcp"][1][0] @ np.array([0, -0.004, 0])
        np.testing.assert_allclose(d, expect, atol=1e-12)

    def test_j2_gain_changes_only_downstream(self):
        q = np.array([[89.0, 104.0, 10.0, 11.0, 90.0]])
        f0 = self.fk.fk(q); f1 = self.fk.fk(q, gains=[1, 0.913, 1, 1])
        np.testing.assert_allclose(f0["Arm2_Joint"][0], f1["Arm2_Joint"][0], atol=1e-12)
        self.assertGreater(np.linalg.norm(f0["tcp"][0] - f1["tcp"][0]), 1e-3)

    def test_rejects_bad_shape(self):
        with self.assertRaises(ValueError):
            self.fk.fk(np.zeros((1, 4)))


if __name__ == '__main__':
    unittest.main()
