"""들어올림 러너 판정·시계·계산 로직 시험(scripts/g01_lift_logic.py). unittest·pytest 모두 실행. Q3 시험은 cv2가 없으면 건너뛴다."""
import copy, sys, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import g01_lift_logic as L
import g01_lift_params as P

MV = lambda cd: cd + 0.3 + 3.25 + 0.1


def params(**kw):
    p = copy.deepcopy(P.PARAMS); p.update(kw); return p


class ClockTest(unittest.TestCase):
    def test_step_upper_bounds_match_plan_2_1(self):
        d = dict(L.step_upper_bounds())
        self.assertAlmostEqual(d["close145"], 4.4); self.assertAlmostEqual(d["stall150"], 4.1); self.assertAlmostEqual(d["lift1"], 3.65)
        self.assertEqual((d["photo_a"], d["photo_b"], d["photo_c"], d["hold"]), (3.0, 3.0, 3.0, 3.0))

    def run_case(self, cd, acts):
        return L.simulate(acts, params(move_countdown_s=cd))

    def test_plan_2_1_table_countdown_0(self):
        mt, mb = 2.95, 3.65
        s, e = self.run_case(0, [2.2, 2.8, 1.052, mt, mt, mt, 1.052, 3.0, 1.052]); self.assertIsNone(s); self.assertAlmostEqual(e, 20.01, 2)
        s, e = self.run_case(0, [2.2, 3.8, 1.052, mb, mb, mb, 1.052, 3.0, 1.052]); self.assertIsNone(s); self.assertAlmostEqual(e, 23.11, 2)
        s, e = self.run_case(0, [4.4, 3.8, 1.052, mb, mb, mb, 1.052, 3.0, 1.052]); self.assertEqual(s, "photo_c"); self.assertAlmostEqual(e, 24.25, 2)
        ubs = [u for _, u in L.step_upper_bounds()]; s, e = self.run_case(0, ubs); self.assertEqual(s, "hold"); self.assertAlmostEqual(e, 25.45, 2)

    def test_plan_2_1_table_countdown_1(self):
        mt, mb = 3.95, 4.65
        s, e = self.run_case(1, [2.2, 2.8, 1.052, mt, mt, mt, 1.052, 3.0, 1.052]); self.assertIsNone(s); self.assertAlmostEqual(e, 23.01, 2)
        s, e = self.run_case(1, [2.2, 3.8, 1.052, mb, mb, mb, 1.052, 3.0, 1.052]); self.assertEqual(s, "photo_c"); self.assertAlmostEqual(e, 25.05, 2)
        s, e = self.run_case(1, [4.4, 3.8, 1.052, mb, mb, mb, 1.052, 3.0, 1.052]); self.assertEqual(s, "hold")
        ubs = [u for _, u in L.step_upper_bounds(params(move_countdown_s=1))]; s, e = self.run_case(1, ubs); self.assertEqual(s, "photo_b")

    def test_once_skipped_everything_after_is_skipped(self):
        t = [0.0]; c = L.PressClock(lambda: t[0]); c.start(); t[0] = 25.0
        self.assertFalse(c.may_start("x", 3.0)); self.assertFalse(c.may_start("y", 0.0)); self.assertEqual(c.skipped_at, "x")

    def test_exactly_at_limit_is_allowed(self):
        t = [0.0]; c = L.PressClock(lambda: t[0]); c.start(); t[0] = 23.8
        self.assertTrue(c.may_start("x", 3.0))      # 23.8 + 3.0 = 26.8 ≤ 26.8


class LiftMathTest(unittest.TestCase):
    def test_second_j2_command_clip_and_half_up(self):
        self.assertEqual(L.second_j2_command(74, 69.0, 72.5)[0], 74 + 6)      # 올라간 3.5° → inc 6.0
        self.assertEqual(L.second_j2_command(74, 69.0, 74.0)[0], 74 + 5)      # 올라간 5° → inc 4.5 → half-up 5
        self.assertEqual(L.second_j2_command(74, 69.0, 66.0)[0], 74 + 6)      # 거꾸로 움직임 → 상한 6으로 클립
        self.assertEqual(L.second_j2_command(74, 69.0, 80.0)[0], 74 + 3)      # 많이 올라감 → 하한 3으로 클립

    def test_shortfall_abort_follows_tol_3(self):
        self.assertFalse(L.shortfall_abort(74, 71)); self.assertTrue(L.shortfall_abort(74, 70)); self.assertTrue(L.shortfall_abort(74, None))

    def test_same_start_and_pa_elapsed(self):
        self.assertTrue(L.same_start([1.5, 1.5, 1.5])); self.assertFalse(L.same_start([1.5, 1.5, 2.5])); self.assertFalse(L.same_start([1.5, None]))
        self.assertTrue(L.pa_elapsed_ok(1500, 1800)); self.assertFalse(L.pa_elapsed_ok(1501, 1800))


class JudgeTest(unittest.TestCase):
    A = {"found": True, "center": [673.0, 267.0], "side_px": 29.0, "yaw_deg": -179.0}

    def b(self, up=16.0, side=0.8, lat=0.0): return {"found": True, "center": [673.0 + lat, 267.0 - up], "side_px": 29.0 + side, "yaw_deg": -179.0}

    def test_e1_pass_and_each_criterion(self):
        self.assertEqual(L.e1_decide(self.A, self.b())["verdict"], "합격")
        self.assertFalse(L.e1_decide(self.A, self.b(up=7.9))["i"]); self.assertFalse(L.e1_decide(self.A, self.b(side=0.6999))["ii"])
        self.assertTrue(L.e1_decide(self.A, self.b(side=0.7))["ii"]); self.assertFalse(L.e1_decide(self.A, self.b(lat=4.1))["iii"])
        self.assertEqual(L.e1_decide(self.A, self.b(side=0.1))["verdict"], "불합격")             # 슬라이드: (i)는 통과, (ii)가 판별
        self.assertEqual(L.e1_decide(self.A, {"found": False})["verdict"], "판정 불가")

    def test_e3_p5_rules(self):
        self.assertEqual(L.e3_verdict(146, 146, hold_done=False, photo_c_done=False)["counts_as"], "실패")          # 유지 생략: E-2 원래 규칙
        self.assertEqual(L.e3_verdict(146, 147, True, False)["verdict"], "합격"); self.assertEqual(L.e3_verdict(146, 148, True, False)["verdict"], "불합격")
        b = {"found": True, "center": [10.0, 10.0], "side_px": 30.0}; c_ok = {"found": True, "center": [10.5, 10.5], "side_px": 30.5}; c_bad = {"found": True, "center": [12.0, 10.0], "side_px": 30.0}
        self.assertEqual(L.e3_verdict(146, 146, True, True, (b, c_ok))["verdict"], "합격"); self.assertEqual(L.e3_verdict(146, 146, True, True, (b, c_bad))["verdict"], "불합격")
        self.assertEqual(L.e3_verdict(146, 146, True, True, (b, {"found": False}))["counts_as"], "실패")

    def test_s4_gate_g2_is_fail_closed_until_conversion_is_set(self):
        ref = dict(self.A, yaw_deg=-179.0); a_small = dict(self.A, center=[673.5, 267.0]); a_big = dict(self.A, center=[675.0, 267.0])
        self.assertTrue(L.s4_gate(ref, a_small)["go"])                                  # 불감대 안: 환산 불필요
        with self.assertRaises(P.ParamsNotFinal): L.s4_gate(ref, a_big)                  # 불감대 밖: 환산·부호가 UNSET
        pp = params(g2_mm_per_px=0.8, g2_x_sign=1)
        self.assertFalse(L.s4_gate(ref, a_big, dx_depth_mm=-2.0, P=pp)["g2"])             # +1.2 mm, 예상 구간 음수 → 방향 불일치
        self.assertTrue(L.s4_gate(ref, dict(self.A, center=[671.5, 267.0]), dx_depth_mm=-2.0, P=pp)["g2"])
        self.assertTrue(L.s4_gate(ref, a_big, dx_depth_mm=1.0, P=pp)["g2"])               # 예상 구간이 0을 걸치면 방향 판정 생략
        self.assertFalse(L.s4_gate(ref, dict(self.A, center=[680.0, 267.0]), dx_depth_mm=1.0, P=pp)["g2"])   # 4 mm > 3.0
        self.assertFalse(L.s4_gate(ref, dict(a_small, side_px=30.0))["go"]); self.assertFalse(L.s4_gate(ref, dict(a_small, yaw_deg=-175.0))["go"])
        self.assertFalse(L.s4_gate(ref, {"found": False})["go"])


try:
    import cv2, numpy as np
except Exception:
    cv2 = None


@unittest.skipIf(cv2 is None, "cv2 없음")
class Q3Test(unittest.TestCase):
    @staticmethod
    def imgs():
        rng = np.random.RandomState(3); base = cv2.GaussianBlur((rng.rand(720, 1280) * 255).astype(np.float32), (0, 0), 2.0)
        a = cv2.cvtColor(base.astype(np.uint8), cv2.COLOR_GRAY2BGR); b = np.roll(a, 1, axis=1).copy(); return a, b

    def test_order_independence_and_repeatability(self):
        a, b = self.imgs(); new = (800, 0, 900, 100); old = L.OLD_BASE_BOX
        alone_new = L.base_shift_fresh(a, b, new); alone_old = L.base_shift_fresh(a, b, old)
        order1 = (L.base_shift_fresh(a, b, old), L.base_shift_fresh(a, b, new)); order2 = (L.base_shift_fresh(a, b, new), L.base_shift_fresh(a, b, old))
        self.assertEqual(order1[0], alone_old); self.assertEqual(order1[1], alone_new); self.assertEqual(order2[0], alone_new); self.assertEqual(order2[1], alone_old)
        self.assertEqual([L.base_shift_fresh(a, b, new) for _ in range(3)], [alone_new] * 3)          # 같은 호출 반복도 같은 값
        self.assertAlmostEqual(abs(alone_new[0]), 1.0, delta=0.2)

    def test_q3_record_fail_closed_then_records_both_boxes(self):
        a, b = self.imgs()
        with self.assertRaises(P.ParamsNotFinal): L.q3_record(a, b)                                # P9 확정 전(UNSET)
        r = L.q3_record(a, b, params(q3_box=(800, 0, 900, 100)))
        self.assertEqual(r["old_box"], [480, 0, 900, 100]); self.assertEqual(r["box"], [800, 0, 900, 100]); self.assertTrue(r["ok"])
        self.assertEqual(r["new"], L.base_shift_fresh(a, b, (800, 0, 900, 100))); self.assertEqual(r["old"], L.base_shift_fresh(a, b, L.OLD_BASE_BOX))

    def test_q3_fails_when_shift_exceeds_2px(self):
        a, _ = self.imgs(); b = np.roll(a, 3, axis=1).copy()
        self.assertFalse(L.q3_record(a, b, params(q3_box=(800, 0, 900, 100)))["ok"])


if __name__ == "__main__": unittest.main()
