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
    def test_second_j2_command_adds_to_actual_angle_and_clips_to_max_target(self):
        pp = params(lift_j2_max_target=77)
        self.assertEqual(L.second_j2_command(74, 68.32, 72.5, pp)[0], 77)         # 실제 72.5(올라간 4.18°): 공식 → 78 → 최대 목표 77 (이전 결함: 첫 명령값에 더하면 총 +10°·tilt 31.1°)
        self.assertEqual(L.second_j2_command(74, 68.32, 73.0, pp)[0], 77)         # 부족 없음 근처: 공식 73+4.5→78 → 77
        self.assertIsNone(L.second_j2_command(74, 68.32, 76.5, pp)[0])            # 77 − 76.5 < 1° → 두 번째 이동 없음
        self.assertEqual(L.second_j2_command(74, 68.32, 70.0, pp)[0], 76)         # 공식 70+6.0=76 < 77 → 76
        self.assertEqual(L.second_j2_command(74, 68.32, 72.5, params(lift_j2_max_target=76))[0], 76)
        self.assertEqual(L.second_j2_command(74, 68.32, 72.5, params(lift_j2_max_target=99))[0], 78)   # 최대 목표가 넉넉하면 공식만(시작 68.32: 올라간 4.18° → inc 5.32 → 72.5+5.32=77.8 → half-up 78)
        with self.assertRaises(ValueError): L.second_j2_command(78, 68.32, 72.5, pp)        # 첫 명령이 최대 목표 초과

    def test_target_never_exceeds_max_over_a_grid(self):
        pp = params(lift_j2_max_target=77)
        for k in range(0, 21):
            actual = 68.32 + k * 0.5 - 1.0; t, d = L.second_j2_command(74, 68.32, actual, pp)
            if t is not None: self.assertLessEqual(t, 77, (actual, t))

    def test_second_j2_cap_unset_is_fail_closed(self):
        with self.assertRaises(P.ParamsNotFinal): L.second_j2_command(74, 69.0, 72.5)

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

    def test_e1_label_marks_possible_signal_shortfall_after_skipped_second_move(self):
        fail = L.e1_decide(self.A, self.b(side=0.1)); ok = L.e1_decide(self.A, self.b())
        self.assertTrue(L.e1_attempt_label(fail, {"second_j2": {"skipped_reason": "요청 폭 3.0° < 3.5°"}})["signal_shortfall_possible"])
        self.assertTrue(L.e1_attempt_label(fail, {"second_j2": {"no_response": True}})["signal_shortfall_possible"])
        self.assertFalse(L.e1_attempt_label(fail, {"second_j2": {"target": 77}})["signal_shortfall_possible"]); self.assertFalse(L.e1_attempt_label(ok, {"second_j2": {"skipped_reason": "x"}})["signal_shortfall_possible"])
        self.assertFalse(L.e1_attempt_label(fail, None)["signal_shortfall_possible"])

    def test_e3_gemini_center_is_2d_distance(self):
        b = {"found": True, "center": [10.0, 10.0], "side_px": 30.0}; diag = {"found": True, "center": [10.8, 10.8], "side_px": 30.0}   # 축별 0.8, 2D 1.13
        self.assertEqual(L.e3_verdict(146, 146, True, True, (b, diag))["verdict"], "불합격")
        self.assertEqual(L.e3_verdict(146, 146, True, True, (b, diag), params(e3_gemini_center_metric="axis"))["verdict"], "합격")

    def test_e3_p5_rules(self):
        self.assertEqual(L.e3_verdict(146, 146, hold_done=False, photo_c_done=False)["counts_as"], "실패")          # 유지 생략: E-2 원래 규칙
        self.assertEqual(L.e3_verdict(146, 147, True, False)["verdict"], "합격"); self.assertEqual(L.e3_verdict(146, 148, True, False)["verdict"], "불합격")
        b = {"found": True, "center": [10.0, 10.0], "side_px": 30.0}; c_ok = {"found": True, "center": [10.5, 10.5], "side_px": 30.5}; c_bad = {"found": True, "center": [12.0, 10.0], "side_px": 30.0}
        self.assertEqual(L.e3_verdict(146, 146, True, True, (b, c_ok))["verdict"], "합격"); self.assertEqual(L.e3_verdict(146, 146, True, True, (b, c_bad))["verdict"], "불합격")
        self.assertEqual(L.e3_verdict(146, 146, True, True, (b, {"found": False}))["counts_as"], "실패")

    def test_s4_gate_g2(self):
        ref = dict(self.A); small = dict(self.A, center=[673.5, 267.0]); big = dict(self.A, center=[675.0, 267.0])
        self.assertTrue(L.s4_gate(ref, small)["go"])                                    # 기본 PARAMS: G2 불감대 = x(P11 ① 확정)
        px = params(g2_deadzone_metric="x"); pd = params(g2_deadzone_metric="dist")
        self.assertTrue(L.s4_gate(ref, small, P=px)["go"])                               # 불감대 안: 입력 불필요
        with self.assertRaises(P.ParamsNotFinal): L.s4_gate(ref, big, P=px)              # 불감대 밖인데 Δx_block·d_x 없음 → 조용한 생략 금지
        with self.assertRaises(P.ParamsNotFinal): L.s4_gate(ref, big, dx_block_mm=1.2, P=px)
        self.assertFalse(L.s4_gate(ref, big, dx_block_mm=+1.2, dx_depth_mm=-2.0, P=px)["g2"])   # 예상 구간 (−3.26,−2.77) 음수, 이동 양수 → 방향 불일치
        self.assertTrue(L.s4_gate(ref, big, dx_block_mm=-1.2, dx_depth_mm=-2.0, P=px)["g2"])
        self.assertTrue(L.s4_gate(ref, big, dx_block_mm=+1.2, dx_depth_mm=1.0, P=px)["g2"])      # 예상 구간 (−0.26,0.23)이 0을 걸치면 방향 판정 생략
        self.assertFalse(L.s4_gate(ref, big, dx_block_mm=+4.0, dx_depth_mm=1.0, P=px)["g2"])     # 4 mm > 3.0
        # y만 1.2 px 이동 + 잡음 수준 dx_block(±0.1 mm): 어느 지표든, 어느 예상 부호든 go (2D 지표도 x가 불감대 안이면 부호 판정을 하지 않는다)
        yonly = dict(self.A, center=[673.0, 268.2])
        for pm in (px, pd):
            for depth in (2.0, -2.0):
                for dxb in (0.1, -0.1): self.assertTrue(L.s4_gate(ref, yonly, dx_block_mm=dxb, dx_depth_mm=depth, P=pm)["go"], (pm["g2_deadzone_metric"], depth, dxb))
        ydrift = dict(self.A, center=[673.0, 268.0])                                      # y만 1 px 이동
        self.assertTrue(L.s4_gate(ref, ydrift, P=px)["go"])                               # x 지표면 불감대 안
        with self.assertRaises(P.ParamsNotFinal): L.s4_gate(ref, ydrift, P=pd)            # 2D 지표면 불감대 밖인데 입력 없음 → 거부
        self.assertFalse(L.s4_gate(ref, dict(small, side_px=30.0), P=px)["go"]); self.assertFalse(L.s4_gate(ref, dict(small, yaw_deg=-175.0), P=px)["go"])
        self.assertFalse(L.s4_gate(ref, {"found": False}, P=px)["go"])

    def test_params_jsonable_survives_json(self):
        import json
        d = json.loads(json.dumps(L.params_jsonable())); self.assertEqual(d["q3_box"], "UNSET"); self.assertEqual(d["j2_window"], [66, 110]); self.assertIsNone(d["first_move_min_ms"])

    def test_q3_verdict_zncc_rule(self):
        z_ok = {"k1_a": 0.97, "k1_b": 0.95, "a_b": 0.96}; rec = {"shift_resp_ok": True, "zncc": z_ok}
        with self.assertRaises(P.ParamsNotFinal): L.q3_verdict(rec)                                    # 문턱 UNSET → 판정 없음
        pm = params(q3_zncc_min=0.90)
        self.assertEqual(L.q3_verdict(rec, pm)["verdict"], "합격")
        self.assertEqual(L.q3_verdict(dict(rec, zncc=dict(z_ok, k1_b=0.89)), pm)["verdict"], "판정 불가")   # 세 쌍 중 하나만 낮아도
        self.assertEqual(L.q3_verdict(dict(rec, zncc=dict(z_ok, a_b=0.90)), pm)["verdict"], "합격")         # 경계 0.90은 통과(< 문턱만 불가)
        self.assertEqual(L.q3_verdict(dict(rec, zncc=None), pm)["verdict"], "판정 불가")                   # 원값 없음 → 배제 못 함
        self.assertEqual(L.q3_verdict({"shift_resp_ok": False, "zncc": z_ok}, pm)["verdict"], "불합격")
        self.assertIn("실패", L.q3_verdict(dict(rec, zncc=None), pm)["counts_as"])

    def test_region_zncc_positive_and_negative_control(self):
        import numpy as np
        rng = np.random.RandomState(0); base = (rng.rand(120, 960) * 200 + 20).astype(np.uint8); k1 = np.dstack([base] * 3)
        box = (800, 0, 900, 100)
        same = k1.copy(); same[:, :, :] = np.clip(same.astype(int) + 3, 0, 255).astype(np.uint8)          # 밝기만 약간 변함
        arm = k1.copy(); arm[10:90, 820:880] = 255                                                          # 팔이 영역에 들어옴(양성 대조)
        self.assertGreater(L.region_zncc(k1, same, box), 0.99)
        self.assertLess(L.region_zncc(k1, arm, box), 0.90)
        self.assertEqual(L.region_zncc(k1, np.full_like(k1, 7), box), 0.0)                                  # 상수 영상 → 0.0
        pm = params(q3_box=box, q3_zncc_min=0.90); r_ok = L.q3_record(k1, same, pm, k1_bgr=k1); r_arm = L.q3_record(k1, arm, pm, k1_bgr=k1)
        self.assertEqual(L.q3_verdict(r_ok, pm)["verdict"], "합격"); self.assertEqual(L.q3_verdict(r_arm, pm)["verdict"], "판정 불가")
        self.assertIsNone(L.q3_record(k1, same, pm)["zncc"])

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
        self.assertEqual(r["old_box"], [480, 0, 900, 100]); self.assertEqual(r["box"], [800, 0, 900, 100]); self.assertTrue(r["shift_resp_ok"])
        self.assertEqual(r["new"], L.base_shift_fresh(a, b, (800, 0, 900, 100))); self.assertEqual(r["old"], L.base_shift_fresh(a, b, L.OLD_BASE_BOX))

    def test_q3_fails_when_shift_exceeds_2px(self):
        a, _ = self.imgs(); b = np.roll(a, 3, axis=1).copy()
        self.assertFalse(L.q3_record(a, b, params(q3_box=(800, 0, 900, 100)))["shift_resp_ok"])

if __name__ == "__main__": unittest.main()
