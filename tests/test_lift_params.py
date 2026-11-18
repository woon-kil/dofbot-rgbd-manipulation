"""들어올림 러너 상수 블록 시험(scripts/g01_lift_params.py): 미확정 상수가 있으면 --real 거부(fail-closed). unittest·pytest 모두 실행된다."""
import copy, sys, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import g01_lift_params as P


class LiftParamsTest(unittest.TestCase):
    def test_current_block_has_exactly_the_p9_unset_values(self):
        self.assertEqual(P.unset_names(), ["q3_arm_in_box_criterion", "q3_box"])

    def test_real_refused_while_unset(self):
        with self.assertRaises(P.ParamsNotFinal) as c: P.check_real_allowed("real")
        self.assertIn("q3_box", str(c.exception))

    def test_fake_and_dry_run_allowed_while_unset(self):
        P.check_real_allowed("fake"); P.check_real_allowed("dry_run")

    def test_real_allowed_only_when_every_unset_is_filled(self):
        p = copy.deepcopy(P.PARAMS); p["q3_box"] = (800, 0, 900, 100)
        with self.assertRaises(P.ParamsNotFinal): P.check_real_allowed("real", p)      # 하나만 채우면 아직 거부
        p["q3_arm_in_box_criterion"] = "x"; P.check_real_allowed("real", p)

    def test_none_is_a_legit_value_not_unset(self):
        self.assertIsNone(P.PARAMS["first_move_min_ms"]); self.assertNotIn("first_move_min_ms", P.unset_names())   # P7: 명시적 None
        p = copy.deepcopy(P.PARAMS); p["x"] = None; self.assertNotIn("x", P.unset_names(p))

    def test_removing_a_default_cannot_slip_through(self):
        p = copy.deepcopy(P.PARAMS); p["clock_limit_s"] = P.UNSET
        with self.assertRaises(P.ParamsNotFinal): P.check_real_allowed("real", p)

    def test_confirmed_values_match_approvals(self):
        d = P.PARAMS
        self.assertEqual((d["clock_start"], d["clock_limit_s"], d["move_countdown_s"], d["move_time_ms"], d["photo_wait_max_s"], d["pa_elapsed_margin_s"], d["j2_window"]),
                         ("145_write", 26.8, 0, 1500, 3.0, 300, (66, 110)))
        self.assertEqual((d["e1_center_up_px"], d["e1_side_len_px"], d["e1_lateral_px"], d["q3_shift_max_px"], d["q3_response_min"]), (8.0, 0.7, 4.0, 2.0, 0.3))
        self.assertEqual((d["photo_c_skip_e3"], d["hold_skip_e3"], d["return_photo_use"]), ("j6_readback_only", "e2_original", "record_only"))

    def test_unset_cannot_be_used_as_a_value(self):
        U = P.UNSET
        for f in (lambda: U or (480, 0, 900, 100), lambda: bool(U), lambda: U == 1, lambda: U + 1, lambda: U[0], lambda: list(U), lambda: len(U), lambda: float(U), lambda: U < 2, lambda: 1 in U):
            with self.assertRaises(P.ParamsNotFinal): f()
        self.assertEqual(repr(U), "UNSET")

    def test_unknown_mode_is_refused(self):
        for mode in ("REAL", "", "Real", "go", None):
            with self.assertRaises(P.ParamsNotFinal): P.check_real_allowed(mode)


if __name__ == "__main__": unittest.main()
