"""세트 33 계획 생성(scripts/g01_s42_plan_lift.py) 시험. 세트 31 계획 픽스처(tests/fixtures)로 생성기 import·FK가 필요해 의존성이 없으면 건너뛴다(venv 전용)."""
import copy, json, os, sys, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
try:
    cwd = os.getcwd(); os.chdir(ROOT)
    import g01_s42_plan_lift as PL
    G, C = PL._load_modules()
except Exception as e:
    PL = None; _err = e
finally:
    os.chdir(cwd)
import g01_lift_params as LP

FIX = ROOT / "tests" / "fixtures" / "s4_2_plan_31_fixture.json"


@unittest.skipIf(PL is None, "생성기·FK 의존성 없음")
class LiftPlanTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.src = json.load(open(FIX, encoding="utf-8")); cls.src_copy = copy.deepcopy(cls.src)
        cls.plan, cls.errs = PL.make_lift_plan(cls.src, G, C)

    def test_set33_lift_clause_matches_p10_principle(self):
        L = self.plan["lift"]
        self.assertEqual(self.plan["set"], "33"); self.assertEqual((L["j2_first_target"], L["j2_max_target"]), (74, 77)); self.assertEqual(L["q0_source"], "run4_raw")
        self.assertLessEqual(L["tilt_peak_at_max_deg"], 30.0); self.assertGreater(L["tilt_peak_at_max_deg"], 29.0); self.assertEqual(self.errs, [])

    def test_max_target_is_the_largest_integer_within_30_degrees(self):
        p77, _ = PL.worst_case_peak_tilt(C, C.Q0, 74, 77); p78, _ = PL.worst_case_peak_tilt(C, C.Q0, 74, 78); p76, _ = PL.worst_case_peak_tilt(C, C.Q0, 74, 76)
        self.assertAlmostEqual(p76, 28.8, delta=0.1); self.assertAlmostEqual(p77, 29.8, delta=0.1); self.assertAlmostEqual(p78, 30.8, delta=0.1)   # 자문·세션 표와 일치
        self.assertEqual(PL.max_target(C, C.Q0, 74)[0], 77); self.assertEqual(PL.max_target(C, C.Q0, 74, limit=29.0)[0], 76); self.assertLess(PL.max_target(C, C.Q0, 74, limit=28.0)[0], 76)   # 한도를 낮추면 목표도 내려간다

    def test_lifted_return_entries_are_appended_after_the_verified_routes(self):
        n0 = len(self.src["return_routes"]); L = self.plan["lift"]; rr = self.plan["return_routes"]
        self.assertEqual(len(rr), n0 + len(L["return_index"])); self.assertEqual(rr[:n0], self.src_copy["return_routes"])           # 기존 항목 불변
        for key, idx in L["return_index"].items():
            e = rr[idx]; self.assertEqual(e["after_move"], idx); self.assertEqual(e["state"][0], 86); self.assertEqual(e["state"][2], 12); self.assertTrue(e["route"])
            stage, j2 = key.split(":"); self.assertEqual(e["state"][1], int(j2)); self.assertEqual(e["state"][3], 32 if stage == "before_j4" else 27)
        self.assertEqual(sorted(int(k.split(":")[1]) for k in L["return_index"] if k.startswith("after_j4")), [72, 73, 74, 75, 76, 77])

    def test_source_plan_is_not_mutated_and_other_fields_kept(self):
        self.assertEqual(self.src, self.src_copy)
        for k in ("moves", "run2", "return_target", "return_nominal"): self.assertEqual(self.plan[k], self.src[k])
        self.assertEqual(self.plan["p2k"], dict(self.src["p2k"], hold_s=3.0))          # 세트 33은 정지 대기만 3 s로(P-A 사용, Gemini 36 s 사진 없음)

    def test_only_set31_plans_are_accepted(self):
        bad = copy.deepcopy(self.src_copy); bad["set"] = "29"
        with self.assertRaises(ValueError): PL.make_lift_plan(bad, G, C)

    def test_generated_plan_supplies_the_p10_target_to_the_runner_gate(self):
        self.assertEqual(LP.unset_names(LP.params_for_plan(self.plan)), ["q3_arm_in_box_criterion", "q3_box"])     # 남은 UNSET은 P9 두 개뿐
        self.assertTrue(LP.is_lift_plan(self.plan))


if __name__ == "__main__": unittest.main()
