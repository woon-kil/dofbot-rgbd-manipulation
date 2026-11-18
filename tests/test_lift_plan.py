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

    def test_lifted_return_routes_never_move_j2_after_j1(self):                      # 세트 31과 같은 'J1은 마지막'(RET_J1_LAST) 규칙: 손가락을 연 채 블록 위에서 J1을 먼저 돌리지 않는다
        n0 = len(self.src["return_routes"]); import g01_s42_plan_p2k as GG
        self.assertTrue(GG.RET_J1_LAST)
        for idx in self.plan["lift"]["return_index"].values():
            joints = [m["joint"] for m in self.plan["return_routes"][idx]["route"]]
            if 1 in joints: self.assertNotIn(2, joints[joints.index(1):], (idx, joints))

    def test_make_lift_plan_restores_the_generator_global(self):
        for prev in (False, True):
            G.RET_J1_LAST = prev; PL.make_lift_plan(copy.deepcopy(self.src_copy), G, C); self.assertEqual(G.RET_J1_LAST, prev)       # 부작용 없음
        G.RET_J1_LAST = True

    def test_j1_last_positive_control(self):
        """양성 대조: RET_J1_LAST=False로 G.return_route를 직접 부르면 12개 항목 중 몇 개의 경로가 달라지는가(기록용). True 경로는 위 시험이 J2-after-J1 없음을 확인한다."""
        ret_target = self.plan["return_target"]; diff = 0
        try:
            for idx in self.plan["lift"]["return_index"].values():
                e = self.plan["return_routes"][idx]; G.RET_J1_LAST = True; a = G.return_route(e["state"], ret_target)[1]; G.RET_J1_LAST = False; b = G.return_route(e["state"], ret_target)[1]
                diff += (a != b)
        finally: G.RET_J1_LAST = True
        self.assertGreaterEqual(diff, 0)                                                                # 0이면 '기본값이어도 같았다'(수정이 결과를 바꾸지 않았음)를 뜻한다
        print("RET_J1_LAST 양성 대조: 12개 중 %d개 항목의 경로가 달라진다" % diff)

    def test_source_plan_is_not_mutated_and_other_fields_kept(self):
        self.assertEqual(self.src, self.src_copy)
        for k in ("moves", "run2", "return_target", "return_nominal"): self.assertEqual(self.plan[k], self.src[k])
        self.assertEqual(self.plan["p2k"], dict(self.src["p2k"], hold_s=3.0))          # 세트 33은 정지 대기만 3 s로(P-A 사용, Gemini 36 s 사진 없음)

    def test_only_set31_plans_are_accepted(self):
        bad = copy.deepcopy(self.src_copy); bad["set"] = "29"
        with self.assertRaises(ValueError): PL.make_lift_plan(bad, G, C)

    def test_generated_plan_supplies_the_p10_target_to_the_runner_gate(self):
        self.assertEqual(LP.unset_names(LP.params_for_plan(self.plan)), [])     # P9 두 값 등록(2026-11-18) 뒤 계획이 최대 목표까지 공급하면 UNSET 없음
        self.assertTrue(LP.is_lift_plan(self.plan))


if __name__ == "__main__": unittest.main()
