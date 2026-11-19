"""J1 85 계획(세트 34 → 세트 33 입력) 시험(플랜 v2 §4, 사용자 승인 2026-11-19): 세트 31(J1 86)은 그대로이고, J1 85 기본 계획(새 자세 3111~3115)만 세트 33의 입력이 된다."""
import json, sys, tempfile, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
try:
    import g01_s42_plan_lift as PL, g01_lift_marker_ref as LMR; G, C = PL._load_modules()
except Exception as e: PL = None


@unittest.skipIf(PL is None, "생성기 의존성 없음")
class J185PlanTest(unittest.TestCase):
    def test_set34_fixture_makes_set33_with_j1_85_contact_and_assumed_raw(self):
        plan34 = json.load(open(ROOT / "tests" / "fixtures" / "s4_2_plan_34_fixture.json", encoding="utf-8")); self.assertEqual(plan34["set"], "34")
        plan, errs = PL.make_lift_plan(plan34, G, C); self.assertEqual(errs, []); self.assertEqual(plan["set"], "33")
        L = plan["lift"]; st = plan["return_routes"][L["contact_moves_done"]]["state"]
        self.assertEqual([int(v) for v in st], [85, 69, 12, 32]); self.assertEqual(L["q0_source"], "run4_raw_j1_assumed"); self.assertAlmostEqual(L["q0"][0], LMR.j1_assumption())
        self.assertEqual((L["j2_first_target"], L["j2_max_target"]), (74, 77))           # J1은 tilt 계산에 영향이 없다(J1 86 계획과 같은 값)
        p = Path(tempfile.mkdtemp()) / "plan33.json"; p.write_text(json.dumps(plan), encoding="utf-8"); self.assertEqual(LMR.check_plan_j1(str(p)), 85)   # 가정 파일의 명령과 일치
    def test_set31_path_unchanged_and_poses_3101_to_3105_intact(self):
        plan31 = json.load(open(ROOT / "tests" / "fixtures" / "s4_2_plan_31_fixture.json", encoding="utf-8")); plan, _ = PL.make_lift_plan(plan31, G, C)
        L = plan["lift"]; self.assertEqual(L["q0_source"], "run4_raw"); self.assertEqual([int(v) for v in plan["return_routes"][L["contact_moves_done"]]["state"]], [86, 69, 12, 32])
        P = {q["id"]: q["sdk_deg"] for q in json.load(open(ROOT / "evidence" / "G06" / "p2k_poses_s42.json", encoding="utf-8"))["poses"]}
        for i in range(1, 6): self.assertEqual(P[3100 + i][0], 86); self.assertEqual(P[3110 + i][0], 85); self.assertEqual(P[3100 + i][1:], P[3110 + i][1:])   # J1만 다르다
    def test_j1_86_plan_with_j1_85_assumption_file_is_refused(self):
        plan31 = json.load(open(ROOT / "tests" / "fixtures" / "s4_2_plan_31_fixture.json", encoding="utf-8")); plan, _ = PL.make_lift_plan(plan31, G, C)
        p = Path(tempfile.mkdtemp()) / "plan33.json"; p.write_text(json.dumps(plan), encoding="utf-8")
        with self.assertRaises(LMR.MarkerRefError): LMR.check_plan_j1(str(p))                 # J1 86 계획은 J1 가정 파일(명령 85)과 맞지 않아 정렬 NG


if __name__ == "__main__": unittest.main()
