"""J1 가정 파일(`evidence/G06/lift_j1_assumption_*.txt`)과 `LMR.j1_assumption`/`check_plan_j1` 시험(플랜 v2 §4 J1 85, 사용자 승인 2026-11-19).
마커 기준 파일(`lift_marker_ref_*.txt`, 정확히 1개 규칙)은 건드리지 않고 J1 가정만 별도 단일 파일에서 읽는다. 파일이 정확히 1개가 아니거나 형식이 틀리면 오류(정렬 NG, fail-closed)."""
import hashlib, json, shutil, sys, tempfile, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import g01_lift_marker_ref as M


class J1AssumptionTest(unittest.TestCase):
    def setUp(self):
        self.d = Path(tempfile.mkdtemp()); (self.d / "evidence" / "G06").mkdir(parents=True)
    def tearDown(self): shutil.rmtree(self.d, ignore_errors=True)
    def put(self, name, text): (self.d / "evidence" / "G06" / name).write_text(text, encoding="utf-8")

    def test_requires_exactly_one_file(self):
        with self.assertRaises(M.MarkerRefError): M.j1_assumption(root=str(self.d))
        self.put("lift_j1_assumption_000000-0000.txt", "# c\n85 85.4\n"); self.assertEqual(M.j1_assumption(root=str(self.d)), 85.4)
        self.put("lift_j1_assumption_000000-0001.txt", "# c\n85 85.4\n")
        with self.assertRaises(M.MarkerRefError): M.j1_assumption(root=str(self.d))
    def test_bad_formats_fail_closed(self):
        for text in ("", "# only comment\n", "85\n", "85 abc\n", "85 85.4 extra\n", "0 85.4\n", "85 -1\n", "85 85.4\n85 86.0\n"):
            for f in (self.d / "evidence" / "G06").glob("lift_j1_assumption_*"): f.unlink()
            self.put("lift_j1_assumption_000000-0000.txt", text)
            with self.assertRaises(M.MarkerRefError, msg=repr(text)): M.j1_assumption(root=str(self.d))
    def test_command_and_plan_check(self):
        self.put("lift_j1_assumption_000000-0000.txt", "# c\n85 85.4\n"); self.assertEqual(M.j1_command(root=str(self.d)), 85)
        plan = {"lift": {"contact_moves_done": 1}, "return_routes": [{"state": [85, 69, 12, 32]}, {"state": [85, 69, 12, 32]}]}
        p = self.d / "plan.json"; p.write_text(json.dumps(plan), encoding="utf-8"); M.check_plan_j1(str(p), root=str(self.d))   # 같으면 통과
        plan["return_routes"][1]["state"][0] = 86; p.write_text(json.dumps(plan), encoding="utf-8")
        with self.assertRaises(M.MarkerRefError): M.check_plan_j1(str(p), root=str(self.d))   # 계획의 접촉 자세 J1 명령이 가정 파일의 명령과 다르면 오류
    def test_j1_raw_for_plan_uses_plan_j1_not_set_number(self):
        """J1 원시 가정은 세트 번호가 아니라 계획의 접촉 자세 J1 명령에서 정한다(간격 도구 `_J1A` 하드코딩 결함 재발 방지)."""
        mk = lambda j1, st: {"set": st, "lift": {"contact_moves_done": 0}, "return_routes": [{"state": [j1, 69, 12, 32]}]}
        self.assertEqual(M.j1_raw_for_plan(mk(85, "33")), 85.4); self.assertEqual(M.j1_raw_for_plan(mk(86, "33")), 86.4)
        with self.assertRaises(M.MarkerRefError): M.j1_raw_for_plan(mk(84, "33"))
        self.assertEqual(M.j1_raw_for_plan({"set": "31"}), 86.4); self.assertEqual(M.j1_raw_for_plan({"set": "30"}), 87.4)
        for f in ("evidence/G06/run2_contact_clearance_261117.py",): self.assertIn("j1_raw_for_plan(plan)", (ROOT / f).read_text(encoding="utf-8"))
        self.assertNotIn("in ('31', '33') else 87.4", (ROOT / "evidence/G06/run2_contact_clearance_261117.py").read_text(encoding="utf-8"))
        self.assertIn("LMR.j1_assumption()", (ROOT / "scripts/p2k_align_report.py").read_text(encoding="utf-8"))      # 정렬 보고도 J1 가정 파일을 읽는다
    def test_real_repo_file_and_old_files_unchanged(self):
        self.assertEqual(M.j1_assumption(), 85.4); self.assertEqual(M.j1_command(), 85)
        sha = lambda p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest()
        self.assertTrue(sha("evidence/G06/lift_marker_ref_261118-1702.txt").startswith("322c5bb893ad"))   # 마커 기준 파일 불변
        self.assertTrue(sha("evidence/G06/p2k_marker_ref.txt").startswith("46ea1aa420de"))
        x, y, j1_old, name, _ = M.load(); self.assertEqual((x, y, j1_old), (673.25, 267.25, 86.4))     # 마커 좌표 불변, 옛 J1 줄도 그대로


if __name__ == "__main__": unittest.main()
