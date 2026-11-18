"""세트 33 마커 기준 재등록 도구(g01_lift_marker_ref) 시험: 기존 사진으로 양성·음성 대조, 1회 한정·비덮어쓰기·시도 상한·승인 문구. venv 전용(cv2)."""
import os, shutil, sys, tempfile, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
try:
    import cv2; import g01_lift_marker_ref as M
except ImportError: M = None
G = ROOT / "data" / "g04"
RUN4 = G / "p2kpost_261117-153705.png"; RUN3 = G / "p2kpost_261117-143127.png"


@unittest.skipUnless(M is not None and RUN4.exists() and RUN3.exists(), "cv2 또는 원 사진 없음")
class TestRegister(unittest.TestCase):
    def setUp(self):
        self.d = Path(tempfile.mkdtemp()); (self.d / "evidence" / "G06").mkdir(parents=True)
        shutil.copy(ROOT / "evidence/G06/p2k_marker_ref.txt", self.d / "evidence/G06/p2k_marker_ref.txt")
        self.saved = (M.ROOT, M.G06, M.OLD_REF, M.ATT, M.RUN4_POST)
        M.ROOT = str(self.d); M.G06 = str(self.d / "evidence/G06"); M.OLD_REF = str(self.d / "evidence/G06/p2k_marker_ref.txt"); M.ATT = str(self.d / "evidence/G06/lift_marker_attempts.log"); M.RUN4_POST = str(RUN4)

    def tearDown(self): M.ROOT, M.G06, M.OLD_REF, M.ATT, M.RUN4_POST = self.saved; shutil.rmtree(self.d, ignore_errors=True)

    def test_load_requires_exactly_one_file(self):
        with self.assertRaises(M.MarkerRefError): M.load()                                                  # 0개
        out = M.register(str(RUN4), "사용자 건별 승인 2026-11-18"); x, y, j1, name, meta = M.load()
        self.assertTrue(name.startswith("lift_marker_ref_")); self.assertAlmostEqual(x, 673.5, places=1); self.assertEqual(j1, 86.4)   # Run 4 복귀 뒤 사진은 0 px로 합격, J1 기대값은 옛 파일에서 복사
        (self.d / "evidence/G06/lift_marker_ref_000000-0000.txt").write_text("1 2\n86.4\n")
        with self.assertRaises(M.MarkerRefError): M.load()                                                  # 2개 이상도 오류

    def test_old_reference_is_preserved_and_recorded(self):
        before = (self.d / "evidence/G06/p2k_marker_ref.txt").read_text(); out = M.register(str(RUN4), "승인")
        self.assertEqual((self.d / "evidence/G06/p2k_marker_ref.txt").read_text(), before); txt = Path(out).read_text(encoding="utf-8")
        self.assertIn("676.25 266.5", txt); self.assertIn("1회", txt); self.assertIn("sha256", txt); self.assertIn("승인", txt)

    def test_far_photo_is_refused_and_attempt_logged(self):
        with self.assertRaises(M.MarkerRefError) as c: M.register(str(RUN3), "승인")                         # Run 3 복귀 뒤: 약 2.9 px
        self.assertIn("px", str(c.exception)); self.assertEqual(M.files(), []); self.assertEqual(M._attempt_count(), 1)

    def test_attempt_cap_and_one_shot_and_approval(self):
        with self.assertRaises(M.MarkerRefError): M.register(str(RUN4), "  ")                                # 승인 문구 필수
        for _ in range(M.MAX_ATTEMPTS):
            with self.assertRaises(M.MarkerRefError): M.register(str(RUN3), "승인")
        with self.assertRaises(M.MarkerRefError) as c: M.register(str(RUN4), "승인")                         # 상한 뒤에는 합격 사진이어도 받지 않는다
        self.assertIn("올린다", str(c.exception))

    def test_cannot_overwrite_existing_registration(self):
        M.register(str(RUN4), "승인")
        with self.assertRaises(M.MarkerRefError) as c: M.register(str(RUN4), "승인 2")
        self.assertIn("이번 1회", str(c.exception)); self.assertEqual(len(M.files()), 1)


class TestWrapperStrings(unittest.TestCase):
    def test_wrapper_uses_only_lift_ref_and_fixes_question_text(self):
        t = (ROOT / "scripts" / "g01_lift_run.sh").read_text(encoding="utf-8")
        self.assertNotIn("678.2", t); self.assertNotIn("Run 3 이후 만지지 않았고", t); self.assertIn("LMR.load()", t); self.assertIn("마지막으로 위치를 확인한 사진(재등록 또는 복원 재측정) 이후 만지지 않았고", t.replace("**", "")); self.assertNotIn("재등록 사진 이후 만지지 않았고", t.replace("**", ""))   # T6: 래퍼 질문은 마커 관문보다 앞이라 관문 통과를 말하지 않는다
        self.assertIn("g01_lift_classify.py", t)
        self.assertLess(t.index("g01_lift_marker_ref.py\" info"), t.index('scp -q -o BatchMode=yes "${ROOT}/scripts/g01_dabai_probe.py"'))   # 파일 확인이 배포보다 앞


if __name__ == "__main__": unittest.main()
