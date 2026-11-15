"""출발 전 공통 점검(src/planning/p1_preflight.py) 단위 테스트. 사진 점검은 gitignore된 data/g04의 실제 Gemini 사진이 있을 때만 실행한다."""
import hashlib
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from planning import p1_preflight as PF   # noqa: E402

IMG6 = ROOT / "data/g04/p1_block_check6_261115.png"
IMG7 = ROOT / "data/g04/p1_block_check7_261115.png"


class ReadbackTest(unittest.TestCase):
    def test_k1_exact_passes(self):
        self.assertTrue(PF.check_readback(list(PF.K1))["ok"])

    def test_p1_after_return_state_passes(self):   # P1 복귀 뒤 (89, 91, 92, 91, 90, 179)
        self.assertTrue(PF.check_readback([89, 91, 92, 91, 90, 179])["ok"])

    def test_j6_released_fails(self):              # 오므림 유지 확인: J6 98(열림)이면 실패
        r = PF.check_readback([89, 91, 91, 91, 90, 98])
        self.assertFalse(r["ok"]); self.assertEqual([x["joint"] for x in r["rows"] if not x["ok"]], [6])

    def test_boundaries(self):
        self.assertTrue(PF.check_readback([91, 93, 93, 89, 91, 180])["ok"])       # J1~J4 ±2, J5·J6 ±1 경계
        self.assertFalse(PF.check_readback([92, 91, 91, 91, 90, 179])["ok"])      # J1 +3
        self.assertFalse(PF.check_readback([89, 91, 91, 91, 92, 179])["ok"])      # J5 +2

    def test_none_fails(self):
        r = PF.check_readback([89, None, 91, 91, 90, 179])
        self.assertFalse(r["ok"]); self.assertIsNone(r["rows"][1]["diff"])

    def test_short_readback_fails(self):
        self.assertFalse(PF.check_readback([89, 91, 91])["ok"])


class ScriptsTest(unittest.TestCase):
    def _manifest(self, d, files):
        (Path(d) / "MANIFEST.sha256").write_text("".join("%s  %s\n" % (hashlib.sha256(c).hexdigest(), n) for n, c in files.items()), encoding="utf-8")
        return Path(d) / "MANIFEST.sha256"

    def test_match_and_mismatch(self):
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / "a.py").write_bytes(b"x"); m = self._manifest(d, {"a.py": b"x"})
            self.assertTrue(PF.check_scripts(m, d)["ok"])
            (Path(d) / "a.py").write_bytes(b"y")
            r = PF.check_scripts(m, d); self.assertFalse(r["ok"]); self.assertEqual(r["mismatch"], ["a.py"])

    def test_missing_file_fails(self):
        with tempfile.TemporaryDirectory() as d:
            m = self._manifest(d, {"gone.py": b"x"}); self.assertFalse(PF.check_scripts(m, d)["ok"])

    def test_nested_paths_and_custom_name(self):           # 저장소 루트 기준 상대 경로(하위 폴더)와 점검 이름
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / "src").mkdir(); (Path(d) / "src/a.py").write_bytes(b"q"); m = self._manifest(d, {"src/a.py": b"q"})
            r = PF.check_scripts(m, d, name="src_sha256"); self.assertTrue(r["ok"]); self.assertEqual(r["name"], "src_sha256")
            (Path(d) / "src/a.py").write_bytes(b"changed"); self.assertEqual(PF.check_scripts(m, d, name="src_sha256")["mismatch"], ["src/a.py"])


class RawSampleTest(unittest.TestCase):
    def _samples(self, jitter):
        import random
        random.seed(1); out = {}
        for j in (1, 2, 3, 4):
            ints, raws = [], []
            for _ in range(20):
                base = 2000 + random.randint(-jitter, jitter)
                a = 180.0 * (base - 900) / 2200; i = int(a); i = 180 - i if j in (2, 3, 4) else i
                ints.append(i); raws.append(base)
            out[str(j)] = {"int": ints, "raw": raws}
        return out

    def test_quiet_samples_pass_and_below_rounding(self):
        r = PF.check_raw_samples(self._samples(1))
        self.assertTrue(r["ok"]); self.assertFalse(r["gate"]); self.assertTrue(all(x["sd_below_rounding"] for x in r["rows"]))

    def test_noisy_samples_flagged(self):         # 원시 값이 ±40 카운트(약 ±3.3°) 흔들리면 반올림 흔들림보다 크다고 표시
        r = PF.check_raw_samples(self._samples(40)); self.assertFalse(any(x["sd_below_rounding"] for x in r["rows"]))

    def test_none_samples_fail(self):
        s = self._samples(1); s["2"]["raw"][0] = None
        self.assertFalse(PF.check_raw_samples(s)["ok"])

    def test_inconsistent_int_fails(self):        # 정수 회독이 원시 환산과 3° 어긋나면 불일치
        s = self._samples(1); s["3"]["int"] = [x + 3 for x in s["3"]["int"]]
        self.assertFalse(PF.check_raw_samples(s)["ok"])


class ProjectionTest(unittest.TestCase):
    NEAR = ROOT / "evidence/G06/p1_near_m3_poses_s42.json"; BIG = ROOT / "evidence/G06/p1_bigm3_poses_s42.json"

    def test_near_poses_pass(self):
        if not self.NEAR.exists(): self.skipTest("자세 파일 없음")
        r = PF.check_projection(self.NEAR); self.assertTrue(r["ok"], r); self.assertTrue(all(x["edge_margin_px"] >= 47 for x in r["rows"]))

    def test_invalid_bigm3_poses_fail(self):   # 무효가 된 큰 이동 M3 자세(마커가 영상 밖)는 반드시 실패해야 한다 — 2026-11-15 사고의 회귀 시험
        if not self.BIG.exists(): self.skipTest("자세 파일 없음")
        r = PF.check_projection(self.BIG); self.assertFalse(r["ok"]); self.assertTrue(any(x["edge_margin_px"] < 0 for x in r["rows"]), r)

    def test_margin_is_gate(self):
        if not self.NEAR.exists(): self.skipTest("자세 파일 없음")
        self.assertFalse(PF.check_projection(self.NEAR, margin=200.0)["ok"])


class ArmLibTest(unittest.TestCase):
    REF = ROOT / "data/refs/yahboom-Arm_Lib-1.0.3-Orin.py"

    def setUp(self):
        if not self.REF.exists(): self.skipTest("참고 소스 없음(gitignore)")

    def test_function_match_even_if_file_differs(self):   # 파일의 다른 곳이 달라도(주석 추가·들여쓰기 변경) 함수가 같으면 일치
        src = self.REF.read_text(encoding="utf-8"); other = "# 추가 주석\n" + src
        fh = PF.func_hash(other); self.assertEqual(fh, PF.func_hash(src))
        r = PF.check_arm_lib(fh, self.REF, installed_file_sha="0" * 64); self.assertTrue(r["ok"]); self.assertEqual(r["status"], "match"); self.assertFalse(r["file_sha_equal"])

    def test_function_mismatch_fails(self):               # 환산식이 바뀌면(3100 → 3000) 함수 해시가 달라 실패
        src = self.REF.read_text(encoding="utf-8").replace("(pos - 900) / (3100 - 900)", "(pos - 900) / (3000 - 900)")
        r = PF.check_arm_lib(PF.func_hash(src), self.REF); self.assertFalse(r["ok"]); self.assertEqual(r["status"], "mismatch")

    def test_unverifiable_is_distinct_failure(self):      # 확인 불가는 불일치와 구별되고 통과가 아니다
        r = PF.check_arm_lib(None, self.REF, error="OSError: zip 안의 파일"); self.assertFalse(r["ok"]); self.assertEqual(r["status"], "unverifiable")

    def test_lib_int_matches_source_formula(self):
        self.assertEqual(PF._lib_int(1000, 1), 8); self.assertEqual(PF._lib_int(1000, 3), 172); self.assertIsNone(PF._lib_int(800, 1))


@unittest.skipUnless(IMG6.exists() and IMG7.exists(), "data/g04 Gemini 사진 없음(gitignore)")
class SceneTest(unittest.TestCase):
    def test_same_scene_passes(self):   # P1-0에서 같은 장면으로 확인한 두 사진(마커 중심 (702.8, 273.0))
        r = PF.check_scene(IMG7, IMG6, (702.8, 273.0))
        self.assertTrue(r["ok"], r); self.assertLessEqual(r["camera_shift_px"], 0.5)

    def test_marker_moved_fails(self):   # 기준 마커 위치를 10 px 틀어 놓으면 마커 점검이 실패해야 한다
        r = PF.check_scene(IMG7, IMG6, (712.8, 273.0))
        self.assertFalse(r["ok"]); self.assertFalse(r["marker_ok"]); self.assertTrue(r["camera_still_ok"])

    def test_missing_image_fails(self):
        self.assertFalse(PF.check_scene(IMG7.with_name("nope.png"), IMG6, (702.8, 273.0))["ok"])

    def test_camera_moved_but_block_still_passes(self):   # 카메라가 균일하게 약 2.4 px 움직여도(전체 영상 이동) 블록의 장면 내 이동은 작으므로 통과, 카메라 이동은 기록(참고 폭 초과)
        import cv2, numpy as np, tempfile
        im = cv2.imread(str(IMG7)); M = np.float32([[1, 0, -2.4], [0, 1, -0.6]])
        sh = cv2.warpAffine(im, M, (im.shape[1], im.shape[0]), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REFLECT)
        with tempfile.TemporaryDirectory() as d:
            f = Path(d) / "shifted.png"; cv2.imwrite(str(f), sh); r = PF.check_scene(f, IMG6, (702.8, 273.0))
        self.assertTrue(r["ok"], r); self.assertFalse(r["camera_still_ok"]); self.assertLess(r["block_move_in_scene_px"], 1.0); self.assertGreater(r["camera_shift_px"], 2.0)

    def test_block_moved_in_scene_fails(self):   # 마커만 장면 안에서 6 px 옮겨 붙이면(블록 이동 모사) 실패
        import cv2, numpy as np, tempfile
        im = cv2.imread(str(IMG7)).copy(); rf = cv2.imread(str(IMG6))
        det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), cv2.aruco.DetectorParameters())
        c, ids, _ = det.detectMarkers(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)); q = c[0].reshape(4, 2)
        x0, y0 = int(q[:, 0].min()) - 6, int(q[:, 1].min()) - 6; x1, y1 = int(q[:, 0].max()) + 6, int(q[:, 1].max()) + 6
        patch = im[y0:y1, x0:x1].copy(); im[y0:y1, x0:x1] = rf[y0:y1, x0:x1] if False else im[y0 - 20:y1 - 20, x0:x1]   # 마커 자리를 위쪽 배경으로 덮고
        im[y0 + 6:y1 + 6, x0:x1] = patch                                                                              # 마커를 아래로 6 px 옮김
        with tempfile.TemporaryDirectory() as d:
            f = Path(d) / "moved.png"; cv2.imwrite(str(f), im); r = PF.check_scene(f, IMG6, (702.8, 273.0))
        self.assertFalse(r["ok"], r); self.assertGreater(r["block_move_in_scene_px"], 4.0)


if __name__ == "__main__":
    unittest.main()
