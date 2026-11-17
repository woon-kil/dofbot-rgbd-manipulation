"""쥔 블록 들어올림 경로 점검(evidence/G06/carried_block_path_261117.py) 단위 테스트. FK 기반 순서 판정을 고정한다(하드웨어 없음)."""
import os, sys, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
os.chdir(ROOT)
for p in (ROOT / "src", ROOT / "scripts", ROOT / "evidence" / "G06"): sys.path.insert(0, str(p))
import carried_block_path_261117 as C

WORST = {"inc_shortfall_deg": 2.0, "dec_overshoot_deg": 0.8}


class CarriedBlockPathTest(unittest.TestCase):
    def test_j4_first_presses_block_into_paper(self):
        st, ok = C.follow([(4, -5), (2, 4), (2, 4)])
        self.assertFalse(ok); self.assertLess(st[0]["dz"], 0)

    def test_nominal_end_point_is_lift_of_about_19mm(self):
        st, ok = C.follow([(2, 4), (2, 4), (4, -5)])
        self.assertTrue(ok); self.assertAlmostEqual(st[-1]["dz"], 19.0, delta=0.5); self.assertLess(abs(st[-1]["dy"]), 1.5)

    def test_j2_j4_j2_fails_under_worst_servo_error(self):
        _, ok_nom = C.follow([(2, 4), (4, -5), (2, 4)]); st, ok_worst = C.follow([(2, 4), (4, -5), (2, 4)], **WORST)
        self.assertTrue(ok_nom); self.assertFalse(ok_worst); self.assertLess(st[1]["bottom_mm"], 2.0)

    def test_j2_j2_j4_passes_under_worst_servo_error(self):
        st, ok = C.follow([(2, 4), (2, 4), (4, -5)], **WORST)
        self.assertTrue(ok); self.assertGreaterEqual(min(s["bottom_mm"] for s in st), 5.0)


if __name__ == "__main__":
    unittest.main()
