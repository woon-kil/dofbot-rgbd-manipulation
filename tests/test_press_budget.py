"""총 누름 시간 상한(src/planning/press_budget.py) 단위 테스트(가짜 시계, 하드웨어 없음)."""
import sys, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from planning.press_budget import PressBudget, BudgetExceeded


class FakeClock:
    def __init__(self): self.t = 0.0
    def __call__(self): return self.t


class PressBudgetTest(unittest.TestCase):
    def test_disabled_by_default_never_raises(self):
        c = FakeClock(); b = PressBudget(None, c); b.start(); c.t = 1000.0
        self.assertEqual(b.check(), 1000.0)

    def test_accumulates_over_segments_and_raises_over_limit(self):
        c = FakeClock(); b = PressBudget(5.0, c)
        b.start(); c.t = 2.0; b.stop(); c.t = 10.0          # 풀린 구간(8 s)은 세지 않는다
        b.start(); c.t = 12.5                                # 두 번째 구간 2.5 s → 누적 4.5 s
        self.assertAlmostEqual(b.check(), 4.5)
        c.t = 13.1                                           # 누적 5.1 s > 5.0
        with self.assertRaises(BudgetExceeded): b.check()

    def test_stop_without_start_and_double_start_are_safe(self):
        c = FakeClock(); b = PressBudget(3.0, c); b.stop()
        b.start(); c.t = 1.0; b.start(); c.t = 2.0; b.stop()
        self.assertAlmostEqual(b.used_s(), 2.0)


if __name__ == "__main__":
    unittest.main()
