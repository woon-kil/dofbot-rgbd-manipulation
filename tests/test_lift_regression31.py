"""세트 31 회귀 시험(M2b-2 선행, 플랜 §3-1 ④): g01_p2k_test.py(검증된 본체)를 들어올림 훅으로 고치기 전의 코드로 세트 31 가짜 실행을 가상 시계로 돌려 저장한
결정적 요약(tests/fixtures/set31_fake_baseline.json: 이동 목록·어댑터 쓰기 로그·접촉 분기 결과·종료 상태)과, 고친 뒤의 실행이 같은지 비교한다. venv 전용(의존성 없으면 건너뜀)."""
import json, os, sys, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "scripts"), str(ROOT / "tests")]
try:
    import vclock, g01_p2k_test  # noqa
except Exception as e:
    vclock = None

PLAN = ROOT / "tests" / "fixtures" / "s4_2_plan_31_fixture.json"; BASE = ROOT / "tests" / "fixtures" / "set31_fake_baseline.json"


@unittest.skipIf(vclock is None, "러너 import 불가")
class Set31RegressionTest(unittest.TestCase):
    def test_set31_fake_run_matches_the_pre_change_baseline(self):
        got = vclock.digest(vclock.run_runner_fake(PLAN)); base = json.load(open(BASE, encoding="utf-8"))
        self.assertEqual(got["ended"], "completed")
        for k in base: self.assertEqual(got[k], base[k], "세트 31 회귀 불일치: %s" % k)
        self.assertEqual(got, base)

    def test_fake_run_is_deterministic(self):
        self.assertEqual(vclock.digest(vclock.run_runner_fake(PLAN)), vclock.digest(vclock.run_runner_fake(PLAN)))

    def test_baseline_contains_the_contact_branch_and_release(self):
        base = json.load(open(BASE, encoding="utf-8")); r2 = base["poses"][3]["run2"]
        self.assertIn("close_steps", r2); self.assertIn("release_readback", r2); self.assertEqual([s["target"] for s in r2["close_steps"]][:2], [120, 130])
        self.assertIn("contact", base["j6_adapter_logs"]); self.assertTrue(base["returned_to_start"])


if __name__ == "__main__": unittest.main()
