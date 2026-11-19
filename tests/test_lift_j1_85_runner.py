"""J1 85 계획(세트 34 → 세트 33)으로 가짜 러너를 돌려 사용자에게 묻는 안전 질문이 계획의 J1(85)을 말하는지(86이라고 거짓말하지 않는지) 확인한다(플랜 v2, CLAUDE.md 하드웨어 점검 7번)."""
import contextlib, io, json, sys, tempfile, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "scripts"), str(ROOT / "tests")]
import test_lift_hook as T
vclock, R = T.vclock, T.R


@unittest.skipIf(vclock is None, "러너·생성기 의존성 없음")
class J185RunnerQuestionTest(unittest.TestCase):
    def run_with(self, fixture):
        plan, _ = T.PL.make_lift_plan(json.load(open(ROOT / "tests" / "fixtures" / fixture, encoding="utf-8")), T.G, T.C)
        p = Path(tempfile.mkdtemp()) / "plan33.json"; p.write_text(json.dumps(plan), encoding="utf-8")
        old = (R.LIFT_CHANNEL, R.ask); asked = []
        def ask(msg, default=False): asked.append(msg); return True
        R.ask = ask; R.LIFT_CHANNEL = T.FakeChannel(T.REF, T.photos())
        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf): vclock.run_runner_fake(p, ("--fake-ceil", "6:146", "--fake-hyst", "1.9,0.27"))
        finally: R.LIFT_CHANNEL, R.ask = old
        return " ".join(asked) + buf.getvalue()   # 확인 질문 목록(출력)과 개별 질문
    def test_questions_name_the_plan_j1(self):
        q85 = self.run_with("s4_2_plan_34_fixture.json"); self.assertIn("J1 85", q85); self.assertNotIn("J1 86", q85)
        q86 = self.run_with("s4_2_plan_31_fixture.json"); self.assertIn("J1 86", q86); self.assertNotIn("J1 85", q86)


if __name__ == "__main__": unittest.main()
