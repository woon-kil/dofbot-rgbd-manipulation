"""M6 정렬 NG 자동 N 시험(T13, 플랜 v1.3 M6·v2 §4, 사용자 승인 D2·`ALIGN_TIMEOUT_S = 15`). 시험 먼저: 구현 전에는 실패해야 한다.
규칙: 세트 33(dry_run 제외)에서 정렬 판정(OK/NG)을 Mac 다리가 파일로 Jetson에 전달하고, 러너는 `[정렬 확인]` 뒤 판정을 최대 ALIGN_TIMEOUT_S(15 s) 기다린다.
NG·없음·시간 초과·이전 실행(run_id 불일치)·옛 요청(토큰 불일치)·형식 오류는 **사람 입력 없이** 자동 N(`Abort`)이고, OK일 때만 기존 사람 y/N 질문을 한다."""
import json, sys, time, unittest, tempfile
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "scripts"), str(ROOT / "tests")]
import test_lift_hook as T
import g01_lift_bridge as B
vclock, R, LH = T.vclock, T.R, T.LH
QUESTION = "정렬 확인이 모두 맞습니까?"
OK_TEXT = "d_x = +2.55 mm (허용 ±3), 캡별 옆 여유 [왼쪽 8.15, 오른쪽 13.06] mm (허용 ≥ 6), 참고 J1 보정량 -0.82°\n결과: 정렬 OK — 조건이 모두 맞으면 러너 질문에 y\n"
NG_TEXT = "d_x = +3.06 mm (허용 ±3), 캡별 옆 여유 [왼쪽 7.73, 오른쪽 13.66] mm (허용 ≥ 6), 참고 J1 보정량 -0.98°\n결과: 정렬 NG — 러너 질문에 N (하강 없이 복귀)\n"


class BridgeVerdictTest(unittest.TestCase):
    def test_constants_are_the_approved_value_and_equal(self):
        self.assertEqual(B.ALIGN_TIMEOUT_S, 15); self.assertEqual(LH.ALIGN_TIMEOUT_S, B.ALIGN_TIMEOUT_S)
    def test_verdict_from_report_text(self):
        v = B.align_verdict(OK_TEXT, "run1", "run1-align_verdict-5"); self.assertEqual((v["verdict"], v["run_id"], v["request_id"]), ("OK", "run1", "run1-align_verdict-5"))
        self.assertEqual(B.align_verdict(NG_TEXT, "run1", "t")["verdict"], "NG")
        for bad in ("", "계산 실패", "결과: 정렬 OK 가 아닌 문장", B.ALIGN_FAIL % "x" if hasattr(B, "ALIGN_FAIL") else "정렬 보고 계산 실패"):
            self.assertEqual(B.align_verdict(bad, "run1", "t")["verdict"], "NG", bad)   # 보고가 없거나 형식이 다르면 NG(OK는 명시적 OK 줄이 있을 때만)
    def test_bridge_pushes_verdict_for_request_line(self):
        pushed = []; br = B.Bridge({"detect": {"found": True}, "started_epoch": 1.0}, lambda n: {}, lambda: 1.0, lambda n, r: pushed.append((n, r)), P=dict(T.LP.PARAMS), say=lambda s: None, align=lambda: OK_TEXT)
        br.handle("\n[정렬 확인] Mac 화면의 정렬 보고"); br.handle("[사진 요청 align_verdict id=run1-align_verdict-9]")
        self.assertEqual([p[0] for p in pushed], ["align_verdict"]); self.assertEqual((pushed[0][1]["verdict"], pushed[0][1]["request_id"], pushed[0][1]["run_id"]), ("OK", "run1-align_verdict-9", "run1"))
    def test_bridge_pushes_ng_when_report_failed_or_missing(self):
        pushed = []
        def boom(): raise RuntimeError("정렬 보고 계산 실패")
        br = B.Bridge({"detect": {"found": True}, "started_epoch": 1.0}, lambda n: {}, lambda: 1.0, lambda n, r: pushed.append((n, r)), P=dict(T.LP.PARAMS), say=lambda s: None, align=boom)
        br.handle("\n[정렬 확인] Mac 화면의 정렬 보고"); br.handle("[사진 요청 align_verdict id=run1-align_verdict-9]")
        self.assertEqual(pushed[0][1]["verdict"], "NG")
        pushed2 = []; br2 = B.Bridge({"detect": {"found": True}, "started_epoch": 1.0}, lambda n: {}, lambda: 1.0, lambda n, r: pushed2.append((n, r)), P=dict(T.LP.PARAMS), say=lambda s: None, align=None)
        br2.handle("[사진 요청 align_verdict id=run1-align_verdict-9]"); self.assertEqual(pushed2[0][1]["verdict"], "NG")   # 보고를 만든 적이 없으면 NG


@unittest.skipIf(vclock is None, "러너·생성기 의존성 없음")
class AlignGateTest(unittest.TestCase):
    def gate(self, resp, **kw):
        ch = T.FakeChannel(T.REF, {"align_verdict": resp} if resp is not None else {}, **kw)
        with vclock.virtual_clock() as t:
            t0 = t[0]; out = LH.align_gate(ch, "reg"); return out, t[0] - t0
    def test_ok_passes(self):
        (go, why), el = self.gate({"run_id": "reg", "verdict": "OK"}); self.assertTrue(go, why)
    def test_ng_and_garbage_fail(self):
        for resp in ({"run_id": "reg", "verdict": "NG"}, {"run_id": "reg"}, {"run_id": "reg", "verdict": "ok"}, {"run_id": "reg", "verdict": "OK "}, {"verdict": "OK"}, {"run_id": "reg", "verdict": None}):
            (go, why), _ = self.gate(resp); self.assertFalse(go, resp)
    def test_non_dict_response_fails(self):
        class Ch:
            def request(self, name, run_id, extra=""): return "tok"
            def wait(self, name, timeout_s, token): return "not a dict"
        self.assertFalse(LH.align_gate(Ch(), "reg")[0])
    def test_other_run_id_fails_even_if_ok(self):
        (go, why), _ = self.gate({"run_id": "old-run", "verdict": "OK"}); self.assertFalse(go); self.assertIn("run_id", why)
    def test_stale_request_token_fails(self):
        (go, why), _ = self.gate({"run_id": "reg", "verdict": "OK"}, stale=("align_verdict",)); self.assertFalse(go)   # 옛 파일(토큰 불일치): 이전 OK가 게이트를 통과하지 못한다
    def test_missing_times_out_within_approved_limit(self):
        (go, why), el = self.gate(None); self.assertFalse(go); self.assertLessEqual(el, LH.ALIGN_TIMEOUT_S + 1e-6); self.assertIn("시간", why)
    def test_slow_response_over_limit_fails(self):
        (go, why), el = self.gate({"run_id": "reg", "verdict": "OK"}, delays={"align_verdict": 16.0}); self.assertFalse(go); self.assertLessEqual(el, LH.ALIGN_TIMEOUT_S + 1e-6)


@unittest.skipIf(vclock is None, "러너·생성기 의존성 없음")
class RunnerAlignIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        plan31 = json.load(open(T.FIX, encoding="utf-8")); cls.plan33, _ = T.PL.make_lift_plan(plan31, T.G, T.C)
        cls.td = tempfile.TemporaryDirectory(); cls.path = Path(cls.td.name) / "plan33.json"; cls.path.write_text(json.dumps(cls.plan33), encoding="utf-8")
    @classmethod
    def tearDownClass(cls): cls.td.cleanup()
    def setUp(self): self.old = (R.LIFT_CHANNEL, R.ask); self.asked = []
    def tearDown(self): R.LIFT_CHANNEL, R.ask = self.old
    def run33(self, resp):
        photos = dict(T.photos()); photos.pop("align_verdict")
        if resp is not None: photos["align_verdict"] = resp
        R.LIFT_CHANNEL = T.FakeChannel(T.REF, photos)
        def ask(msg, default=False): self.asked.append(msg); return True    # 사람이 미리 y를 쳐 둔 상황(입력 버퍼의 y)
        R.ask = ask
        return vclock.run_runner_fake(self.path, ("--fake-ceil", "6:146", "--fake-hyst", "1.9,0.27"))
    def align_questions(self): return [m for m in self.asked if QUESTION in m]
    def test_ng_is_n_without_asking_even_if_y_is_pre_typed(self):
        r = self.run33({"run_id": "reg", "verdict": "NG"}); self.assertEqual(self.align_questions(), [])
        runs = [p.get("run2", {}) for p in r.get("poses", [])]; self.assertFalse(any(q.get("align_confirmed") for q in runs))
        verdicts = [q["align_verdict"] for q in runs if "align_verdict" in q]; self.assertEqual(len(verdicts), 1); self.assertFalse(verdicts[0]["go"]); self.assertIn("NG", verdicts[0]["reason"])
        self.assertEqual(r["ended"], "aborted")   # 하강하지 않고 복귀(Abort)
    def test_missing_verdict_is_n_without_asking(self):
        r = self.run33(None); self.assertEqual(self.align_questions(), [])
    def test_other_run_verdict_is_n_without_asking(self):
        r = self.run33({"run_id": "yesterday", "verdict": "OK"}); self.assertEqual(self.align_questions(), [])
    def test_ok_keeps_the_human_question(self):
        r = self.run33({"run_id": "reg", "verdict": "OK"}); self.assertEqual(len(self.align_questions()), 1)


if __name__ == "__main__": unittest.main()
