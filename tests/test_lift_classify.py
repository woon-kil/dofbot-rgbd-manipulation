"""세트 33 시도 분류기(g01_lift_classify) 시험: 접촉 어댑터 쓰기 로그가 단일 출처. 합성 결과와, 실제 가짜 실행을 mode만 real로 바꾼 결과(145 쓰기 직후 중단 포함)."""
import copy, json, sys, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "scripts"), str(ROOT / "tests")]
import g01_lift_classify as K

W = lambda t: ["write", t, 1500, "real"]
LOG150 = [["read", 100], W(120), W(130), W(135), W(140), W(145), W(150), ["write", 100, 3000, "real", "return_to_start"]]


def res(log, run2=None, mode="real"): return {"mode": mode, "j6_adapter_logs": {"contact": log}, "poses": [{"pose_id": 3104, "run2": run2 or {}}]}


class ClassifyTest(unittest.TestCase):
    def test_fake_and_dry_run_never_count_even_with_real_labelled_logs(self):
        for m in ("fake", "dry_run"): c = K.classify(res(LOG150, {"close_steps": []}, mode=m)); self.assertFalse(c["counted"]); self.assertIn("mode=" + m, c["why"])

    def test_no_close_write_is_not_an_attempt(self):
        c = K.classify(res([["read", 100]])); self.assertFalse(c["counted"]); self.assertEqual(c["attempt"], "시도 아님")
        c = K.classify(res([["read", 100], ["write", 100, 3000, "real", "return_to_start"]])); self.assertFalse(c["counted"])             # 열기만 있고 닫기 쓰기 없음

    def test_stall_before_145_is_an_attempt(self):
        c = K.classify(res([W(120), W(130), W(135), W(140)], {"close_steps": [{"target": 140, "stall": True}], "stall_readback": 137, "stall_command": 140}))
        self.assertTrue(c["counted"]); self.assertEqual(c["attempt"], "시도(정체)"); self.assertFalse(c["wrote_145"]); self.assertEqual(c["code"], "STALL_140")

    def test_abort_right_after_145_write_loses_close_steps_but_still_counts(self):
        c = K.classify(res([W(120), W(130), W(135), W(140), W(145)]))                                                                    # close_steps 없음(루프 중 예외)
        self.assertTrue(c["counted"]); self.assertEqual(c["attempt"], "시도(접촉 미확정)"); self.assertTrue(c["wrote_145"]); self.assertEqual(c["outcome"], "no_lift"); self.assertEqual(c["code"], "ABORT")

    def test_no_stall_normal_close_is_counted_as_extension(self):
        c = K.classify(res(LOG150, {"close_steps": [{"target": 150}]})); self.assertEqual(c["attempt"], "시도(정체 없음)"); self.assertEqual(c["code"], "NO_STALL_150")

    def test_stall_with_gate_nogo_lift_and_sag_cases(self):
        base = {"close_steps": [{"target": 150, "stall": True}], "stall_readback": 146, "stall_command": 150}
        c = K.classify(res(LOG150, dict(base, lift={"code": "G2", "moves": []}))); self.assertEqual((c["outcome"], c["code"], c["lifted_once"]), ("no_lift", "G2", False))
        c = K.classify(res(LOG150, dict(base, lift={"code": "P10", "moves": []}))); self.assertEqual(c["code"], "P10")
        c = K.classify(res(LOG150, dict(base, lift={"code": "SAG_LATE", "lifted_once": True, "moves": [{"step": "lift1"}]}))); self.assertEqual((c["outcome"], c["code"], c["lifted_once"]), ("no_lift", "SAG_LATE", True))
        c = K.classify(res(LOG150, dict(base, lift={"moves": [{"step": "lift1"}, {"step": "lift2"}, {"step": "lift3"}], "ended_reason": "완료"}))); self.assertEqual(c["outcome"], "lifted")
        c = K.classify(res(LOG150, dict(base, lift={"moves": [{"step": "lift1", "aborted": True}]}))); self.assertEqual((c["outcome"], c["code"]), ("no_lift", "ABORT"))

    def test_cap_reduced_attempt_is_labelled_not_counted_as_grasp_failure_evidence(self):
        base = {"close_steps": [{"target": 150, "stall": True}], "stall_readback": 146, "stall_command": 150}
        c = K.classify(res(LOG150, dict(base, lift={"moves": [{"step": "lift1"}, {"step": "lift2"}, {"step": "lift3"}], "sag": {"late": {"cap": 76, "plan_max": 77}}})))
        self.assertEqual(c["outcome"], "lifted"); self.assertIn("E1 신호 약함 가능", c["note"]); self.assertIn("30회 0건 집계에 넣지 않는다", c["note"])
        c = K.classify(res(LOG150, dict(base, lift={"moves": [{"step": "lift3"}], "sag": {"late": {"cap": 77, "plan_max": 77}}}))); self.assertNotIn("note", c)

    def test_judgement_count_is_left_to_r8(self):
        self.assertIn("R8", K.classify(res(LOG150, {"close_steps": []}))["judgement_record"])


try:
    import vclock, g01_p2k_test as R, g01_s42_plan_lift as PL
except Exception: vclock = None


@unittest.skipIf(vclock is None, "러너 의존성 없음")
class RealFakeRunTest(unittest.TestCase):
    """실제 가짜 실행 결과의 쓰기 로그로 분류(모드만 real로 바꿔 데이터 경로를 확인). 145 쓰기 직후 중단해도 로그가 남아 시도로 센다."""
    @classmethod
    def setUpClass(cls):
        import os, tempfile
        try: cwd = os.getcwd(); os.chdir(ROOT); G, C = PL._load_modules(); os.chdir(cwd)
        except ImportError as e: raise unittest.SkipTest("venv 전용(cv2·scipy): %r" % e)
        plan31 = json.load(open(ROOT / "tests" / "fixtures" / "s4_2_plan_31_fixture.json", encoding="utf-8")); plan33, _ = PL.make_lift_plan(plan31, G, C)
        cls.td = tempfile.TemporaryDirectory(); cls.path = Path(cls.td.name) / "p.json"; cls.path.write_text(json.dumps(plan33), encoding="utf-8")
    @classmethod
    def tearDownClass(cls): cls.td.cleanup()

    def test_precheck_abort_in_a_real_fake_run_is_not_an_attempt(self):
        R.LIFT_CHANNEL = None                                                                                      # 기준 사진 결과 없음 → 닫기 전 사전 점검 실패
        r = copy.deepcopy(vclock.run_runner_fake(self.path, ("--fake-ceil", "6:146"))); r["mode"] = "real"; c = K.classify(r)
        self.assertFalse(c["counted"]); self.assertEqual(c["attempt"], "시도 아님")

    def test_abort_after_145_write_in_a_real_fake_run_counts_as_attempt(self):
        import test_lift_hook as TH
        R.LIFT_CHANNEL = TH.FakeChannel(TH.REF, TH.photos())
        try: r = vclock.run_runner_fake(self.path, ("--fake-ceil", "6:146", "--fake-abort-close", "5"))               # 닫기 5번째(145) 쓰기·폴링 직후 중단 주입
        finally: R.LIFT_CHANNEL = None
        r = copy.deepcopy(r); r["mode"] = "real"; c = K.classify(r)
        self.assertTrue(c["counted"]); self.assertTrue(c["wrote_145"]); self.assertEqual(c["attempt"], "시도(접촉 미확정)"); self.assertNotIn("close_steps", r["poses"][3]["run2"])   # close_steps는 잃어도 쓰기 로그로 센다


if __name__ == "__main__": unittest.main()
