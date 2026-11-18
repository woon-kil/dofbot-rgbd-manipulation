"""세트 33 러너 훅 시험: 세트 31 계획 → 세트 33 계획(g01_s42_plan_lift) → 가짜 팔·가상 시계·가짜 사진 채널로 g01_p2k_test.main() 실행. J6 정체는 --fake-ceil 6:146(명령 150에서 146)."""
import copy, json, os, sys, tempfile, time, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "scripts"), str(ROOT / "tests")]
try:
    import vclock, g01_p2k_test as R, g01_lift_hook as LH, g01_lift_params as LP, g01_s42_plan_lift as PL
    cwd = os.getcwd(); os.chdir(ROOT); G, C = PL._load_modules(); os.chdir(cwd)
except Exception as e:
    vclock = None
FIX = ROOT / "tests" / "fixtures" / "s4_2_plan_31_fixture.json"


class FakeChannel(LH.PhotoChannel if vclock else object):
    def __init__(self, ref=None, resp=None, delay=1.0, delays=None):
        self.ref = ref; self.resp = resp or {}; self.delay = delay; self.delays = delays or {}; self.requests = []
    def request(self, name): self.requests.append(name)
    def read_ref(self): return self.ref
    def wait(self, name, timeout_s):
        d = self.delays.get(name, self.delay); r = self.resp.get(name)
        if r is None or d > timeout_s: time.sleep(min(d, timeout_s)); return None
        time.sleep(d); return r


DET = {"found": True, "center": [673.0, 267.0], "side_px": 29.0, "yaw_deg": -179.0}
REF = {"started_epoch": 1.5, "elapsed_at_ref_s": 120.0, "maxsec": 1800, "detect": DET}


def photos(go=True, epoch_b=1.5, epoch_c=1.5, epoch_a=1.5):
    return {"A": {"started_epoch": epoch_a, "detect": DET, "gate": {"go": go}}, "B": {"started_epoch": epoch_b, "detect": DET}, "C": {"started_epoch": epoch_c, "detect": DET}}


@unittest.skipIf(vclock is None, "러너·생성기 의존성 없음")
class LiftHookTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        plan31 = json.load(open(FIX, encoding="utf-8")); cls.plan33, _ = PL.make_lift_plan(plan31, G, C)
        cls.td = tempfile.TemporaryDirectory(); cls.path = Path(cls.td.name) / "plan33.json"; cls.path.write_text(json.dumps(cls.plan33), encoding="utf-8")
    @classmethod
    def tearDownClass(cls): cls.td.cleanup()
    def setUp(self):
        self.old_ch, self.old_limit = R.LIFT_CHANNEL, LP.PARAMS["clock_limit_s"]
    def tearDown(self):
        R.LIFT_CHANNEL = self.old_ch; LP.PARAMS["clock_limit_s"] = self.old_limit
    def run33(self, ch, ceil="6:146", extra=()):
        R.LIFT_CHANNEL = ch
        # 시험에서는 Q3 UNSET이 --real만 막으므로 fake 모드로 실행한다
        return vclock.run_runner_fake(self.path, ("--fake-ceil", ceil, *extra))
    def lift(self, r): return r["poses"][3]["run2"].get("lift")

    def test_happy_path_lifts_drops_and_returns_with_j6_closed(self):
        ch = FakeChannel(REF, photos()); r = self.run33(ch); L = self.lift(r)
        self.assertEqual(ch.requests, ["A", "B", "C"]); self.assertEqual(r["ended"], "completed"); self.assertTrue(r["returned_to_start"])
        self.assertEqual([(m["joint"], m["target"]) for m in L["moves"]], [(2, 74), (2, 77), (4, 27)])        # 첫 J2 74 → 최대 목표 77(P10) → J4 −5
        self.assertTrue(L["hold_done"] and L["photo_c_done"]); self.assertEqual(L["ended_reason"], "완료"); self.assertLessEqual(L["elapsed_at_open_s"], 26.8)
        self.assertIsNotNone(L["open_readback"]); self.assertEqual(r["poses"][3]["run2"]["stall_command"], 150); self.assertIsNotNone(r.get("j6_k1_close_readback"))
        self.assertEqual(len(r["steps"]), self.plan33["lift"]["contact_moves_done"])                           # 남은 계획 이동은 건너뜀(상승 자세 없음)
        self.assertEqual(L["gate"], {"go": True}); self.assertAlmostEqual(L["j6_after_close"], 146)

    def test_no_go_does_not_lift_and_uses_the_existing_release_path(self):
        ch = FakeChannel(REF, photos(go=False)); r = self.run33(ch); rr = r["poses"][3]["run2"]
        self.assertEqual(ch.requests, ["A"]); self.assertIsNone(rr["lift"].get("second_j2")); self.assertEqual(rr["lift"]["moves"], []); self.assertIsNotNone(rr.get("release_readback"))
        self.assertEqual(r["ended"], "completed"); self.assertIn("no-go", rr["lift"]["ended_reason"])

    def test_photo_a_timeout_or_other_pa_start_means_no_lift(self):
        for ch in (FakeChannel(REF, photos(), delays={"A": 3.5}), FakeChannel(REF, photos(epoch_a=2.5))):
            r = self.run33(ch); self.assertEqual(self.lift(r)["moves"], [], ch.delays); self.assertEqual(r["ended"], "completed")

    def test_precheck_failure_never_closes_the_gripper(self):
        for ref in (None, dict(REF, elapsed_at_ref_s=1600.0), dict(REF, detect=None)):
            r = self.run33(FakeChannel(ref, photos())); self.assertEqual(r["ended"], "aborted"); self.assertIn("사전 점검", r["abort_reason"]); self.assertNotIn("close_steps", r["poses"][3].get("run2", {}))

    def test_clock_stop_mid_lift_opens_and_returns_from_the_lifted_state(self):
        LP.PARAMS["clock_limit_s"] = 14.0                                                                       # 이동 도중 시계 규칙으로 중단되게 한도를 낮춘다(시험 전용)
        ch = FakeChannel(REF, photos()); r = self.run33(ch); L = self.lift(r)
        self.assertIn("시계 규칙", L["ended_reason"]); self.assertGreaterEqual(len(L["moves"]), 1); self.assertLess(len(L["moves"]), 3)
        self.assertEqual(r["ended"], "completed"); self.assertTrue(r["returned_to_start"]); self.assertIsNotNone(L["open_readback"])         # 들어올린 채 열고, 들어올린 상태 복귀 경로로 K1 복귀

    def test_photo_b_from_another_pa_start_skips_hold_and_c_but_still_drops(self):
        ch = FakeChannel(REF, photos(epoch_b=9.9)); r = self.run33(ch); L = self.lift(r)
        self.assertFalse(L["hold_done"]); self.assertEqual(ch.requests, ["A", "B"]); self.assertEqual(r["ended"], "completed"); self.assertIsNotNone(L["open_readback"])

    def test_no_stall_at_150_means_no_lift(self):
        r = self.run33(FakeChannel(REF, photos()), ceil="6:179"); rr = r["poses"][3]["run2"]
        self.assertNotIn("lift", rr); self.assertEqual(r["ended"], "completed")

    def test_stall_below_150_means_no_lift(self):
        r = self.run33(FakeChannel(REF, photos()), ceil="6:140"); rr = r["poses"][3]["run2"]
        self.assertNotIn("lift", rr); self.assertIsNotNone(rr["stall_command"]); self.assertLess(rr["stall_command"], 150)


if __name__ == "__main__": unittest.main()
