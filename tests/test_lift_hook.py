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
    def __init__(self, ref=None, resp=None, delay=1.0, delays=None, stale=()):
        self.ref = ref; self.resp = resp or {}; self.delay = delay; self.delays = delays or {}; self.requests = []; self.stale = set(stale); self.tokens = {}
    def request(self, name, run_id, extra=""): self.requests.append(name); self.extras = getattr(self, "extras", {}); self.extras[name] = extra; self.tokens[name] = "%s-%s-tok" % (run_id, name); return self.tokens[name]
    def read_ref(self): return self.ref
    def wait(self, name, timeout_s, token):
        d = self.delays.get(name, self.delay); r = self.resp.get(name)
        if r is None or d > timeout_s: time.sleep(min(d, timeout_s)); return None
        time.sleep(d); return dict(r, request_id=("old-request" if name in self.stale else token))      # stale: 옛 파일이 남은 상황(요청 토큰 불일치)


DET = {"found": True, "center": [673.0, 267.0], "side_px": 29.0, "yaw_deg": -179.0}
REF = {"run_id": "reg", "started_epoch": 1.5, "elapsed_at_ref_s": 120.0, "maxsec": 1800, "detect": DET}


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
    def run33(self, ch, ceil="6:146", extra=(), hyst="1.9,0.27"):
        R.LIFT_CHANNEL = ch
        # Q3 UNSET은 --real만 막으므로 fake 모드로 실행. 기본은 서보 방향별 오차 모사(증가 방향 부족 1.9°, 감소 넘침 0.27°: 세트 16 대표값)
        return vclock.run_runner_fake(self.path, ("--fake-ceil", ceil, *(("--fake-hyst", hyst) if hyst else ()), *extra))
    def lift(self, r): return r["poses"][3]["run2"].get("lift")

    def test_happy_path_lifts_drops_and_returns_with_j6_closed(self):
        ch = FakeChannel(REF, photos()); r = self.run33(ch); L = self.lift(r)
        import re as _re
        self.assertRegex(ch.extras["A"], r"^ q=[-0-9.]+,[-0-9.]+,[-0-9.]+,[-0-9.]+$"); self.assertEqual(ch.extras["B"], ""); self.assertEqual(ch.extras["C"], "")   # P10 런타임 확인용 접촉 원시 q는 사진 A 요청에만 실린다
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
        for ref in (None, dict(REF, elapsed_at_ref_s=1600.0), dict(REF, detect=None), dict(REF, run_id="other-run")):
            r = self.run33(FakeChannel(ref, photos())); self.assertEqual(r["ended"], "aborted"); self.assertIn("사전 점검", r["abort_reason"]); self.assertNotIn("close_steps", r["poses"][3].get("run2", {}))

    def test_clock_stop_mid_lift_opens_and_returns_from_the_lifted_state(self):
        LP.PARAMS["clock_limit_s"] = 14.0                                                                       # 이동 도중 시계 규칙으로 중단되게 한도를 낮춘다(시험 전용)
        ch = FakeChannel(REF, photos()); r = self.run33(ch); L = self.lift(r)
        self.assertIn("시계 규칙", L["ended_reason"]); self.assertGreaterEqual(len(L["moves"]), 1); self.assertLess(len(L["moves"]), 3)
        self.assertEqual(r["ended"], "completed"); self.assertTrue(r["returned_to_start"]); self.assertIsNotNone(L["open_readback"])         # 들어올린 채 열고, 들어올린 상태 복귀 경로로 K1 복귀

    def test_photo_b_from_another_pa_start_skips_hold_and_c_but_still_drops(self):
        ch = FakeChannel(REF, photos(epoch_b=9.9)); r = self.run33(ch); L = self.lift(r)
        self.assertFalse(L["hold_done"]); self.assertEqual(ch.requests, ["A", "B"]); self.assertEqual(r["ended"], "completed"); self.assertIsNotNone(L["open_readback"])

    def test_stale_photo_results_are_ignored(self):
        for names in (("A",), ("B",)):
            ch = FakeChannel(REF, photos(), stale=names); r = self.run33(ch); L = self.lift(r)
            self.assertEqual(r["ended"], "completed"); self.assertIsNone(L["photos"][names[0]])
            if names == ("A",): self.assertEqual(L["moves"], [])                       # 옛 gate.go로 들어올리지 않는다
            else: self.assertFalse(L["hold_done"]); self.assertEqual(ch.requests, ["A", "B"])   # 옛 B를 '확인됨'으로 받지 않는다

    def test_photo_and_move_records_for_the_dry_run_measurements(self):
        r = self.run33(FakeChannel(REF, photos())); L = self.lift(r)
        for n in "ABC": self.assertEqual(set(L["photo_times"][n]), {"req_mono", "rx_mono", "wait_s", "req_epoch", "rx_epoch", "request_id"}); self.assertTrue(L["photo_times"][n]["request_id"].startswith("reg-%s-" % n))
        for m in L["moves"]: self.assertIn("series", m); self.assertIn("reach_within_tol_ms", m)       # 이동당 시간 원자료(R11)

    def test_second_j2_below_min_effective_step_is_skipped_not_sent(self):
        r = self.run33(FakeChannel(REF, photos()), hyst=None); L = self.lift(r)                  # 이상적 서보: 첫 이동이 74에 정확히 도착 → 두 번째 요청 폭 3.0° < 3.5°
        self.assertEqual([(m["joint"], m["target"]) for m in L["moves"]], [(2, 74), (4, 27)]); self.assertIn("최소 유효 폭", L["second_j2"]["skipped_reason"])
        self.assertEqual(r["ended"], "completed"); self.assertIsNotNone(L["open_readback"])

    def test_second_j2_with_no_response_aborts_via_arrival_check_when_readback_is_low(self):
        orig = R.FakeArmS.Arm_serial_servo_write
        def drop77(self_, id_, angle, time_ms):                                                    # 두 번째 J2 쓰기(77)만 서보가 무시하는 상황
            if id_ == 2 and angle == 77 and R.P2K.get("contact"): self_.writes.append((id_, angle, time_ms)); return   # 접촉(누름) 중의 들어올림 쓰기만
            return orig(self_, id_, angle, time_ms)
        R.FakeArmS.Arm_serial_servo_write = drop77
        try: r = self.run33(FakeChannel(REF, photos())); L = self.lift(r)                          # 부족 1.9°로 첫 이동 뒤 readback 73 → 두 번째(77) 무반응 → |77−73| = 4 > 3
        finally: R.FakeArmS.Arm_serial_servo_write = orig
        self.assertEqual(r["ended"], "aborted"); self.assertIn("도착 실패", r["abort_reason"]); self.assertTrue(L["moves"][-1].get("aborted"))
        # 비상 경로(R12·§3 필수 시험): J6는 접촉 어댑터로 정확히 한 번만 열리고, 들어올린 상태의 복귀 항목으로 K1까지 돌아간다
        self.assertTrue(r["emergency_open"]); opens = [w for w in r["j6_adapter_logs"]["contact"] if "return_to_start" in w]; self.assertEqual(len(opens), 1)
        self.assertTrue(r["returned_to_start"], r.get("return_abort")); self.assertEqual(L["abort_class"], "기타 중단")
        contact_writes = [w[1] for w in r["j6_adapter_logs"]["contact"] if w[0] == "write" and "return_to_start" not in w]
        self.assertEqual(contact_writes, [120, 130, 135, 140, 145, 150])                    # 접촉 로그에는 닫기 계단만(+ return_to_start 1회, 위에서 확인): K1 도착 전 J6 닫기 없음
        closes = [w for w in r["j6_adapter_logs"].get("final", []) if w[0] == "write"]       # 178 닫기는 복귀가 끝난 뒤에만(returned_to_start가 True일 때만 존재)
        self.assertTrue(all(w[1] in (140, 178) for w in closes)); self.assertEqual(L["return_index_used"]["key"].split(":")[0], "before_j4")
        self.assertGreater(len(r["return_steps"]), 0)

    def test_first_j2_move_without_response_returns_via_the_contact_entry(self):
        orig = R.FakeArmS.Arm_serial_servo_write
        def drop74(self_, id_, angle, time_ms):                                                    # 누름 중 첫 J2 쓰기(74)와 재전송을 서보가 모두 무시
            if id_ == 2 and angle == 74 and R.P2K.get("contact"): self_.writes.append((id_, angle, time_ms)); return
            return orig(self_, id_, angle, time_ms)
        R.FakeArmS.Arm_serial_servo_write = drop74
        try: r = self.run33(FakeChannel(REF, photos())); L = self.lift(r)
        finally: R.FakeArmS.Arm_serial_servo_write = orig
        self.assertEqual(r["ended"], "aborted"); self.assertTrue("무반응" in r["abort_reason"] or "도착 실패" in r["abort_reason"], r["abort_reason"])
        self.assertEqual(L["return_index_used"]["index"], self.plan33["lift"]["contact_moves_done"]); self.assertEqual(L["return_index_used"]["key"], "contact")   # 들어올린 상태 경로가 아니라 접촉 항목
        self.assertTrue(r["emergency_open"]); self.assertEqual(len([w for w in r["j6_adapter_logs"]["contact"] if "return_to_start" in w]), 1)
        self.assertTrue(r["returned_to_start"], r.get("return_abort"))

    def test_classify_abort_labels_j6_slip(self):
        self.assertEqual(LH.classify_abort(Exception("J6 회독 150가 시작값 146에서 3° 넘게 벗어났다. 추가 이동 없음.")), "E3 미끄러짐 의심(J6 이탈)")
        self.assertEqual(LH.classify_abort(Exception("J2 도착 실패")), "기타 중단")

    def test_no_stall_at_150_means_no_lift(self):
        r = self.run33(FakeChannel(REF, photos()), ceil="6:179"); rr = r["poses"][3]["run2"]
        self.assertNotIn("lift", rr); self.assertEqual(r["ended"], "completed")

    def test_stall_below_150_means_no_lift(self):
        r = self.run33(FakeChannel(REF, photos()), ceil="6:140"); rr = r["poses"][3]["run2"]
        self.assertNotIn("lift", rr); self.assertIsNotNone(rr["stall_command"]); self.assertLess(rr["stall_command"], 150)


@unittest.skipIf(vclock is None, "러너·생성기 의존성 없음")
class FilePhotoChannelTest(unittest.TestCase):
    def setUp(self):
        self.td = tempfile.TemporaryDirectory(); self.ch = LH.FilePhotoChannel(lambda *a, **k: None, folder=self.td.name)
    def tearDown(self): self.td.cleanup()
    def path(self, n): return os.path.join(self.td.name, "lift_photo_%s.json" % n)

    def test_request_deletes_old_result_and_wait_accepts_only_matching_token(self):
        json.dump({"request_id": "old", "gate": {"go": True}}, open(self.path("A"), "w"))
        tok = self.ch.request("A", "run1"); self.assertFalse(os.path.exists(self.path("A")))             # 요청 직전에 옛 파일 삭제
        self.assertTrue(tok.startswith("run1-A-"))
        with vclock.virtual_clock():
            json.dump({"request_id": "other", "gate": {"go": True}}, open(self.path("A"), "w"))
            self.assertIsNone(self.ch.wait("A", 3.0, tok))                                                # 토큰이 다른 파일은 시간 초과
            json.dump({"request_id": tok, "gate": {"go": False}}, open(self.path("A"), "w"))
            self.assertEqual(self.ch.wait("A", 3.0, tok)["gate"], {"go": False})                          # 맞는 토큰이면 내용을 돌려준다

    def test_partial_file_is_retried_and_missing_ref_is_none(self):
        tok = self.ch.request("B", "run1")
        with vclock.virtual_clock():
            open(self.path("B"), "w").write("{\"request_id\": ")                                          # 쓰는 중인 파일
            self.assertIsNone(self.ch.wait("B", 1.0, tok))
        self.assertIsNone(self.ch.read_ref()); json.dump({"run_id": "run1"}, open(self.path("ref"), "w")); self.assertEqual(self.ch.read_ref(), {"run_id": "run1"})

    def test_tokens_are_unique_per_request(self):
        self.assertNotEqual(self.ch.request("C", "r"), self.ch.request("C", "r"))


if __name__ == "__main__": unittest.main()
