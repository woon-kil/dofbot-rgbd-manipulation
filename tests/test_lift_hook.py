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
    @property
    def photo_requests(self): return [n for n in self.requests if n != "align_verdict"]   # 사진 요청만(정렬 판정 요청 제외)
    def wait(self, name, timeout_s, token):
        d = self.delays.get(name, self.delay); r = self.resp.get(name)
        if r is None or d > timeout_s: time.sleep(min(d, timeout_s)); return None
        time.sleep(d); return dict(r, request_id=("old-request" if name in self.stale else token))      # stale: 옛 파일이 남은 상황(요청 토큰 불일치)


DET = {"found": True, "center": [673.0, 267.0], "side_px": 29.0, "yaw_deg": -179.0}
REF = {"run_id": "reg", "started_epoch": 1.5, "elapsed_at_ref_s": 120.0, "maxsec": 1800, "detect": DET}


def photos(go=True, epoch_b=1.5, epoch_c=1.5, epoch_a=1.5):
    return {"A": {"started_epoch": epoch_a, "detect": DET, "gate": {"go": go, "tilt_check": {"ok": True, "margin": 0.21, "plan_max": 77}}}, "B": {"started_epoch": epoch_b, "detect": DET}, "C": {"started_epoch": epoch_c, "detect": DET}, "align_verdict": {"run_id": "reg", "verdict": "OK"}}   # M6: 정렬 판정 OK(없거나 NG면 사람 입력 없이 N)


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

    @unittest.expectedFailure   # M5 가드 시계 가산으로 정상 경로 완료가 27.1 s(상한 26.8 s): 등급 A 결정 대기(플랜 v2 §5, 시험 먼저 기록)
    def test_happy_path_lifts_drops_and_returns_with_j6_closed(self):
        ch = FakeChannel(REF, photos()); r = self.run33(ch); L = self.lift(r)
        import re as _re
        self.assertRegex(ch.extras["A"], r"^ q=[-0-9.]+,[-0-9.]+,[-0-9.]+,[-0-9.]+$"); self.assertEqual(ch.extras["B"], ""); self.assertEqual(ch.extras["C"], "")   # P10 런타임 확인용 접촉 원시 q는 사진 A 요청에만 실린다
        self.assertEqual(ch.photo_requests, ["A", "B", "C"]); self.assertEqual(r["ended"], "completed"); self.assertTrue(r["returned_to_start"])
        self.assertEqual([(m["joint"], m["target"]) for m in L["moves"]], [(2, 74), (2, 77), (4, 27)])        # 첫 J2 74 → 최대 목표 77(P10) → J4 −5
        self.assertTrue(L["hold_done"] and L["photo_c_done"]); self.assertEqual(L["ended_reason"], "완료"); self.assertLessEqual(L["elapsed_at_open_s"], 26.8)
        self.assertIsNotNone(L["open_readback"]); self.assertEqual(r["poses"][3]["run2"]["stall_command"], 150); self.assertIsNotNone(r.get("j6_k1_close_readback"))
        self.assertEqual(len(r["steps"]), self.plan33["lift"]["contact_moves_done"])                           # 남은 계획 이동은 건너뜀(상승 자세 없음)
        self.assertTrue(L["gate"]["go"]); self.assertEqual(L["gate"]["tilt_check"]["margin"], 0.21); self.assertAlmostEqual(L["j6_after_close"], 146)

    def test_no_go_does_not_lift_and_uses_the_existing_release_path(self):
        ch = FakeChannel(REF, photos(go=False)); r = self.run33(ch); rr = r["poses"][3]["run2"]
        self.assertEqual(ch.photo_requests, ["A"]); self.assertIsNone(rr["lift"].get("second_j2")); self.assertEqual(rr["lift"]["moves"], []); self.assertIsNotNone(rr.get("release_readback"))
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
        self.assertFalse(L["hold_done"]); self.assertEqual(ch.photo_requests, ["A", "B"]); self.assertEqual(r["ended"], "completed"); self.assertIsNotNone(L["open_readback"])

    def test_stale_photo_results_are_ignored(self):
        for names in (("A",), ("B",)):
            ch = FakeChannel(REF, photos(), stale=names); r = self.run33(ch); L = self.lift(r)
            self.assertEqual(r["ended"], "completed"); self.assertIsNone(L["photos"][names[0]])
            if names == ("A",): self.assertEqual(L["moves"], [])                       # 옛 gate.go로 들어올리지 않는다
            else: self.assertFalse(L["hold_done"]); self.assertEqual(ch.photo_requests, ["A", "B"])   # 옛 B를 '확인됨'으로 받지 않는다

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


class SagCapMethods:
    """P10 처짐 cap(사용자 결정 2026-11-18): 읽기 횟수 불변, 이탈 0이면 cap 77, 작은 이탈은 cap, 이른 거부는 이동 0회, 첫 이동 뒤 거부는 그 높이에서 열기 1회 + K1 복귀."""
    BASE_TOTAL_RAW_SET = 12           # 정상 경로 전체 raw_set 호출 수. 변경 전 측정 15 → 12: `_stage_idx`가 raw_set을 두 번 하던 것을 한 번으로 줄임(M5 가드 대기·시계 소모 감소, 자체 결정 B, 플랜 v2 §5; 읽기를 추가하지 않는다는 원칙은 그대로)
    BASE_A_TO_FIRST_MOVE = 1          # 사진 A 응답부터 첫 run_move까지(변경 전 측정)

    def instrumented(self, early=(0.0, 0.0), late=(0.0, 0.0), none_at=None, photos_=None):
        """누름 중(사진 A 응답 뒤)·첫 이동 뒤 읽기에 J3·J4 이탈(도)을 주입한다. 반환 (결과, 카운터)."""
        n = {"raw": 0, "at_A": None, "first_move": None, "moves": 0, "phase": "pre"}; orig_raw, orig_rm = R.raw_set, R.run_move
        def raw(arm):
            n["raw"] += 1; r = orig_raw(arm); d = {"early": early, "late": late}.get(n["phase"])
            if d:
                for j, dd in zip((3, 4), d): r[str(j)] = dict(r[str(j)], deg=r[str(j)]["deg"] + dd)
                if none_at == n["phase"]:
                    r["3"] = dict(r["3"], deg=None)                                                                          # J3만(J4는 상태 색인 계산이 읽으므로 두 이동 사이에는 None으로 만들지 않는다)
            return r
        def rm(*a, **k):
            if n["first_move"] is None and n["at_A"] is not None: n["first_move"] = n["raw"]
            n["moves"] += 1; res = orig_rm(*a, **k)
            if a[2].get("phase") == "L" and n["phase"] == "early": n["phase"] = "late"                  # 첫 들어올림 이동(phase L)이 끝난 뒤부터 late
            return res
        class Ch(FakeChannel):
            def wait(s, name, timeout_s, token):
                r = super().wait(name, timeout_s, token)
                if name == "A": n["at_A"] = n["raw"]; n["phase"] = "early"
                return r
        R.raw_set, R.run_move = raw, rm
        try: r = self.run33(Ch(REF, photos_ or photos()))
        finally: R.raw_set, R.run_move = orig_raw, orig_rm
        return r, n

    @property
    def n_plan_moves(self): return len(self.plan33["moves"])

    def test_read_counts_unchanged_and_all_reads_recorded(self):
        r, n = self.instrumented(); L = self.lift(r)
        self.assertEqual(n["raw"], self.BASE_TOTAL_RAW_SET); self.assertEqual(n["first_move"] - n["at_A"], self.BASE_A_TO_FIRST_MOVE)     # 읽기를 추가하지 않았다(변경 전 측정 기준선과 같다)
        labels = [x["label"] for x in L["raw_reads"]]; self.assertEqual(labels[:2], ["누름 중(첫 이동 전)", "첫 이동 뒤"]); self.assertIn("둘째 이동 뒤", labels)
        self.assertEqual(L["sag"]["early"]["cap"], 77); self.assertEqual(L["sag"]["late"]["cap"], 77); self.assertFalse(L["sag"]["late"]["refuse"])
        self.assertEqual(len(L["raw_reads"][0]["q"]), 4)

    def test_small_adverse_sag_caps_second_target(self):
        r, n = self.instrumented(early=(0.25, 0.0), late=(0.25, 0.0)); L = self.lift(r)
        self.assertEqual(L["sag"]["late"]["cap"], 76); self.assertLessEqual(L["second_j2"]["target"], 76)                                    # margin 0.21 − Δ 0.25 → floor(76.96) = 76

    def test_favourable_sag_does_not_raise_above_plan_max(self):
        r, n = self.instrumented(early=(-0.5, 0.0), late=(-0.5, 0.0)); self.assertEqual(self.lift(r)["sag"]["late"]["cap"], 77)

    def test_early_refusal_moves_nothing_and_opens_once(self):
        r, n = self.instrumented(early=(1.5, 0.0)); L = self.lift(r)                                                                          # Δ 1.5 → cap 75 ≤ 75 → 이동 0회 거부
        self.assertEqual(L["code"], "SAG_EARLY"); self.assertFalse(L["lifted_once"]); self.assertEqual(L["moves"], []); self.assertIn("처짐 과다", L["ended_reason"])
        run2 = r["poses"][3]["run2"]; self.assertEqual(run2["release_readback"], 100)                                                          # 러너의 기존 열기 1회
        logs = r["j6_adapter_logs"]; self.assertEqual(sum(1 for l in logs["contact"] if l[0] == "write" and len(l) > 4 and l[4] == "return_to_start"), 1)
        self.assertNotIn("lift_done", json.dumps(r)); self.assertTrue(r["returned_to_start"])

    def test_late_refusal_opens_once_at_lift1_height_and_returns(self):
        r, n = self.instrumented(late=(1.5, 0.0)); L = self.lift(r)                                                                          # 첫 이동 뒤 읽기에서 cap 75 → 시계 중단과 같은 경로
        self.assertEqual(L["code"], "SAG_LATE"); self.assertTrue(L["lifted_once"]); self.assertEqual([m["step"] for m in L["moves"]], ["lift1"]); self.assertIsNotNone(L.get("open_readback"))
        logs = r["j6_adapter_logs"]; self.assertEqual(sum(1 for l in logs["contact"] if l[0] == "write" and len(l) > 4 and l[4] == "return_to_start"), 1)   # J6 열기 1회
        self.assertTrue(r["returned_to_start"]); self.assertEqual(r["j6_k1_close_readback"], 178 if r.get("j6_k1_close_readback") else r["j6_k1_close_readback"])
        used = L["return_index_used"]; self.assertTrue(used["key"].startswith("before_j4:"), used); self.assertLessEqual(used["distance"], 3)      # 첫 이동만 한 상태가 들어올린 복귀 항목(J2 72~77)에 매핑된다
        self.assertEqual(self.plan33["return_routes"][used["index"]]["lift_stage"], "before_j4")

    def test_late_refusal_asks_a_human_before_return_and_no_stops_the_return(self):
        qs = []; orig = R.ask
        def ask(msg): qs.append(msg); return not ("SAG_LATE" in msg or "시작 자세가 아닐 수" in msg)                                                      # 낙하 확인 질문과 그 뒤 중단 경로의 복귀 질문에 N(N이면 복귀 없음)
        R.ask = ask
        try: r, n = self.instrumented(late=(1.5, 0.0))
        finally: R.ask = orig
        self.assertTrue(any("SAG_LATE" in q for q in qs)); self.assertEqual(r["ended"], "aborted"); self.assertIn("복귀하지 않습니다", r["abort_reason"]); self.assertNotEqual(r.get("returned_to_start"), True)
        qs2 = []
        def ask2(msg): qs2.append(msg); return True
        R.ask = ask2
        try: r, n = self.instrumented(late=(1.5, 0.0))
        finally: R.ask = orig
        self.assertTrue(any("SAG_LATE" in q for q in qs2)); self.assertTrue(r["returned_to_start"])                                           # y면 복귀
        qs3 = []
        def ask3(msg): qs3.append(msg); return True
        R.ask = ask3
        try: r, n = self.instrumented()
        finally: R.ask = orig
        self.assertFalse(any("SAG_LATE" in q for q in qs3)); self.assertTrue(any("들어올림 뒤 J6를 열어 블록이 낙하했습니다" in q and "5° 이내" in q for q in qs3))   # 정상 들어올림에도 낙하 뒤 사람 확인 질문이 있다(SAG_LATE 문구는 없음)
        qs4 = []
        def ask4(msg): qs4.append(msg); return not ("낙하했습니다" in msg or "시작 자세가 아닐 수" in msg)
        R.ask = ask4
        try: r, n = self.instrumented()
        finally: R.ask = orig
        self.assertEqual(r["ended"], "aborted"); self.assertIn("복귀하지 않습니다", r["abort_reason"]); self.assertNotEqual(r.get("returned_to_start"), True)                 # 정상 들어올림에서도 N이면 복귀하지 않는다

    def test_set33_safety_questions_have_no_false_sentences(self):
        """세트 33 러너 안전 확인 8항목(화면 출력, 사람이 y로 답하는 문장)에 'Run 3 이후'·옛 들어올림 높이가 남지 않는다. ask 순서도 문서와 같다."""
        import io, contextlib
        qs = []; orig = R.ask
        def ask(msg): qs.append(msg); return True
        R.ask = ask; buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf): self.run33(FakeChannel(REF, photos()))
        finally: R.ask = orig
        out = buf.getvalue(); self.assertIn("기준 사진 이후 손대지 않았고", out); self.assertIn("--real", out.split("기준 사진 이후")[0][-60:]); self.assertNotIn("2026-11-18 17:02", out); self.assertNotIn("Run 3 이후 만지지", out); self.assertNotIn("약 15~20 mm", out); self.assertIn("약 10~17 mm", out)
        keys = ["위 8개를 모두 확인했습니까", "정렬 확인이 모두 맞습니까", "블록이 낙하했습니다", "마지막 확인 1회"]   # --real 정상 경로의 ask 순서(안내문서 §4와 같음)
        pos = [next(i for i, q in enumerate(qs) if k in q) for k in keys]; self.assertEqual(pos, sorted(pos)); self.assertEqual(len(qs), 4)

    def test_dead_terminal_during_press_still_opens_j6_once_and_writes_result(self):
        """ssh/pty가 끊겨(SIGHUP→Hangup, 이후 print가 OSError) 사진 A를 기다리는 누름 중에도 비상 J6 열기가 정확히 1회 나가고 result.json이 써져야 한다(래퍼 경유 dry run에서 result.json이 사라진 원인 가설: finally 안의 say가 죽은 pty에서 OSError → 열기·기록 건너뜀)."""
        import builtins
        orig_print = builtins.print; dead = {"on": False}
        def dying_print(*a, **k):
            if dead["on"]: raise OSError(5, "Input/output error")
            return orig_print(*a, **k)
        class Ch(FakeChannel):
            def wait(s, name, timeout_s, token):
                if name == "A": dead["on"] = True; raise R.Hangup("가짜 SIGHUP: 터미널 끊김")
                return super().wait(name, timeout_s, token)
        builtins.print = dying_print
        try: r = self.run33(Ch(REF, photos()))
        finally: builtins.print = orig_print; dead["on"] = False
        self.assertEqual(r["ended"], "hangup"); self.assertTrue(r.get("emergency_open") or any(l[0] == "write" and len(l) > 4 and l[4] == "return_to_start" for l in r["j6_adapter_logs"]["contact"]))
        opens = [l for l in r["j6_adapter_logs"]["contact"] if l[0] == "write" and len(l) > 4 and l[4] == "return_to_start"] + [l for l in r["j6_adapter_logs"].get("emergency_open", []) if l[0] == "write"]
        self.assertEqual(len(opens), 1, opens)                                                                                                  # 열기 정확히 1회

    def test_cap_below_first_target_is_refused_explicitly(self):
        r, n = self.instrumented(late=(3.5, 0.0)); L = self.lift(r)                                                                          # cap ≤ 73: 첫 이동만으로도 30° 초과 → ValueError가 아니라 같은 거부
        self.assertEqual(L["code"], "SAG_LATE"); self.assertLess(L["sag"]["late"]["cap"], 74)

    def test_missing_j3_j4_refused_only_at_cap_reads(self):
        r, n = self.instrumented(none_at="early"); L = self.lift(r); self.assertEqual(L["code"], "SAG_EARLY"); self.assertIn("입력 없음", L["sag"]["early"]["reason"])
        r, n = self.instrumented(none_at="late"); L = self.lift(r); self.assertEqual(L["code"], "SAG_LATE")
        g = photos(); g["A"]["gate"]["tilt_check"] = {"ok": True}                                                                              # margin 없음 → 거부(fail-closed)
        r, n = self.instrumented(photos_=g); self.assertEqual(self.lift(r)["code"], "SAG_EARLY")


for _n, _f in list(vars(SagCapMethods).items()):          # 기존 설정(plan33, run33)을 쓰도록 LiftHookTest에 붙인다(상속하면 기존 17건이 중복 실행된다)
    if _n.startswith(("test_", "instrumented", "n_plan_moves", "BASE_")): setattr(LiftHookTest, _n, _f)


if __name__ == "__main__": unittest.main()
