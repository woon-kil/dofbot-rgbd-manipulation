"""들어올림 어댑터 생성(read_start) 실패 시험(재실행 #1 안전 이상 재현, 플랜 docs/261118-2323-복귀-색인-결함-수정-묶음-플랜.md T1~T4·T8·T9).
`LH._adapter`를 시험 안에서 감싸 특정 순번 이동의 어댑터 생성 때만 서보 i의 읽기를 None으로 만든다(scripts/의 가짜 팔은 고치지 않는다).
수정 전 코드: 복귀 색인이 쓰기 전 거부된 이동의 '끝난 뒤' 색인으로 남아 복귀가 쓰기 없이 멈춘다(return_state_mismatch)."""
import json, sys, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "scripts"), str(ROOT / "tests")]
import test_lift_hook as T
vclock, R, LH = T.vclock, T.R, T.LH


@unittest.skipIf(vclock is None, "러너·생성기 의존성 없음")
class AdapterFailureTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import tempfile
        plan31 = json.load(open(T.FIX, encoding="utf-8")); cls.plan33, _ = T.PL.make_lift_plan(plan31, T.G, T.C)
        cls.td = tempfile.TemporaryDirectory(); cls.path = Path(cls.td.name) / "plan33.json"; cls.path.write_text(json.dumps(cls.plan33), encoding="utf-8")
    @classmethod
    def tearDownClass(cls): cls.td.cleanup()
    def setUp(self):
        self.old = (R.LIFT_CHANNEL, T.LP.PARAMS["clock_limit_s"], LH._adapter, R.FakeArmS)
    def tearDown(self):
        R.LIFT_CHANNEL, T.LP.PARAMS["clock_limit_s"], LH._adapter, R.FakeArmS = self.old

    def run33(self, extra=()):
        R.LIFT_CHANNEL = T.FakeChannel(T.REF, T.photos())
        return vclock.run_runner_fake(self.path, ("--fake-ceil", "6:146", "--fake-hyst", "1.9,0.27", *extra))

    def fail_adapter_at(self, nth):
        """nth번째(1=lift1, 2=lift2, 3=lift3) 들어올림 어댑터 생성에서 해당 관절의 읽기를 None으로 만든다."""
        orig = self.old[2]; cnt = {"n": 0}
        def wrapped(ctx, joint, target):
            cnt["n"] += 1
            if cnt["n"] != nth: return orig(ctx, joint, target)
            ctx["arm"].none_ids.add(joint)
            try: return orig(ctx, joint, target)
            finally: ctx["arm"].none_ids.discard(joint)
        LH._adapter = wrapped; return cnt

    def lift(self, r): return r["poses"][3]["run2"].get("lift") or {}
    def opens(self, r):
        logs = r.get("j6_adapter_logs") or {}
        return [l for l in logs.get("contact", []) if l[0] == "write" and len(l) > 4 and l[4] == "return_to_start"] + [l for l in logs.get("emergency_open", []) if l[0] == "write"]

    def check_recovers(self, r):
        self.assertEqual(r["ended"], "adapter_refused", r.get("error"))
        self.assertNotIn("return_state_mismatch", r, r.get("return_state_mismatch"))      # 수정 전: 쓰기 없이 멈춤
        self.assertNotIn("return_abort", r, r.get("return_abort"))                         # 복귀 모든 이동이 이탈 검사 통과
        self.assertTrue(r.get("returned_to_start"))
        self.assertEqual(len(self.opens(r)), 1, self.opens(r))                             # 비상 열기 정확히 1회
        self.assertTrue(r.get("emergency_open") or len(self.opens(r)) == 1)

    # --- T1·T3: 셋째 이동(J4) 어댑터 생성 실패 = 재실행 #1 사고 재현 ---
    def test_T1_third_move_adapter_none_returns_to_start(self):
        cnt = self.fail_adapter_at(3); r = self.run33(); self.assertEqual(cnt["n"], 3)
        self.check_recovers(r)
        self.assertEqual([m["target"] for m in self.lift(r)["moves"] if not m.get("aborted")], [74, 77])   # 쓰기는 J2 두 개뿐

    def test_T3_prog_sent_is_last_written_stage(self):
        self.fail_adapter_at(3); r = self.run33(); used = self.lift(r).get("return_index_used") or {}
        self.assertTrue(str(used.get("key", "")).startswith("before_j4"), used)            # J4를 쓰지 않았으므로 '이 이동 전' 상태 색인
        self.assertEqual(R.PROG["sent"], used.get("index"), (R.PROG["sent"], used))        # 쓰기 전 거부된 이동의 '끝난 뒤' 색인(29)이 남으면 안 된다(수정 전 29 대 28)

    # --- T2: 첫·둘째 이동 어댑터 생성 실패 ---
    def test_T2_first_move_adapter_none_returns_to_start(self):
        self.fail_adapter_at(1); r = self.run33(); self.check_recovers(r)

    def test_T2_second_move_adapter_none_returns_to_start(self):
        self.fail_adapter_at(2); r = self.run33(); self.check_recovers(r)

    # --- T4: 분류(사전 등록 로직 불변, 출력만) ---
    def test_T4_classification_code(self):
        import g01_lift_classify as CL
        self.fail_adapter_at(3); r = self.run33(); r["mode"] = "real"; c = CL.classify(r)
        self.assertEqual(c["outcome"], "no_lift"); self.assertEqual(c["code"], "ADAPTER_BEFORE_WRITE"); self.assertTrue(c["lifted_once"])

    # --- 어댑터 생성 중 Ctrl-C: 분류 라벨은 붙이지 않고 복귀 색인·cmd 복원만 한다 ---
    def test_keyboard_interrupt_in_adapter_restores_but_not_labeled_adapter(self):
        orig = self.old[2]; cnt = {"n": 0}
        def wrapped(ctx, joint, target):
            cnt["n"] += 1
            if cnt["n"] == 3: raise KeyboardInterrupt()
            return orig(ctx, joint, target)
        LH._adapter = wrapped; r = self.run33(); L = self.lift(r)
        self.assertNotEqual(L.get("code"), "ADAPTER_BEFORE_WRITE"); self.assertEqual((L.get("adapter_error") or {}).get("type"), "KeyboardInterrupt")
        used = L.get("return_index_used") or {}; self.assertEqual(R.PROG["sent"], used.get("index"))      # 복귀 색인은 실제 상태 기준으로 복원됨

    # --- T8: 실패한 어댑터의 로그가 결과 JSON에 남는다 ---
    def test_T8_failed_adapter_log_is_saved(self):
        self.fail_adapter_at(3); r = self.run33(); logs = self.lift(r).get("adapter_logs")
        self.assertIsNotNone(logs, "lift.adapter_logs 없음"); flat = json.dumps(logs)
        self.assertIn('["read", null]', flat)

    # --- T9: 셋째 이동(J4) 이탈 검사. 사용자 승인(2026-11-19)으로 이동 성공 뒤 cmd[joint]를 실제 정수 회독으로 갱신한다(HOLD_TOL 3 불변). 추가 처짐은 셋째 이동 중에만 적용한다 ---
    def sag_arm_class(self):
        base = self.old[3]
        class SagArm(base):
            extra = 0.0; arm_next = False; until = -1.0
            def Arm_serial_servo_write(s, id, angle, time_ms):
                import time as _t
                if id == 4 and s.arm_next: s.until = _t.monotonic() + 3.5; s.arm_next = False   # 셋째 이동의 J4 쓰기부터 3.5 s 동안만 J2가 extra만큼 더 낮게 읽힌다(복귀에는 영향 없음)
                return super().Arm_serial_servo_write(id, angle, time_ms)
            def Arm_serial_servo_read(s, id):
                import time as _t
                v = super().Arm_serial_servo_read(id)
                return v if (v is None or id != 2 or _t.monotonic() > s.until) else int(round(v - s.extra))
        return SagArm

    def run_with_sag(self, extra, path=None, hyst="1.9,0.27"):
        Sag = self.sag_arm_class(); Sag.extra = extra; R.FakeArmS = Sag; orig = self.old[2]
        def wrapped(ctx, joint, target):
            if joint == 4: ctx["arm"].arm_next = True
            return orig(ctx, joint, target)
        LH._adapter = wrapped
        old_path = self.path
        if path is not None: self.path = path
        try:
            R.LIFT_CHANNEL = T.FakeChannel(T.REF, T.photos())
            return vclock.run_runner_fake(self.path, ("--fake-ceil", "6:146", *(("--fake-hyst", hyst) if hyst else ())))
        finally: self.path = old_path

    def test_T9_third_move_hold_check_uses_actual_reading(self):
        out = {}
        for extra in (0.0, 1.0, 2.0, 3.0, 4.0):
            r = self.run_with_sag(extra); L = self.lift(r)
            out[extra] = (r.get("ended"), [(m["target"], bool(m.get("aborted"))) for m in L.get("moves", [])], (r.get("abort_reason") or "")[:70])
        print("\nT9(cmd 갱신 뒤) J2 추가 처짐 → 결과:", json.dumps({str(k): v for k, v in out.items()}, ensure_ascii=False))
        for extra in (0.0, 1.0, 2.0, 3.0): self.assertEqual(out[extra][0], "completed", (extra, out[extra])); self.assertEqual([t for t, a in out[extra][1]], [74, 77, 27])
        self.assertEqual(out[4.0][0], "aborted"); self.assertTrue(out[4.0][1][-1][1])                # 기준값(실제 회독) 대비 3° 넘게 늘면 여전히 셋째 이동이 중단된다

    # --- T10: 셋째 이동 중 J2 하한이 접촉 깊이 위인지(최대 목표 76·77, 둘째 이동 생략 경로, 추가 처짐 포함) ---
    def plan_variant(self, max_target):
        import copy, tempfile
        pl = copy.deepcopy(self.plan33); pl["lift"]["j2_max_target"] = int(max_target)
        f = Path(self.td.name) / ("plan33_max%d.json" % max_target); f.write_text(json.dumps(pl), encoding="utf-8"); return f

    def test_T10_j2_floor_above_contact_depth(self):
        rows = {}
        for mx in (76, 77):
            for hyst in ("1.9,0.27", ""):                     # "" = 서보 부족분 없음(최대 목표가 가까우면 둘째 이동 생략 경로)
                r = self.run_with_sag(3.0, path=self.plan_variant(mx), hyst=hyst); L = self.lift(r)
                moves = L.get("moves", []); contact_j2 = moves[0]["cmd_before"]["2"] if moves else None
                last = next((m for m in reversed(moves) if m.get("target") == 27 or m.get("joint") == 4), None)
                floor = min((v for t, v in (last or {}).get("series", {}).get("2", []) if v is not None), default=None)
                rows[(mx, hyst)] = {"ended": r.get("ended"), "moves": [m["target"] for m in moves], "second": (L.get("second_j2") or {}).get("skipped_reason") or (L.get("second_j2") or {}).get("target"), "contact_j2": contact_j2, "lift3_j2_floor": floor}
        print("\nT10 (최대 목표, hyst) → 기록:", json.dumps({str(k): v for k, v in rows.items()}, ensure_ascii=False))
        for k, v in rows.items():
            self.assertEqual(v["ended"], "completed", (k, v))
            if v["lift3_j2_floor"] is not None: self.assertGreater(v["lift3_j2_floor"], v["contact_j2"], (k, v))   # 하한이 접촉 깊이 J2 위

if __name__ == "__main__": unittest.main()
