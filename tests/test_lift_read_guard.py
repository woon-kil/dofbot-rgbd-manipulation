"""서보별 읽기 간격 가드(M5)·`before`/`read_start` None 처리(M5b)·M8 측정 모델 가짜 팔 시험(플랜 v2 `docs/261119-1206-읽기-None-정렬-게이트-수정-플랜-v2.md`).
시험 먼저(2026-11-19): 이 파일의 가드·M5b 시험은 구현 전에 실패해야 한다(수정 전 출력은 Check 문서에 남긴다). 측정 모델 시험은 현재 코드로 통과한다."""
import json, sys, time, unittest, tempfile
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "scripts"), str(ROOT / "tests")]
import test_lift_hook as T
import measured_arm
vclock, R, LH = T.vclock, T.R, T.LH
import g01_m0_readonly as M0


class FakeBus:
    def __init__(self): self.log = []; self.t_end = {}
    def write_byte_data(self, addr, reg, val): self.log.append(("w", reg, val, time.monotonic()))
    def read_word_data(self, addr, reg): self.log.append(("r", reg, None, time.monotonic())); return 0x0008


class BusArm:
    """가드 단위 시험용 팔: SDK 읽기·버스 읽기·쓰기가 모두 있다."""
    addr = 0x15
    def __init__(self): self.bus = FakeBus(); self.sdk = []; self.writes = []
    def Arm_serial_servo_read(self, jid): self.sdk.append((jid, time.monotonic())); time.sleep(0.0045); return 90
    def Arm_serial_servo_write(self, jid, angle, ms): self.writes.append((jid, angle, ms))


@unittest.skipIf(vclock is None, "러너·생성기 의존성 없음")
class MeasuredModelTest(unittest.TestCase):
    """측정 모델이 M0 표와 실제 사고를 그대로 재현하는지(현재 코드로 통과해야 한다)."""
    def test_M0_table_reproduced(self):
        Arm = measured_arm.make(R.FakeArmS)
        with vclock.virtual_clock():
            rec, err = M0.run(M0.ReadOnlyArm(Arm({i: 90 for i in range(1, 7)})), 3, quiet_s=0.3)
        self.assertIsNone(err)
        for cid, r in rec.items():
            self.assertEqual(r["none"][0], 0, cid)   # 첫째 읽기는 전 조건 0
            self.assertEqual(r["none"][1], 3 if cid in ("C01", "C02", "C18") else 0, cid)   # 둘째 읽기 None은 C01·C02·C18 모든 시행뿐

    def test_control_adapter_without_guard_fails_at_known_points(self):
        """대조군(가드 없음, 어댑터 단위): read_start 직후 read_position(C01형)은 None, raw_set 직후 같은 서보 read_start(C18형)는 거부된다."""
        Arm = measured_arm.make(R.FakeArmS)
        with vclock.virtual_clock():
            arm = Arm({i: 90 for i in range(1, 7)}); today = __import__("datetime").date.today()
            ad = R.GripperTestAdapter(arm, {"approved": True, "expires": today.isoformat(), "allowed_times_ms": [1500], "first_move_min_ms": None, "max_writes": 3, "delta_deg": 1,
                                            "joint_id": 2, "direction": 1, "angle_window": [66, 110], "pattern": "list"}, today=today, dry_run=False)
            ad.read_start(); self.assertIsNone(ad.read_position())        # C01형
            time.sleep(0.2)
            R.raw_set(arm)                                                  # 마지막 읽기 J4
            ad4 = R.GripperTestAdapter(arm, {"approved": True, "expires": today.isoformat(), "allowed_times_ms": [1500], "first_move_min_ms": None, "max_writes": 3, "delta_deg": 1,
                                             "joint_id": 4, "direction": 1, "angle_window": [10, 105], "pattern": "list"}, today=today, dry_run=False)
            with self.assertRaises(R.GripperTestError): ad4.read_start()    # C18형


@unittest.skipIf(vclock is None, "러너·생성기 의존성 없음")
class GuardUnitTest(unittest.TestCase):
    def setUp(self):
        try: import g01_read_guard as G
        except ImportError as e: self.fail("가드 모듈 없음(구현 전): %s" % e)
        self.G = G
    def test_min_gap_from_previous_response_end_sdk(self):
        with vclock.virtual_clock():
            arm = BusArm(); g = self.G.ReadGuardArm(arm)
            g.Arm_serial_servo_read(2); end1 = time.monotonic(); g.Arm_serial_servo_read(2)
            self.assertGreaterEqual(arm.sdk[1][1] - end1, 0.05 - 1e-9)       # 응답 끝 → 다음 요청 시작 ≥ 0.05 s
    def test_other_servo_does_not_wait_but_same_servo_does_even_with_reads_between(self):
        with vclock.virtual_clock():
            arm = BusArm(); g = self.G.ReadGuardArm(arm)
            t0 = time.monotonic(); g.Arm_serial_servo_read(2); g.Arm_serial_servo_read(3); g.Arm_serial_servo_read(4)
            self.assertLess(time.monotonic() - t0, 0.05)                       # 다른 서보는 기다리지 않는다
            end_j2 = t0 + 0.0045; g.Arm_serial_servo_read(2)
            self.assertGreaterEqual(arm.sdk[3][1] - end_j2, 0.05 - 1e-9)       # 사이에 다른 읽기가 있어도 J2는 J2 응답 끝 기준 0.05 s
    def test_bus_level_reads_guarded_and_other_bus_writes_pass(self):
        with vclock.virtual_clock():
            arm = BusArm(); g = self.G.ReadGuardArm(arm)
            g.bus.write_byte_data(0x15, 0x32, 0); g.bus.read_word_data(0x15, 0x32); end1 = time.monotonic()
            g.bus.write_byte_data(0x15, 0x32, 0)
            self.assertGreaterEqual(time.monotonic() - end1, 0.05 - 1e-9)
            g.bus.write_byte_data(0x15, 0x19, 7); self.assertIn(("w", 0x19, 7), [(a, b, c) for a, b, c, _ in arm.bus.log])   # 읽기 요청이 아닌 쓰기는 그대로 통과
    def test_raw_servo_read_exposed_only_if_underlying_has_it(self):
        arm = BusArm(); g = self.G.ReadGuardArm(arm); self.assertIsNone(getattr(g, "raw_servo_read", None))
        Arm = measured_arm.make(R.FakeArmS); f = Arm({i: 90 for i in range(1, 7)}); gf = self.G.ReadGuardArm(f); self.assertIsNotNone(getattr(gf, "raw_servo_read", None))
    def test_fake_raw_set_then_sdk_j4_waits(self):
        """`raw_servo_read`가 가드를 우회하지 않는다: raw_set 직후 SDK J4 → 가드 대기가 실제로 발생하고 None이 아니다."""
        Arm = measured_arm.make(R.FakeArmS)
        with vclock.virtual_clock():
            f = Arm({i: 90 for i in range(1, 7)}); g = self.G.ReadGuardArm(f); R.raw_set(g)
            v = g.Arm_serial_servo_read(4); self.assertIsNotNone(v)
            self.assertTrue(any(w[0] == 4 and w[1] > 0 for w in g.wait_log), g.wait_log)
    def test_wait_log_records_servo_seconds_label_and_other_attrs_forward(self):
        with vclock.virtual_clock():
            arm = BusArm(); g = self.G.ReadGuardArm(arm); g.set_label("lift1"); g.Arm_serial_servo_read(2); g.Arm_serial_servo_read(2)
            self.assertEqual(len(g.wait_log), 1); s, w, lab = g.wait_log[0]; self.assertEqual((s, lab), (2, "lift1")); self.assertGreater(w, 0.04)
            g.Arm_serial_servo_write(1, 90, 500); self.assertEqual(arm.writes, [(1, 90, 500)])   # 쓰기는 가드를 거치지 않고 그대로


@unittest.skipIf(vclock is None, "러너·생성기 의존성 없음")
class SetThirtyThreeIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        plan31 = json.load(open(T.FIX, encoding="utf-8")); cls.plan33, _ = T.PL.make_lift_plan(plan31, T.G, T.C)
        cls.td = tempfile.TemporaryDirectory(); cls.path = Path(cls.td.name) / "plan33.json"; cls.path.write_text(json.dumps(cls.plan33), encoding="utf-8")
    @classmethod
    def tearDownClass(cls): cls.td.cleanup()
    def setUp(self):
        self.old = (R.LIFT_CHANNEL, T.LP.PARAMS["clock_limit_s"], LH._adapter, R.FakeArmS, LH.lift_after_stall)
        self.Base = R.FakeArmS; self.Arm = measured_arm.make(self.Base); self.arms = []; Arm = self.Arm; arms = self.arms
        class Rec(Arm):
            def __init__(s, *a, **k): super().__init__(*a, **k); arms.append(s)
        R.FakeArmS = Rec
    def tearDown(self): R.LIFT_CHANNEL, T.LP.PARAMS["clock_limit_s"], LH._adapter, R.FakeArmS, LH.lift_after_stall = self.old
    def run33(self, plan_path=None, extra=()):
        R.LIFT_CHANNEL = T.FakeChannel(T.REF, T.photos())
        return vclock.run_runner_fake(plan_path or self.path, ("--fake-ceil", "6:146", "--fake-hyst", "1.9,0.27", *extra))
    def lift(self, r): return r["poses"][3]["run2"].get("lift") or {}

    def test_completes_third_move_and_every_before_is_read(self):
        """측정 모델에서 수정 전에는 이동 1·2 before가 None이고 셋째 이동 어댑터가 거부된다(재실행 #1 사고 재현). 수정 후에는 셋째 이동까지 완료된다."""
        r = self.run33(); lift = self.lift(r)
        self.assertNotEqual(r["ended"], "adapter_refused", r.get("error"))
        moves = [m for m in lift["moves"] if not m.get("aborted")]; self.assertEqual([m["step"] for m in moves], ["lift1", "lift2", "lift3"], lift.get("adapter_error"))
        for m in moves: self.assertIsNotNone(m["readback_before"], m["step"])
    def test_guard_wait_per_step_within_step_upper_bound_addition(self):
        """읽기 시간 0인 가짜 팔(최대 대기)로 단계별 가드 대기 합이 `step_upper_bounds` 가산값(GUARD_WAIT_UB_S) 이하임(재시도 최악 0.2 s는 이동 단계에 따로 포함)."""
        import g01_lift_logic as L
        guards = []; Rg = LH  # noqa
        import g01_read_guard as RG; orig = RG.ReadGuardArm.__init__
        def init(s, *a, **k): orig(s, *a, **k); guards.append(s)
        RG.ReadGuardArm.__init__ = init
        R.FakeArmS = self.Base   # 읽기 시간 0(측정 모델 아님)
        try: self.run33()
        finally: RG.ReadGuardArm.__init__ = orig
        by = {}
        for s_, w, lab in guards[0].wait_log: by[lab] = by.get(lab, 0.0) + w
        for lab, ub in L.GUARD_WAIT_UB_S.items(): self.assertLessEqual(by.get(lab, 0.0), ub - (0.2 if lab.startswith("lift") else 0.0) + 1e-9, (lab, by.get(lab), ub))
        self.assertEqual({k for k in by if k not in ("창 밖",)} - set(L.GUARD_WAIT_UB_S), set(), by)   # 상한이 없는 단계에서 대기가 생기지 않는다
    def test_guard_installed_in_fake_mode_set33_and_not_in_set31(self):
        seen = []; orig = LH.lift_after_stall
        def spy(ctx): seen.append(type(ctx["arm"]).__name__); return orig(ctx)
        LH.lift_after_stall = spy; self.run33()
        self.assertEqual(seen, ["ReadGuardArm"], seen)   # 세트 33에서는 fake 모드에서도 같은 가드로 감싼다
    def test_guard_install_has_no_mode_condition_and_set31_has_no_guard(self):
        """가드 설치 줄은 plan["set"] == "33"만 조건으로 하며 실행 모드(fake·dry_run·real) 조건이 없다(정적 확인). 세트 31 실행에는 가드가 설치되지 않는다(동적)."""
        src = (ROOT / "scripts" / "g01_p2k_test.py").read_text(encoding="utf-8").splitlines()
        i = [k for k, l in enumerate(src) if "import g01_read_guard as RG" in l][0]; cond = src[i - 1]
        self.assertIn('plan["set"] == "33"', cond); self.assertNotIn("mode", cond); self.assertNotIn("a.fake", cond); self.assertNotIn("a.real", cond)
        R.P2K.pop("guard", None); plan31 = json.load(open(T.FIX, encoding="utf-8")); p = Path(self.td.name) / "plan31.json"; p.write_text(json.dumps(plan31), encoding="utf-8")
        R.LIFT_CHANNEL = None; vclock.run_runner_fake(p, ("--fake-ceil", "6:146", "--fake-hyst", "1.9,0.27"))
        self.assertNotIn("guard", R.P2K); self.assertFalse(any(type(a).__name__ == "ReadGuardArm" for a in self.arms))
    def test_before_transient_none_is_retried_and_move_completes(self):
        """before가 한 번 None이면 가드 대기 뒤 재읽기로 성공하고 이동이 그대로 나간다(기존 3회 규칙 재사용)."""
        cnt = {"n": 0}; orig = LH._adapter
        def wrapped(ctx, joint, target):
            cnt["n"] += 1; ad = orig(ctx, joint, target)
            if cnt["n"] == 1: ctx["arm"].forced_none[joint] = 1
            return ad
        LH._adapter = wrapped; r = self.run33(); lift = self.lift(r)
        moves = [m for m in lift["moves"] if not m.get("aborted")]; self.assertEqual(len(moves), 3); self.assertIsNotNone(moves[0]["readback_before"])
    def _persistent_before_none(self, nth):
        cnt = {"n": 0}; orig = LH._adapter
        def wrapped(ctx, joint, target):
            cnt["n"] += 1; ad = orig(ctx, joint, target)
            if cnt["n"] == nth: ctx["arm"].forced_none[joint] = 3   # before 3회 시도를 모두 None으로(그 뒤 복귀 읽기는 정상)
            return ad
        LH._adapter = wrapped; return self.run33()
    def test_clock_log_and_guard_waits_saved_on_failure_paths(self):
        """M7: 실패·중단 경로(before 끝내 None)에서도 clock_log가 저장되고 가드 대기 기록이 남는다."""
        r = self._persistent_before_none(3); lift = self.lift(r)
        self.assertTrue(lift.get("clock_log")); self.assertEqual(lift["clock_log"][0][0], "close145")   # 첫 단계 기록(145 닫기부터 센다)
        self.assertTrue(any(l[0] in ("lift1", "lift2", "lift3") for l in lift["clock_log"])); self.assertIn("guard_waits", lift); self.assertTrue(all(len(w) == 3 for w in lift["guard_waits"]))
    def test_before_persistent_none_in_move1_aborts_before_write(self):
        """새 중단 경로(이동 1): before가 끝내 None이면 쓰기 전 중단, 쓰기 0, lifted_once False, 전용 라벨, 복귀가 쓰기 없이 멈추지 않는다."""
        r = self._persistent_before_none(1); lift = self.lift(r)
        self.assertEqual(lift.get("code"), "BEFORE_NONE"); self.assertFalse(lift.get("lifted_once"))
        self.assertFalse([w for w in self.arms[0].writes if w[0] == 2 and w[1] == 74], "이번 이동의 쓰기 0")
        self.assertNotIn("return_state_mismatch", r); self.assertNotIn("return_abort", r); self.assertTrue(r.get("returned_to_start"))
    def test_before_persistent_none_in_move3_keeps_lifted_state(self):
        """새 중단 경로(이동 3): lifted_once True, 이번 이동(J4) 쓰기 0, 복귀 색인이 들어올린 상태 항목, 복귀 완료."""
        r = self._persistent_before_none(3); lift = self.lift(r)
        self.assertEqual(lift.get("code"), "BEFORE_NONE"); self.assertTrue(lift.get("lifted_once"))
        self.assertFalse([w for w in self.arms[0].writes if w[0] == 4 and w[1] == 27], "이번 이동의 쓰기 0")
        self.assertIn("return_index_used", lift); self.assertNotIn("return_state_mismatch", r); self.assertNotIn("return_abort", r); self.assertTrue(r.get("returned_to_start"))
    def test_read_start_retry_only_on_none_not_on_window_refusal(self):
        """read_start가 창 밖 값으로 거부되면 재시도하지 않는다(None일 때만 재시도)."""
        cnt = {"n": 0}; orig = LH._adapter
        def wrapped(ctx, joint, target):
            cnt["n"] += 1
            if cnt["n"] == 1:
                ctx["arm"].forced_value[joint] = [200]   # 첫 read_start가 승인 창 밖 값을 읽는다(다시 읽으면 정상 값이라, 재시도하면 거부되지 않는다)
            return orig(ctx, joint, target)
        LH._adapter = wrapped; r = self.run33(); lift = self.lift(r)
        self.assertEqual(lift["adapter_error"]["type"], "GripperTestError"); self.assertNotIn("None", lift["adapter_error"]["error"])
        self.assertIn("승인 범위 밖", lift["adapter_error"]["error"])


if __name__ == "__main__": unittest.main()
