import datetime, sys, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from control.gripper_adapter import GripperTestAdapter, GripperTestError


class FakeArm:
    """Arm_Lib.py 원문의 시그니처를 따른다: Arm_serial_servo_read(self, id), Arm_serial_servo_write(self, id, angle, time).
    다른 메서드(write6, Arm_reset, set_torque 등)는 호출되면 실패한다."""
    def __init__(self, value=100):
        self.value = value; self.writes = []; self.reads = 0
    def Arm_serial_servo_read(self, id):
        self.reads += 1
        return self.value
    def Arm_serial_servo_write(self, id, angle, time):
        self.writes.append((id, angle, time))
    def __getattr__(self, name):
        raise AssertionError("허용되지 않은 SDK API 호출: " + name)


def approval(**kw):
    a = {"approved": True, "expires": "2026-12-31", "allowed_times_ms": [1500, 3000], "max_writes": 6, "delta_deg": 5}
    a.update(kw); return a

TODAY = datetime.date(2026, 11, 12)


def ready(value=100, dry_run=False, **kw):
    arm = FakeArm(value); ad = GripperTestAdapter(arm, approval(**kw), today=TODAY, dry_run=dry_run)
    s = ad.read_start(); ad.set_targets([s, s - 5, s - 10, s]); return arm, ad


class TestAdapter(unittest.TestCase):
    def test_unapproved_rejected(self):
        with self.assertRaises(GripperTestError): GripperTestAdapter(FakeArm(), approval(approved=False), today=TODAY)
    def test_expired_rejected(self):
        with self.assertRaises(GripperTestError): GripperTestAdapter(FakeArm(), approval(expires="2026-11-11"), today=TODAY)
    def test_short_time_rejected(self):
        with self.assertRaises(GripperTestError): GripperTestAdapter(FakeArm(), approval(allowed_times_ms=[300]), today=TODAY)
    def test_none_read_aborts(self):
        ad = GripperTestAdapter(FakeArm(None), approval(), today=TODAY)
        with self.assertRaises(GripperTestError): ad.read_start()
    def test_start_read_only_once(self):
        ad = GripperTestAdapter(FakeArm(), approval(), today=TODAY); ad.read_start()
        with self.assertRaises(GripperTestError): ad.read_start()
    def test_targets_must_match_formula(self):
        ad = GripperTestAdapter(FakeArm(100), approval(), today=TODAY); ad.read_start()
        with self.assertRaises(GripperTestError): ad.set_targets([100, 90, 80, 100])
    def test_targets_out_of_range(self):
        ad = GripperTestAdapter(FakeArm(8), approval(), today=TODAY); ad.read_start()
        with self.assertRaises(GripperTestError): ad.set_targets([8, 3, -2, 8])
    def test_move_needs_hands_clear(self):
        arm, ad = ready()
        with self.assertRaises(GripperTestError): ad.move(95, 1500)
        self.assertEqual(arm.writes, [])
    def test_hands_clear_single_use(self):
        arm, ad = ready(); ad.confirm_hands_clear(); ad.move(95, 1500)
        with self.assertRaises(GripperTestError): ad.move(90, 1500)
    def test_order_and_values_enforced(self):
        arm, ad = ready()
        ad.confirm_hands_clear()
        with self.assertRaises(GripperTestError): ad.move(90, 1500)   # 순서 건너뜀 (첫 이동은 S-Δ)
        with self.assertRaises(GripperTestError): ad.move(100, 1500)  # 첫 이동은 S가 아님
    def test_time_enforced(self):
        arm, ad = ready(); ad.confirm_hands_clear()
        with self.assertRaises(GripperTestError): ad.move(95, 2000)
    def test_full_sequence_writes_only_servo6(self):
        arm, ad = ready()
        for tgt, t in ((95, 1500), (90, 3000), (100, 1500)):
            ad.confirm_hands_clear(); ad.move(tgt, t)
        self.assertEqual(arm.writes, [(6, 95, 1500), (6, 90, 3000), (6, 100, 1500)])
        ad.confirm_hands_clear()
        with self.assertRaises(GripperTestError): ad.move(95, 1500)  # 목록 소진
    def test_max_writes_cap(self):
        arm, ad = ready(max_writes=2)
        ad.confirm_hands_clear(); ad.move(95, 1500); ad.confirm_hands_clear(); ad.move(90, 1500)
        ad.confirm_hands_clear()
        with self.assertRaises(GripperTestError): ad.move(100, 1500)
    def test_dry_run_never_writes(self):
        arm, ad = ready(dry_run=True); ad.confirm_hands_clear(); ad.move(95, 1500)
        self.assertEqual(arm.writes, [])
    def test_only_two_sdk_apis_used(self):
        arm, ad = ready(); ad.confirm_hands_clear(); ad.move(95, 1500); ad.read_position()  # 다른 API 호출 시 FakeArm이 AssertionError
    def test_no_id_parameter(self):
        import inspect
        self.assertNotIn("id", inspect.signature(GripperTestAdapter.move).parameters)

    def test_return_to_start_after_write(self):
        arm, ad = ready(); ad.confirm_hands_clear(); ad.move(95, 1500)
        ad.confirm_hands_clear(); ad.return_to_start(1500)
        self.assertEqual(arm.writes, [(6, 95, 1500), (6, 100, 1500)])
    def test_return_needs_prior_write_and_hands_clear_and_once(self):
        arm, ad = ready()
        ad.confirm_hands_clear()
        with self.assertRaises(GripperTestError): ad.return_to_start(1500)   # 쓰기 이력 없음
        ad.move(95, 1500)
        with self.assertRaises(GripperTestError): ad.return_to_start(1500)   # 손 제거 확인 소진
        ad.confirm_hands_clear(); ad.return_to_start(1500)
        ad.confirm_hands_clear()
        with self.assertRaises(GripperTestError): ad.return_to_start(1500)   # 한 번만
    def test_return_counts_toward_max_writes(self):
        arm, ad = ready(max_writes=1); ad.confirm_hands_clear(); ad.move(95, 1500)
        ad.confirm_hands_clear()
        with self.assertRaises(GripperTestError): ad.return_to_start(1500)
    def test_return_time_enforced(self):
        arm, ad = ready(); ad.confirm_hands_clear(); ad.move(95, 1500); ad.confirm_hands_clear()
        with self.assertRaises(GripperTestError): ad.return_to_start(2000)
    def test_return_dry_run_no_write(self):
        arm, ad = ready(dry_run=True); ad.confirm_hands_clear(); ad.move(95, 1500); ad.confirm_hands_clear(); ad.return_to_start(1500)
        self.assertEqual(arm.writes, [])


class TestJoint5(unittest.TestCase):
    def wrist(self, value=90, **kw):
        arm = FakeArm(value)
        ad = GripperTestAdapter(arm, approval(joint_id=5, direction=1, angle_window=[60, 120], **kw), today=TODAY, dry_run=False)
        s = ad.read_start(); ad.set_targets([s, s + 5, s + 10, s]); return arm, ad
    def test_wrist_moves_servo5_only(self):
        arm, ad = self.wrist(); ad.confirm_hands_clear(); ad.move(95, 1500)
        self.assertEqual(arm.writes, [(5, 95, 1500)])
    def test_disallowed_joint_ids_rejected(self):
        for jid in (0, 7, -1, 8):
            with self.assertRaises(GripperTestError): GripperTestAdapter(FakeArm(), approval(joint_id=jid), today=TODAY)
    def test_window_enforced_on_start(self):
        ad = GripperTestAdapter(FakeArm(130), approval(joint_id=5, direction=1, angle_window=[60, 120]), today=TODAY)
        with self.assertRaises(GripperTestError): ad.read_start()
    def test_window_enforced_on_targets(self):
        ad = GripperTestAdapter(FakeArm(115), approval(joint_id=5, direction=1, angle_window=[60, 120]), today=TODAY); s = ad.read_start()
        with self.assertRaises(GripperTestError): ad.set_targets([s, s + 5, s + 10, s])   # 125 > 120
    def test_window_outside_sdk_range_rejected(self):
        with self.assertRaises(GripperTestError): GripperTestAdapter(FakeArm(), approval(joint_id=5, angle_window=[0, 300]), today=TODAY)
    def test_direction_negative_formula(self):
        arm = FakeArm(90); ad = GripperTestAdapter(arm, approval(joint_id=5, direction=-1, angle_window=[60, 120]), today=TODAY); s = ad.read_start()
        ad.set_targets([s, s - 5, s - 10, s])
    def test_wrong_direction_list_rejected(self):
        ad = GripperTestAdapter(FakeArm(90), approval(joint_id=5, direction=1, angle_window=[60, 120]), today=TODAY); s = ad.read_start()
        with self.assertRaises(GripperTestError): ad.set_targets([s, s - 5, s - 10, s])
    def test_return_uses_joint5(self):
        arm, ad = self.wrist(); ad.confirm_hands_clear(); ad.move(95, 1500); ad.confirm_hands_clear(); ad.return_to_start(1500)
        self.assertEqual(arm.writes, [(5, 95, 1500), (5, 90, 1500)])


class TestJoint1Pair(unittest.TestCase):
    def base(self, value=90, dry_run=False, **kw):
        arm = FakeArm(value)
        ad = GripperTestAdapter(arm, approval(joint_id=1, direction=1, angle_window=[70, 110], pattern="pair", **kw), today=TODAY, dry_run=dry_run)
        s = ad.read_start(); ad.set_targets([s, s + 5, s, s + 5, s]); return arm, ad
    def test_pair_sequence_writes_servo1(self):
        arm, ad = self.base()
        for tgt, t in ((95, 1500), (90, 1500), (95, 3000), (90, 3000)):
            ad.confirm_hands_clear(); ad.move(tgt, t)
        self.assertEqual(arm.writes, [(1, 95, 1500), (1, 90, 1500), (1, 95, 3000), (1, 90, 3000)])
    def test_pair_pattern_rejects_ramp_list(self):
        ad = GripperTestAdapter(FakeArm(90), approval(joint_id=1, direction=1, angle_window=[70, 110], pattern="pair"), today=TODAY); s = ad.read_start()
        with self.assertRaises(GripperTestError): ad.set_targets([s, s + 5, s + 10, s])
    def test_ramp_pattern_rejects_pair_list(self):
        ad = GripperTestAdapter(FakeArm(90), approval(joint_id=1, direction=1, angle_window=[70, 110]), today=TODAY); s = ad.read_start()
        with self.assertRaises(GripperTestError): ad.set_targets([s, s + 5, s, s + 5, s])
    def test_unknown_pattern_rejected(self):
        with self.assertRaises(GripperTestError): GripperTestAdapter(FakeArm(), approval(joint_id=1, pattern="zigzag"), today=TODAY)
    def test_joint1_window_enforced(self):
        ad = GripperTestAdapter(FakeArm(120), approval(joint_id=1, direction=1, angle_window=[70, 110], pattern="pair"), today=TODAY)
        with self.assertRaises(GripperTestError): ad.read_start()
    def test_pair_order_enforced(self):
        arm, ad = self.base(); ad.confirm_hands_clear()
        with self.assertRaises(GripperTestError): ad.move(90, 1500)   # 첫 이동은 S+Δ
    def test_joints_2_3_4_allowed_with_window(self):
        for jid in (2, 3, 4):
            arm = FakeArm(90); ad = GripperTestAdapter(arm, approval(joint_id=jid, direction=1, angle_window=[80, 100], pattern="pair"), today=TODAY, dry_run=False)
            s = ad.read_start(); ad.set_targets([s, s + 5, s, s + 5, s]); ad.confirm_hands_clear(); ad.move(95, 1500)
            self.assertEqual(arm.writes, [(jid, 95, 1500)])
    def test_pitch_joint_window_enforced(self):
        for jid in (2, 3, 4):
            ad = GripperTestAdapter(FakeArm(105), approval(joint_id=jid, direction=1, angle_window=[80, 100], pattern="pair"), today=TODAY)
            with self.assertRaises(GripperTestError): ad.read_start()
    def test_pitch_joint_cannot_leave_window_via_targets(self):
        ad = GripperTestAdapter(FakeArm(98), approval(joint_id=3, direction=1, angle_window=[80, 100], pattern="pair"), today=TODAY); s = ad.read_start()
        with self.assertRaises(GripperTestError): ad.set_targets([s, s + 5, s, s + 5, s])   # 103 > 100


class TestFirstMove(unittest.TestCase):
    def pitch(self, **kw):
        arm = FakeArm(90)
        ad = GripperTestAdapter(arm, approval(joint_id=4, direction=1, angle_window=[80, 100], pattern="pair", allowed_times_ms=[1500, 8000], first_move_min_ms=8000, **kw), today=TODAY, dry_run=False)
        s = ad.read_start(); ad.set_targets([s, s + 5, s, s + 5, s]); return arm, ad
    def test_first_write_1500_rejected(self):
        arm, ad = self.pitch(); ad.confirm_hands_clear()
        with self.assertRaises(GripperTestError): ad.move(95, 1500)
        self.assertEqual(arm.writes, [])
    def test_first_write_8000_then_1500_allowed(self):
        arm, ad = self.pitch(); ad.confirm_hands_clear(); ad.move(95, 8000); ad.confirm_hands_clear(); ad.move(90, 1500)
        self.assertEqual(arm.writes, [(4, 95, 8000), (4, 90, 1500)])
    def test_first_move_min_must_be_in_allowed_times(self):
        with self.assertRaises(GripperTestError): GripperTestAdapter(FakeArm(90), approval(joint_id=4, allowed_times_ms=[1500], first_move_min_ms=8000), today=TODAY)
    def test_no_first_move_rule_when_key_absent(self):
        arm, ad = ready(); ad.confirm_hands_clear(); ad.move(95, 1500)   # 그리퍼 G01.5a 경로(키 없음)는 기존 동작
        self.assertEqual(arm.writes, [(6, 95, 1500)])

class TestSpeedAndStair(unittest.TestCase):
    """동결 상한: 이동 1회 90° 이하, 60 ms/° 이상(직전 명령 위치 기준). stair 패턴(S3)."""
    def stair(self, s=91, delta=20, n=4, **kw):
        arm = FakeArm(s)
        ad = GripperTestAdapter(arm, approval(joint_id=3, direction=-1, delta_deg=delta, angle_window=[10, 105], pattern="stair", stair_steps=n, allowed_times_ms=[1500, 3000, 8000], first_move_min_ms=8000, max_writes=10, **kw), today=TODAY, dry_run=False)
        s0 = ad.read_start(); ad.set_targets([s0] + [s0 - delta * i for i in range(1, n + 1)] + [s0 - delta * i for i in range(n - 1, -1, -1)]); return arm, ad
    def go(self, ad, target, ms): ad.confirm_hands_clear(); ad.move(target, ms)
    def test_stair_full_sequence(self):
        arm, ad = self.stair()
        for tgt, ms in [(71, 8000), (51, 1500), (31, 1500), (11, 3000), (31, 3000), (51, 1500), (71, 1500), (91, 1500)]: self.go(ad, tgt, ms)
        self.assertEqual([w[1] for w in arm.writes], [71, 51, 31, 11, 31, 51, 71, 91]); self.assertTrue(all(w[0] == 3 for w in arm.writes))
    def test_stair_wrong_list_rejected(self):
        arm = FakeArm(91); ad = GripperTestAdapter(arm, approval(joint_id=3, direction=-1, delta_deg=20, angle_window=[10, 105], pattern="stair", stair_steps=4, allowed_times_ms=[1500, 8000], first_move_min_ms=8000, max_writes=10), today=TODAY)
        s = ad.read_start()
        with self.assertRaises(GripperTestError): ad.set_targets([s, s - 20, s - 40, s - 20, s])   # 4단계가 아닌 목록
    def test_stair_needs_steps(self):
        with self.assertRaises(GripperTestError): GripperTestAdapter(FakeArm(91), approval(joint_id=3, pattern="stair", direction=-1, delta_deg=20, angle_window=[10, 105]), today=TODAY)
    def test_stair_target_below_window_rejected(self):
        arm = FakeArm(91); ad = GripperTestAdapter(arm, approval(joint_id=3, direction=-1, delta_deg=20, angle_window=[20, 105], pattern="stair", stair_steps=4, allowed_times_ms=[1500, 8000], first_move_min_ms=8000, max_writes=10), today=TODAY)
        s = ad.read_start()
        with self.assertRaises(GripperTestError): ad.set_targets([s] + [s - 20 * i for i in range(1, 5)] + [s - 20 * i for i in range(3, -1, -1)])   # 11 < 20
    def test_speed_limit_rejects_fast_big_move(self):
        arm = FakeArm(100); ad = GripperTestAdapter(arm, approval(joint_id=3, direction=-1, delta_deg=40, angle_window=[10, 105], pattern="ramp", allowed_times_ms=[1500, 8000], first_move_min_ms=8000, max_writes=6), today=TODAY, dry_run=False)
        s = ad.read_start(); ad.set_targets([s, s - 40, s - 80, s]); self.go(ad, s - 40, 8000)
        with self.assertRaises(GripperTestError): self.go(ad, s - 80, 1500)   # 40° / 1500 ms = 37.5 ms/° < 60
        self.assertEqual(len(arm.writes), 1)
    def test_speed_limit_exact_boundary_allowed(self):
        arm, ad = self.stair(); self.go(ad, 71, 8000); self.go(ad, 51, 1500)   # 20° / 1500 ms = 75 ms/° 허용
        self.assertEqual(len(arm.writes), 2)
    def test_move_over_90_rejected_even_if_slow(self):
        arm = FakeArm(100); ad = GripperTestAdapter(arm, approval(joint_id=3, direction=-1, delta_deg=95, angle_window=[0, 180], pattern="pair", allowed_times_ms=[8000], first_move_min_ms=8000, max_writes=6), today=TODAY, dry_run=False)
        s = ad.read_start(); ad.set_targets([s, s - 95, s, s - 95, s]); ad.confirm_hands_clear()
        with self.assertRaises(GripperTestError): ad.move(s - 95, 8000)
        self.assertEqual(arm.writes, [])
    def test_return_speed_checked_from_last_target(self):
        arm, ad = self.stair()
        for tgt, ms in [(71, 8000), (51, 1500), (31, 1500), (11, 3000)]: self.go(ad, tgt, ms)
        ad.confirm_hands_clear()
        with self.assertRaises(GripperTestError): ad.return_to_start(1500)   # 11 -> 91 = 80° / 1500 ms
        ad.confirm_hands_clear(); ad.return_to_start(8000)                    # 80° / 8000 ms = 100 ms/°
        self.assertEqual(arm.writes[-1], (3, 91, 8000))

if __name__ == '__main__': unittest.main()
