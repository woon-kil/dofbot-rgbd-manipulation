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

if __name__ == '__main__': unittest.main()
