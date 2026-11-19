import sys, unittest, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import g01_m0_readonly as M


class _Bus:
    def __init__(self): self.log = []
    def write_byte_data(self, addr, reg, val): self.log.append(("w", reg, val))
    def read_word_data(self, addr, reg): return 0x0008   # 원시 카운트 ≠ 0


class _RealLike:
    addr = 0x15
    def __init__(self): self.bus = _Bus(); self.writes = []
    def Arm_serial_servo_read(self, jid): return 90
    def Arm_serial_servo_write(self, *a): self.writes.append(a)
    def Arm_serial_servo_torque(self, *a): self.writes.append(a)


class ReadOnlyTests(unittest.TestCase):
    def test_write_apis_blocked(self):
        arm = _RealLike(); ro = M.ReadOnlyArm(arm)
        for name in ("Arm_serial_servo_write", "Arm_serial_servo_torque", "Arm_Buzzer", "Arm_serial_servo_write6"):
            with self.assertRaises(M.WriteBlocked): getattr(ro, name)(1, 90, 500)
        self.assertEqual(arm.writes, [])
    def test_bus_writes_limited_to_read_requests(self):
        arm = _RealLike(); ro = M.ReadOnlyArm(arm)
        ro.bus.write_byte_data(0x15, 0x32, 0)   # 읽기 요청
        for reg, val in ((0x12, 0), (0x32, 5), (0x2, 0)):
            with self.assertRaises(M.WriteBlocked): ro.bus.write_byte_data(0x15, reg, val)
        with self.assertRaises(M.WriteBlocked): ro.bus.write_i2c_block_data(0x15, 1, [1])
        self.assertEqual(arm.bus.log, [("w", 0x32, 0)])
    def test_attr_set_blocked(self):
        with self.assertRaises(M.WriteBlocked): M.ReadOnlyArm(_RealLike()).x = 1
    def test_real_like_raw_read_works(self):
        ro = M.ReadOnlyArm(_RealLike()); self.assertEqual(M.read_raw(ro, 2), 0x0800)
        self.assertEqual(ro.Arm_serial_servo_read(2), 90)


class SanityOutputTests(unittest.TestCase):
    def test_sanity_table_has_no_none_columns(self):
        rec, _ = M.run(M.ReadOnlyArm(M.FakeArm(none_every=3)), 3, True, quiet_s=0.0)
        out = M.table(rec, True); self.assertNotIn("None", out); self.assertIn("C18", out)


class RunTests(unittest.TestCase):
    def test_counts_and_table_fake(self):
        arm = M.FakeArm(none_every=5); rec, err = M.run(M.ReadOnlyArm(arm), 3, quiet_s=0.0)
        self.assertIsNone(err); self.assertEqual(len(rec), 18)
        self.assertTrue(all(r["n"] == 3 for r in rec.values()))
        total_none = sum(sum(r["none"]) for r in rec.values()); self.assertGreater(total_none, 0)
        self.assertIn("C16", M.table(rec)); self.assertNotIn("뚜렷", M.table(rec))
    def test_write_attempt_stops_and_keeps_record(self):
        arm = M.FakeArm(); ro = M.ReadOnlyArm(arm); orig = M._sdk; calls = {"n": 0}
        def bad(a, j):
            calls["n"] += 1
            if calls["n"] == 5: a.Arm_serial_servo_write(1, 90, 500)
            return orig(a, j)
        M._sdk = bad
        try: rec, err = M.run(ro, 2, quiet_s=0.0)
        finally: M._sdk = orig
        self.assertIn("WriteBlocked", err)
        self.assertGreaterEqual(rec["C01"]["n"], 1)   # 쓰기 시도 전에 끝난 시행의 기록이 남아 있다
        self.assertEqual(len(arm.writes_seen) if hasattr(arm, "writes_seen") else 0, 0)
    def test_systemexit_keeps_partial_record(self):
        arm = M.FakeArm(); orig = M._sdk; calls = {"n": 0}
        def bad(a, j):
            calls["n"] += 1
            if calls["n"] == 7: raise SystemExit("SIGHUP")
            return orig(a, j)
        M._sdk = bad
        try: rec, err = M.run(M.ReadOnlyArm(arm), 2, quiet_s=0.0)
        finally: M._sdk = orig
        self.assertIn("SystemExit", err); self.assertGreaterEqual(rec["C01"]["n"], 1)


if __name__ == "__main__": unittest.main()
