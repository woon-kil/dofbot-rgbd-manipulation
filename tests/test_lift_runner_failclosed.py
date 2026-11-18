"""러너 수준 fail-closed 시험(사용자 결정 2026-11-18 '이 거부는 가짜 시험으로 확인'): `--real`과 세트 33으로 g01_p2k_test.main()을 실행하되 Arm_Lib를 호출 기록용 가짜 모듈로 바꾼다.
통과 조건: UNSET이 남아 있으면 SystemExit(세트 33: ... 미확정), Arm_Device 생성 0회·쓰기 0회·input(안전 질문) 0회."""
import builtins, copy, json, sys, tempfile, types, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import g01_lift_params as LP
try:
    import g01_p2k_test as R
except Exception as e:   # 시스템 python에 의존성이 없으면 건너뜀
    R = None; _err = e


class Rec:
    created = 0; writes = 0; inputs = 0


class FakeArmDevice:
    def __init__(self, *a, **k): Rec.created += 1
    def Arm_serial_servo_write(self, *a, **k): Rec.writes += 1
    def Arm_serial_servo_write6(self, *a, **k): Rec.writes += 1
    def Arm_serial_servo_read(self, i): return 90


@unittest.skipIf(R is None, "g01_p2k_test import 불가")
class RunnerFailClosedTest(unittest.TestCase):
    def setUp(self):
        Rec.created = Rec.writes = Rec.inputs = 0
        self.mod = types.ModuleType("Arm_Lib"); self.mod.Arm_Device = FakeArmDevice; self.old = sys.modules.get("Arm_Lib"); sys.modules["Arm_Lib"] = self.mod
        self.old_input = builtins.input; builtins.input = lambda *a, **k: (_ for _ in ()).throw(AssertionError("안전 질문이 나왔다"))
        self.old_argv = sys.argv; self.td = tempfile.TemporaryDirectory()
    def tearDown(self):
        builtins.input = self.old_input; sys.argv = self.old_argv; self.td.cleanup()
        if self.old is None: sys.modules.pop("Arm_Lib", None)
        else: sys.modules["Arm_Lib"] = self.old
    def plan(self, d):
        p = Path(self.td.name) / "plan.json"; p.write_text(json.dumps(d), encoding="utf-8"); return str(p)
    def run_main(self, plan_path, *flags):
        sys.argv = ["g01_p2k_test.py", "--plan", plan_path, *flags]; R.main()

    def test_real_set33_refused_with_zero_hardware_contact(self):
        with self.assertRaises(SystemExit) as c: self.run_main(self.plan({"set": "33"}), "--real")
        msg = str(c.exception); self.assertIn("세트 33", msg); self.assertIn("q3_box", msg); self.assertIn("lift_j2_max_target", msg)
        self.assertEqual((Rec.created, Rec.writes), (0, 0))

    def test_real_still_refused_when_plan_supplies_target_but_q3_unset(self):
        with self.assertRaises(SystemExit) as c: self.run_main(self.plan({"set": "33", "lift": {"j2_max_target": 77}}), "--real")
        self.assertNotIn("lift_j2_max_target", str(c.exception)); self.assertIn("q3_box", str(c.exception)); self.assertEqual((Rec.created, Rec.writes), (0, 0))

    def test_non_integer_target_in_plan_stays_unset(self):
        for bad in (None, 77.0, "77", True):
            self.assertIn("lift_j2_max_target", LP.unset_names(LP.params_for_plan({"set": "33", "lift": {"j2_max_target": bad}})), bad)
        self.assertNotIn("lift_j2_max_target", LP.unset_names(LP.params_for_plan({"set": "33", "lift": {"j2_max_target": 77}})))

    def test_fake_and_dry_run_pass_the_gate(self):
        for flag in (("--fake",), ()):
            with self.assertRaises(Exception) as c: self.run_main(self.plan({"set": "33"}), *flag)    # 게이트는 통과하고 이후 계획 키 부재로 실패
            self.assertNotIsInstance(c.exception, SystemExit); self.assertIsInstance(c.exception, KeyError)
        self.assertEqual(Rec.writes, 0)

    def test_lift_clause_without_set33_is_also_gated(self):
        with self.assertRaises(SystemExit) as c: self.run_main(self.plan({"set": "31", "lift": {"j2_max_target": 77}}), "--real")   # set이 33이 아니어도 lift 절이 있으면 거부(fail-open 방지)
        self.assertIn("q3_box", str(c.exception)); self.assertEqual((Rec.created, Rec.writes), (0, 0))
        self.assertTrue(LP.is_lift_plan({"set": "33"})); self.assertTrue(LP.is_lift_plan({"set": "31", "lift": {}})); self.assertFalse(LP.is_lift_plan({"set": "31"}))

    def test_other_sets_are_not_affected(self):
        with self.assertRaises(Exception) as c: self.run_main(self.plan({"set": "31"}), "--real")
        self.assertNotIsInstance(c.exception, SystemExit)                                             # 세트 33 게이트는 다른 세트에 적용하지 않는다

    def test_real_allowed_only_after_every_unset_is_filled(self):
        p = LP.params_for_plan({"set": "33", "lift": {"j2_max_target": 77}}); p["q3_box"] = (800, 0, 900, 100)
        with self.assertRaises(LP.ParamsNotFinal): LP.check_real_allowed("real", p)
        p["q3_arm_in_box_criterion"] = lambda r: False; LP.check_real_allowed("real", p)


if __name__ == "__main__": unittest.main()
