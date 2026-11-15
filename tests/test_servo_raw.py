"""러너의 원시 서보 값 환산(raw_to_deg)이 Yahboom Arm_Lib 1.0.3 `Arm_serial_servo_read`의 환산·절삭과 이어지는지 확인한다.
러너(scripts/g01_5b4_s42_test.py)는 gitignore라 없으면 건너뛴다. 기준 식은 data/refs/yahboom-Arm_Lib-1.0.3-Orin.py 189~215행(원시 900~3100 = 0~180°, int() 절삭, J2~J4는 180 − 값)."""
import importlib.util
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "scripts/g01_5b4_s42_test.py"


def lib_read(pos, jid):   # Arm_Lib 1.0.3의 환산 그대로(범위 밖은 None)
    p = int(180 * (pos - 900) / (3100 - 900))
    if p > 180 or p < 0: return None
    return 180 - p if jid in (2, 3, 4) else p


@unittest.skipUnless(RUNNER.exists(), "러너 없음(gitignore)")
class RawTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(RUNNER.parent))
        spec = importlib.util.spec_from_file_location("s42runner", RUNNER); cls.m = importlib.util.module_from_spec(spec); spec.loader.exec_module(cls.m)

    def test_truncation_relation(self):   # 정수 회독은 원시 환산값의 절삭이다: J1은 [정수, 정수+1), J2~J4는 (정수−1, 정수]
        for jid in (1, 2, 3, 4):
            for raw in range(900, 3101, 7):
                lib = lib_read(raw, jid); deg = self.m.raw_to_deg(raw, jid)
                if jid in (2, 3, 4): self.assertTrue(lib - 1 < deg <= lib + 1e-9, (jid, raw, lib, deg))
                else: self.assertTrue(lib <= deg + 1e-9 and deg < lib + 1, (jid, raw, lib, deg))

    def test_resolution(self):            # 원시 카운트 1개 = 약 0.082°
        self.assertAlmostEqual(self.m.raw_to_deg(1001, 1) - self.m.raw_to_deg(1000, 1), 180 / 2200, places=3)

    def test_none(self):
        self.assertIsNone(self.m.raw_to_deg(None, 3))

    def test_fake_arm_roundtrip(self):    # 가짜 팔의 원시 값을 환산하면 위치로 돌아온다(0.1° 이내)
        arm = self.m.FakeArmS({i: 90 for i in range(1, 7)}); arm.pos[3] = 37.0
        self.assertLess(abs(self.m.raw_to_deg(self.m.read_raw(arm, 3), 3) - 37.0), 0.1)

    def test_read_raw_failure_returns_none(self):   # 가짜 아닌 객체에서 속성이 없으면 예외 없이 None
        class Dummy: pass
        self.assertIsNone(self.m.read_raw(Dummy(), 2))


if __name__ == "__main__":
    unittest.main()
