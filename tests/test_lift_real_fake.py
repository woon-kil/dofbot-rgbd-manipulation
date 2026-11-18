"""`g01_lift_run.sh --real` 전체 흐름 가짜 리허설(tests/harness/real_fake_rehearsal.py)을 실행한다. 약 6분이라 LIFT_SLOW=1일 때만(venv 필요)."""
import os, subprocess, sys, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get("LIFT_SLOW") == "1" and (ROOT / ".venv" / "bin" / "python").exists(), "LIFT_SLOW=1일 때만(약 6분)")
class RealFakeRehearsal(unittest.TestCase):
    def test_full_real_flow_with_fakes(self):
        r = subprocess.run([str(ROOT / ".venv" / "bin" / "python"), "-I", str(ROOT / "tests" / "harness" / "real_fake_rehearsal.py")], capture_output=True, text=True, timeout=1700)
        self.assertEqual(r.returncode, 0, r.stdout[-1500:])


if __name__ == "__main__": unittest.main()
