"""스냅샷 도구 이름 검증(T7). main()에 --help를 넘기지 않는다(저장소에 폴더를 만들 수 있다): 검증 함수만 단위 시험한다."""
import importlib.util, sys, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("make_lift_snapshot", ROOT / "tools" / "make_lift_snapshot.py"); M = importlib.util.module_from_spec(spec); spec.loader.exec_module(M)


class SnapshotNameTest(unittest.TestCase):
    def test_rejects_option_like_and_unsafe_names(self):
        for bad in ("--help", "-x", "", "a/b", "a b", "..", "a..b", "../x", "x\n"):
            with self.assertRaises(SystemExit, msg=repr(bad)): M.check_name(bad)

    def test_accepts_normal_names(self):
        for ok in ("261119-lift2", "lift1", "a.b_c-1"): self.assertEqual(M.check_name(ok), ok)

    def test_main_validates_before_creating_anything(self):
        self.assertIn("check_name(sys.argv[1])", (ROOT / "tools" / "make_lift_snapshot.py").read_text(encoding="utf-8"))   # 이름 검증이 폴더 생성보다 앞(소스 확인)


if __name__ == "__main__": unittest.main()
