"""세트 33 래퍼(g01_lift_run.sh) 시험: 구문, 세트 31 래퍼의 33 거부, 간격 검증 명령의 세트 31 일치, 배포 목록의 import 폐포, 미확정 상수의 조기 거부(가짜 ssh·scp, 쓰기·사진·P-A 0회).
venv 전용(계획 생성이 numpy·scipy 필요). 느린 간격 검증 전체 실행은 LIFT_SLOW=1일 때만."""
import json, os, re, shutil, subprocess, sys, tempfile, time, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
W33 = ROOT / "scripts" / "g01_lift_run.sh"; W31 = ROOT / "scripts" / "g01_p2k_run.sh"
VENV_PY = ROOT / ".venv" / "bin" / "python"
START = "88,92,92,91,90,176"          # tests/fixtures/s4_2_plan_31_fixture.json의 시작 회독
t33, t31 = W33.read_text(encoding="utf-8"), W31.read_text(encoding="utf-8")


def line_with(text, needle):
    ls = [l for l in text.splitlines() if needle in l]; assert len(ls) == 1, (needle, len(ls)); return ls[0].strip()


class TestStatic(unittest.TestCase):
    def test_syntax(self):
        for w in (W33, W31): self.assertEqual(subprocess.run(["bash", "-n", str(w)]).returncode, 0, w)

    def test_set31_wrapper_rejects_33(self):
        r = subprocess.run(["bash", str(W31), "--set", "33", "--check"], capture_output=True, text=True, env=dict(os.environ, JETSON_HOST="stub"))
        self.assertEqual(r.returncode, 2); self.assertIn("--set 25", r.stdout)                               # 허용 목록 case 25|28|29|31이 33을 거부한다(조용히 안전 검사가 빠지지 않음)
        self.assertRegex(t31, r'case "\$\{SET\}" in 25\|28\|29\|31\) ;;')

    def test_clearance_commands_identical_to_set31(self):
        for needle in ('P1_CAP_OPEN_ONLY_MM=30.9', 'RUN2_GATE_MM=2.0', 'P1_GATE_MM=10 P1_DZ_EXTRA_MM=35.8'):
            self.assertEqual(line_with(t33, needle), line_with(t31, needle), needle)
        self.assertIn('BXS="7.4 13.4"', t33); self.assertIn('[ "${SET}" = 31 ] && BXS="7.4 13.4"', t31)         # 세트 31의 BXS 값과 같은 값
        self.assertIn('TSET=32', t33); self.assertIn('[ "${SET}" = 31 ] && TSET=32', t31)
        for needle in ('p2kref_', 'assert d <= 1.0'): self.assertTrue(needle in t33 and needle in t31, needle)    # 기준 사진·마커 관문이 같은 문턱으로 있다
        for needle in ('Arm_Lib_ok', '팔 제어 관련 프로세스', '[ -t 0 ] && [ -t 1 ]'): self.assertTrue(needle in t33 and needle in t31, needle)

    def test_deploy_list_covers_runner_import_closure(self):
        """러너(Jetson)가 import하는 프로젝트 모듈이 모두 래퍼 scp 목록에 있다(없으면 실기기에서 게이트 대신 import 실패)."""
        scp_line = [l for l in t33.splitlines() if l.startswith("scp") and "g01_p2k_test.py" in l][0]
        shipped = set(re.findall(r"scripts/([A-Za-z0-9_]+\.py)", scp_line))
        seen, todo = set(), ["g01_p2k_test.py"]
        while todo:
            f = todo.pop()
            if f in seen: continue
            seen.add(f); src = (ROOT / "scripts" / f).read_text(encoding="utf-8")
            for m in re.finditer(r"^\s*(?:from|import)\s+((?:g01_|gripper_adapter)\w*)", src, re.M): todo.append(m.group(1) + ".py")
        runner_side = {f for f in seen if f != "g01_p2k_test.py"} | {"g01_p2k_test.py"}
        self.assertLessEqual(runner_side - {"g01_s42_plan_lift.py"}, shipped | {"g01_s42_plan_lift.py"}, runner_side - shipped)
        for f in ("g01_lift_params.py", "g01_lift_logic.py", "g01_lift_hook.py"): self.assertIn(f, shipped)

    def test_set_keyed_clearance_script_knows_set33(self):
        """run2 간격 스크립트는 plan['set']으로 J1을 고른다. '33'이 빠지면 87.4로 계산되어 거짓 미달(-0.3 mm)이 났다(2026-11-18 실측)."""
        src = (ROOT / "evidence" / "G06" / "run2_contact_clearance_261117.py").read_text(encoding="utf-8")
        self.assertIn("in ('31', '33') else 87.4", src)

    def test_r8_and_classifier_called_and_listed(self):
        for f in ("g01_lift_r8.py", "g01_lift_classify.py"): self.assertIn("scripts/" + f, t33)
        self.assertIn('g01_lift_r8.py" "${OUT}" --ts "${TS}" --log "${BLOG}"', t33); self.assertIn("sha=hashlib.sha256", t33)                    # 기준 사진 해시를 기준 JSON에 싣는다

    def test_no_wildcard_delete_and_bridge_not_deployed_to_jetson(self):
        self.assertNotRegex(t33, r"rm -[a-z]*f?[^\n]*\*")                                                       # 와일드카드 삭제 금지
        self.assertNotIn("g01_lift_bridge.py\" \"${ROOT}", "".join(l for l in t33.splitlines() if l.startswith("scp")))   # 다리는 Mac 쪽이다


@unittest.skipUnless(VENV_PY.exists(), "venv 필요")
class TestEarlyReject(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp(); self.log = Path(self.d) / "calls.log"; bindir = Path(self.d) / "bin"; bindir.mkdir()
        (bindir / "ssh").write_text('#!/bin/bash\necho "ssh $*" >> "%s"\ncase "$*" in *"import Arm_Lib"*) echo Arm_Lib_ok;; esac\nexit 0\n' % self.log)
        (bindir / "scp").write_text('#!/bin/bash\necho "scp $*" >> "%s"\nexit 0\n' % self.log)
        (bindir / "afplay").write_text('#!/bin/bash\nexit 0\n'); (bindir / "python3").write_text('#!/bin/bash\nexec "%s" "$@"\n' % VENV_PY); os.chmod(bindir / "python3", 0o755)        # 계획 생성의 python3 = venv(numpy·scipy)
        for f in ("ssh", "scp", "afplay"): os.chmod(bindir / f, 0o755)
        self.env = dict(os.environ, PATH="%s:%s" % (bindir, os.environ["PATH"]), JETSON_HOST="stub")

    def tearDown(self): shutil.rmtree(self.d, ignore_errors=True)

    def test_real_with_unset_exits_before_any_write_photo_or_pa(self):
        # 가짜 ssh는 회독 읽기에 빈 값을 주므로 계획 생성 전에 멈춘다 → 회독을 주는 가짜로 교체
        (Path(self.d) / "bin" / "ssh").write_text('#!/bin/bash\necho "ssh $*" >> "%s"\ncase "$*" in *"import Arm_Lib"*) echo Arm_Lib_ok;; *"Arm_serial_servo_read"*) echo %s;; esac\nexit 0\n' % (self.log, START))
        shim = Path(self.d) / "bin" / "pyshim"          # 거부 검사(check_real_allowed)만 실패시키고 나머지는 venv python 그대로(P9 등록 뒤 실제 UNSET이 없으므로 래퍼의 거부 경로를 이렇게 확인)
        shim.write_text('#!/bin/bash\ncase "$*" in *check_real_allowed*) echo "q3_box"; exit 1;; esac\nexec "%s" "$@"\n' % VENV_PY); os.chmod(shim, 0o755)
        r = subprocess.run(["bash", str(W33), "--real"], capture_output=True, text=True, env=dict(self.env, P1_CHECK_PY=str(shim)), cwd=str(ROOT), timeout=120)
        self.assertEqual(r.returncode, 8, r.stdout[-600:] + r.stderr[-600:]); self.assertIn("미확정 상수(UNSET)가 남아 --real을 거부한다", r.stdout)
        calls = self.log.read_text(encoding="utf-8")
        self.assertNotIn("scp ", calls); self.assertNotIn("p620", calls); self.assertNotIn("mkdir", calls); self.assertNotIn("g01_p2k_test.py", calls)   # 배포·P-A·러너 0회
        self.assertNotIn("기준 사진", r.stdout); self.assertNotIn("[y/N]", r.stdout)                                                                    # 사진·질문 0회

    def test_plan_check_and_real_cannot_combine(self):
        r = subprocess.run(["bash", str(W33), "--plan-check", "--real", "--start", START], capture_output=True, text=True, env=self.env, cwd=str(ROOT)); self.assertEqual(r.returncode, 2)
        r = subprocess.run(["bash", str(W33), "--plan-check"], capture_output=True, text=True, env=self.env, cwd=str(ROOT)); self.assertEqual(r.returncode, 2)

    def _until_marker(self, args, marker, tlimit=90):
        (Path(self.d) / "bin" / "ssh").write_text('#!/bin/bash\ncase "$*" in *"import Arm_Lib"*) echo Arm_Lib_ok;; *"Arm_serial_servo_read"*) echo %s;; esac\nexit 0\n' % START)
        p = subprocess.Popen(["bash", str(W33), *args], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=self.env, cwd=str(ROOT), start_new_session=True)
        seen = ""; t0 = time.time()
        try:
            while time.time() - t0 < tlimit:
                l = p.stdout.readline()
                if not l: break
                seen += l
                if marker in l: break
        finally:
            import signal
            try: os.killpg(p.pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError): pass
            p.wait()
        return seen

    def test_dry_run_and_real_pass_the_reject_point_when_nothing_is_unset(self):
        """양성 대조: 계획이 최대 목표를 공급하고 P9가 등록되어 UNSET이 없으면 dry_run도 --real도 이 지점에서 거부되지 않는다(이후는 느려서 표시를 본 뒤 종료)."""
        for args in ((), ("--real",)):
            seen = self._until_marker(args, "미확정 상수(UNSET):")
            self.assertIn("미확정 상수(UNSET): 없음", seen); self.assertNotIn("거부한다", seen)

@unittest.skipUnless(os.environ.get("LIFT_SLOW") == "1" and VENV_PY.exists(), "LIFT_SLOW=1일 때만(간격 검증 전체 수 분)")
class TestSlowPlanCheck(unittest.TestCase):
    def test_plan_check_passes_for_set33(self):
        r = subprocess.run(["bash", str(W33), "--plan-check", "--start", START], capture_output=True, text=True, cwd=str(ROOT), env=dict(os.environ, PATH="%s:%s" % (VENV_PY.parent, os.environ["PATH"])), timeout=900)
        self.assertEqual(r.returncode, 0, r.stdout[-800:] + r.stderr[-800:]); self.assertIn("계획·간격 검증 통과", r.stdout)


if __name__ == "__main__": unittest.main()
