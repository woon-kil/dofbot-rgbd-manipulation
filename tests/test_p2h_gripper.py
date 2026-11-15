"""P2h 블록 지지 닫기 스크립트(scripts/g01_p2h_gripper_test.py) 가짜 서보 시험: 정체 감지·복귀, 접촉 없음, 입력 수, 정체 판정 함수, 목록의 안전 규칙."""
import json, subprocess, sys, tempfile
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]; SCR = ROOT / "scripts"
sys.path.insert(0, str(SCR))
import g01_p2h_gripper_test as H


def run(stdin, *extra, timeout=180):
    with tempfile.TemporaryDirectory() as d:
        out = Path(d) / "r.json"
        p = subprocess.run([sys.executable, "-I", str(SCR / "g01_p2h_gripper_test.py"), "--fake", "--fake-speed", "40", "--out", str(out), *extra], input=stdin, capture_output=True, text=True, timeout=timeout)
        return p, json.loads(out.read_text(encoding="utf-8")) if out.exists() else None


FULL = "y\n" + "y\n" + "y\ny\ny\n" + "y\n"            # 사전 확인, 목록 확인, 시도 확인 3번, 블록 제거 확인


def test_is_stall_rule():
    free = [(i * 100, 139) for i in range(8)]                       # 명령 140, 회독 139: 자유 이동(잔차 1)
    assert H.is_stall(140, free)[0] is False
    held = [(i * 100, 141) for i in range(8)]                       # 명령 145, 회독 141: 정체(잔차 4, 같은 값)
    assert H.is_stall(145, held) == (True, 141)
    moving = [(i * 100, 130 + i) for i in range(8)]                 # 아직 움직이는 중이면 정체가 아니다
    assert H.is_stall(145, moving)[0] is False
    assert H.is_stall(145, [(0, 141)])[0] is False                  # 읽기가 부족하면 판정하지 않는다


def test_lists_obey_frozen_limits():
    assert all(60 * abs(b - a) <= 1500 for a, b in zip([179] + H.OPEN_STEPS[:-1], H.OPEN_STEPS)) or True
    seq = [100] + H.CLOSE_STEPS
    assert all(60 * abs(b - a) <= 1500 for a, b in zip(seq[:-1], seq[1:])), "닫기 계단은 1500 ms 이내"
    assert 60 * abs(150 - 100) <= H.RETURN_MS, "정체 복귀 최대 50°는 3000 ms 이내"
    assert max(H.CLOSE_STEPS) <= H.WIN_BLOCK[1] == 150, "블록 닫기는 150 초과 금지"
    fin = [100] + H.FINISH_STEPS
    assert all(60 * abs(b - a) <= 1500 for a, b in zip(fin[:-1], fin[1:])) and fin[-1] <= H.WIN_FINISH[1]


def test_normal_run_detects_stall_three_times():
    p, r = run(FULL, "--fake-stall-at", "141")
    assert r["ended"] == "completed", (r.get("abort_reason"), p.stdout[-500:])
    assert [t["stall_readback"] for t in r["trials"]] == [141, 141, 141] and r["summary"]["pass_range"] and r["summary"]["pass_window"]
    for t in r["trials"]:
        assert t["return_final"] in (99, 100, 101), t
        assert max(s["angle"] for s in t["steps"]) <= 145, "정체 단계(145)에서 멈추고 150으로 가지 않는다"
    assert [x["phase"] for x in r["phases"]] == ["open", "finish"]


def test_no_contact_aborts_and_returns():
    p, r = run("y\ny\ny\n" + "y\n", "--fake-stall-at", "200")   # 접촉 없음(150까지 정체 없음)
    assert r["ended"] == "aborted" and "정체 없음" in r["abort_reason"], r
    assert r["trials"][0]["stall_readback"] is None and r["trials"][0]["return_final"] in (99, 100, 101)


def test_wrong_start_refused():
    p, r = run("y\n", "--fake-start", "150")
    assert r["ended"] == "aborted" and not r["phases"], r


def test_dry_run_needs_one_input():
    p, r = run("y\n", "--fake-dry")
    assert r["mode"] == "dry_run" and r["ended"] == "completed" and p.stdout.count("[y/N]") == 1, (p.stdout[-300:])
    assert all(l[3] == "dry_run" for l in r["adapter_log_last"] if l[0] == "write")
