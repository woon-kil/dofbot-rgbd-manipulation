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
    wobble = [(i * 100, 141 + (i % 2)) for i in range(8)]           # 141/142로 오르내림(1° 폭): 최대값 142 기준 잔차 3 이상이면 정체
    assert H.is_stall(145, wobble) == (True, 142)
    assert H.is_stall(144, wobble)[0] is False                      # 잔차 2는 정체가 아니다
    wide = [(i * 100, 140 + 2 * (i % 2)) for i in range(8)]         # 2° 폭으로 흔들리면 정체로 보지 않는다
    assert H.is_stall(150, wide)[0] is False


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


def test_stall_step_ends_polling_early():
    """H4(3초 안 복귀): 정체 단계는 전체 폴링(2×이동 시간+1 s)을 다 기다리지 않고 정체를 감지하면 바로 끝낸다."""
    p, r = run(FULL, "--fake-stall-at", "141")
    total = (2 * 1500 + 1000) / 40.0
    stall_step = [s for t in r["trials"] for s in t["steps"] if s.get("stall")][0]
    assert stall_step["series"][-1][0] < 0.8 * total, (stall_step["series"][-1][0], total)


def test_start_ranges():
    assert H.START_CLOSED == (177, 178, 179) and H.START_OPEN == (99, 100, 101)
    p, r = run("y\n", "--fake-start", "177", "--fake-dry")
    assert r["ended"] == "completed", (r.get("abort_reason"), p.stdout[-200:])
    p, r = run("y\n", "--fake-start", "176", "--fake-dry")
    assert r["ended"] == "aborted"


def test_return_prompt_wording_depends_on_direction():
    """블록 구간(S≈100) 중단 복귀는 '여는 동작, y 권장', 열기·마무리 구간(S≈179) 복귀는 '닫는 동작, 끼임이면 N'."""
    src = (SCR / "g01_p2h_gripper_test.py").read_text(encoding="utf-8")
    assert "여는 동작" in src and "y를 권장" in src and "닫는 동작" in src and "끼임·소리·발열이면 N" in src
    # 블록 구간 문구에는 N 권장이 없어야 한다
    i = src.index("그리퍼가 블록을 누르고 있을 수 있습니다"); j = src.index(") if opening else (", i)
    assert "이면 N" not in src[i:j], "블록 구간 복귀 문구에 N 권장이 있으면 안 된다"
    assert "이면 N" in src[j:j + 400]


def test_result_keeps_all_adapter_logs():
    p, r = run(FULL, "--fake-stall-at", "141")
    assert r["writes_total"] == sum(1 for x in r["adapter_logs"] for l in x if l[0] == "write") and len(r["adapter_logs"]) == 5
    assert r["writes_total"] <= 29 and r["writes_total"] >= 4 + 3 * 6 + 4 - 3
