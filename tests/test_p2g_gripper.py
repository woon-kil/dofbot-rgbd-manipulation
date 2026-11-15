"""P2g 그리퍼 단독 시험 스크립트(scripts/g01_p2g_gripper_test.py) 가짜 서보 시험: 정상 완주, 시작값 거부, 쓰기 누락, 반대 방향 간격 중단, 복귀 시간 규칙, 목록 구조."""
import json, subprocess, sys, tempfile
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]; SCR = ROOT / "scripts"
sys.path.insert(0, str(SCR))
import g01_p2g_gripper_test as G

SAFE = "y\n" + "왼쪽 안쪽 평판\n"          # 사전 확인 1회(5개 묶음) + 측정면 설명


def gap(angle):   # 가짜 캘리퍼: 각도가 작을수록 벌어짐(G01.5a 방향), 열림 쪽 대략 0.4 mm/°
    return 49.5 + (100 - angle) * 0.4 if angle <= 100 else 49.5 - (angle - 100) * 0.4


def cal(angle, noise=0.0): return "%.1f, %.1f, %.1f\n" % (gap(angle), gap(angle) + noise, gap(angle) - noise)


def run(stdin, *extra, timeout=120):
    with tempfile.TemporaryDirectory() as d:
        out = Path(d) / "r.json"
        p = subprocess.run([sys.executable, "-I", str(SCR / "g01_p2g_gripper_test.py"), "--fake", "--fake-speed", "30", "--out", str(out), *extra], input=stdin, capture_output=True, text=True, timeout=timeout)
        return p, json.loads(out.read_text(encoding="utf-8")) if out.exists() else None


def full_input(targets, bad_at=None):
    s = SAFE + "y\n" + cal(targets[0])             # 목록 확인 + 시작 캘리퍼
    for i, t in enumerate(targets[1:], 1):
        if bad_at == i:   # 예상과 반대: 열릴 때(각도 감소) 간격이 3 mm 줄고, 닫힐 때 3 mm 늘어난 값
            g = gap(targets[i - 1]) + (-3.0 if t < targets[i - 1] else 3.0); s += "y\n%.1f, %.1f, %.1f\n" % (g, g, g); break   # 중단 지점에서 입력을 끊어 이후 질문(복귀)이 남은 줄을 먹지 않게 한다
        else: s += "y\n" + cal(t)
    return s


def test_targets_and_times():
    t = G.build_targets(179)
    assert t == [179, 160, 140, 120, 110, 100, 110, 120, 140, 160, 179] and len(t) - 1 == 10
    assert G.times_for(t) == [3000] + [1500] * 9
    assert all(60 * abs(t[i + 1] - t[i]) <= ms for i, ms in enumerate(G.times_for(t))), "60 ms/° 상한"
    assert all(G.WINDOW[0] <= x <= G.WINDOW[1] for x in t)


def test_return_time_rule():
    assert G.return_time(79) == 4800 and G.return_time(19) == 1500 and G.return_time(30) == 3000
    assert G.return_time(80) == 4800 and G.return_time(81) is None, "81° 이상은 승인된 시간으로 복귀 불가"


def test_parse_caliper():
    assert G.parse_caliper("49.5, 49.6, 49.4") == ([49.5, 49.6, 49.4], 49.5, 0.2)
    assert G.parse_caliper("") is None


def test_full_fake_run_completes():
    t = G.build_targets(179)
    p, r = run(full_input(t))
    assert r["ended"] == "completed", (r.get("abort_reason"), p.stdout[-400:])
    assert [x["angle"] for x in r["steps"] if x["point"] == "move"] == t[1:] and len(r["gap_table"]) == 11
    writes = [l for l in r["adapter_log"] if l[0] == "write"]
    assert len(writes) == 10 and all(w[3] == "fake" for w in writes)


def test_refuses_wrong_start():
    p, r = run(SAFE + "y\n", "--fake-start", "150")
    assert r["ended"] in ("adapter_refused", "aborted") and not [l for l in r["adapter_log"] if l[0] == "write"], r


def test_reverse_gap_aborts_without_further_moves():
    t = G.build_targets(179)
    p, r = run(full_input(t, bad_at=2) + "n\n")        # 이동 2번 뒤 간격을 예상과 반대로 입력, 복귀는 N
    assert r["ended"] == "aborted" and "반대" in r["abort_reason"], r
    assert len([l for l in r["adapter_log"] if l[0] == "write"]) == 2


def test_missed_write_can_be_repeated_once():
    t = G.build_targets(179)
    s = SAFE + "y\n" + cal(t[0]) + "y\n"                 # 첫 이동 앞까지
    # 첫 이동(쓰기 1번)을 서보가 무시 → 재전송 y → 정상 도착
    s += "y\n" + cal(t[1])                               # 재전송 질문 y, 캘리퍼
    for tt in t[2:]: s += "y\n" + cal(tt)
    p, r = run(s, "--fake-nomove-at", "1")
    assert r["steps"][1].get("repeated") is True, (r["steps"][1], p.stdout[-300:])
    assert len([l for l in r["adapter_log"] if l[0] == "write"]) == 11


def test_abort_return_uses_allowed_time():
    t = G.build_targets(179)
    s = full_input(t, bad_at=5)                          # 이동 5(100)에서 반대 방향 간격 → 중단, 현재 100
    s += "y\n"                                           # 복귀 y
    p, r = run(s)
    assert r["ended"] == "aborted" and r.get("returned_to_start") is True and r["return_time_ms"] in (1500, 3000, 4800), r
    last = [l for l in r["adapter_log"] if l[0] == "write"][-1]
    assert last[1] == 179 and last[2] in (3000, 4800), last


def test_dry_run_needs_one_input_only():
    """사용자 요청(입력 최소화): dry run은 사전 확인 1회만 묻고 이동 앞에서 묻지 않는다(쓰기·캘리퍼 없음)."""
    p, r = run("y\n", "--fake-dry")
    assert r["mode"] == "dry_run" and r["ended"] == "completed", (r.get("abort_reason"), p.stdout[-300:])
    assert all(l[3] == "dry_run" for l in r["adapter_log"] if l[0] == "write") and len([x for x in r["steps"] if x["point"] == "dry_run_move"]) == 10
    assert p.stdout.count("[y/N]") == 1, p.stdout
