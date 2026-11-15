"""P2i 큰 명령 블록 있음/없음 비교 스크립트(scripts/g01_p2i_gripper_test.py) 가짜 서보 시험: 판정 규칙, 눌림 감지, 정상 완주, 입력 수, 목록의 안전 규칙."""
import json, subprocess, sys, tempfile
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]; SCR = ROOT / "scripts"
sys.path.insert(0, str(SCR))
import g01_p2i_gripper_test as I


def run(stdin, *extra, timeout=300):
    with tempfile.TemporaryDirectory() as d:
        out = Path(d) / "r.json"
        p = subprocess.run([sys.executable, "-I", str(SCR / "g01_p2i_gripper_test.py"), "--fake", "--fake-speed", "40", "--out", str(out), *extra], input=stdin, capture_output=True, text=True, timeout=timeout)
        return p, json.loads(out.read_text(encoding="utf-8")) if out.exists() else None


FULL = "y\n" + "y\n" + "y\ny\ny\n" + "y\n"            # 사전 확인, 목록 확인, 블록 시도 확인 3번, 블록 제거 확인


def test_decide_rules():
    free = {150: [149, 149], 155: [154, 154]}
    ok = I.decide(free, {150: [142, 142, 143], 155: [143, 143, 144]})
    assert ok["c_separation_ok"] and ok["a_block150_range_ok"] and ok["b_free150_in_147_150"] and "가능(" in ok["verdict"]
    weak = I.decide(free, {150: [146, 146, 147], 155: [147, 147, 147]})        # 차이 약 2.7° < 4°
    assert not weak["c_separation_ok"] and "가능하지 않음" in weak["verdict"]
    wide = I.decide(free, {150: [140, 143, 146], 155: [141, 144, 147]})        # 블록 있음 범위 6° > 2°
    assert not wide["a_block150_range_ok"] and "가능하지 않음" in wide["verdict"]
    odd = I.decide({150: [140, 141], 155: [150, 150]}, {150: [136, 136, 136], 155: [137, 137, 137]})   # 블록 없음 150이 147~150 밖
    assert not odd["b_free150_in_147_150"]


def test_crush_check_rules():
    assert I.crush_check([(i * 100, 141 + i // 2) for i in range(10)])[0] is True             # 정체 영역(141 이상)에서 같은 명령에서 계속 오름
    assert I.crush_check([(i * 100, 139 + i) for i in range(5)])[0] is False                   # 아직 블록에 다가가는 중(139→143)인 상승은 눌림이 아니다(검토 반영)
    assert I.crush_check([(i * 100, 142) for i in range(8)])[0] is False                       # 일정하면 눌림 아님
    assert I.crush_check([(i * 100, 148) for i in range(8)], ref150=142, cmd=155)[0] is True   # 155에서 150 정체보다 4° 넘게 큼
    assert I.crush_check([(i * 100, 145) for i in range(8)], ref150=142, cmd=155)[0] is False


def test_lists_obey_frozen_limits():
    seq = [100] + I.PRESS_STEPS
    assert all(60 * abs(b - a) <= 1500 for a, b in zip(seq[:-1], seq[1:])), "계단은 1500 ms 이내"
    assert max(I.PRESS_STEPS) == 155 == I.WIN_PRESS[1], "누르는 창 상한 155"
    assert 60 * (155 - 100) <= I.RETURN_MS, "155→100 복귀는 4800 ms 이내"
    fin = [100] + I.FINISH_STEPS
    assert all(60 * abs(b - a) <= 1500 for a, b in zip(fin[:-1], fin[1:])) and fin[-1] <= I.WIN_FINISH[1]
    assert (I.FREE_TRIALS, I.BLOCK_TRIALS) == (2, 3) and 4 + 2 * 5 + 3 * 5 + 4 == 33


def test_normal_run_separation_possible():
    p, r = run(FULL, "--fake-stall-at", "142")
    assert r["ended"] == "completed", (r.get("abort_reason"), p.stdout[-500:])
    s = r["summary"]; assert s["150"]["diff"] >= 4 and s["c_separation_ok"] and "가능(" in s["verdict"], s
    assert r["writes_total"] == 33 and len(r["adapter_logs"]) == 7 and len(r["free_trials"]) == 2 and len(r["block_trials"]) == 3
    for t in r["block_trials"] + r["free_trials"]: assert t["return_final"] in (99, 100, 101), t


def test_weak_separation_reported():
    p, r = run(FULL, "--fake-stall-at", "148")                                  # 블록이 148에서 막힘: 150에서 차이 약 2° → 판정 불가
    assert r["ended"] == "completed" and not r["summary"]["c_separation_ok"] and "가능하지 않음" in r["summary"]["verdict"], r["summary"]


def test_crush_aborts_and_returns():
    p, r = run("y\ny\ny\n" + "y\n", "--fake-stall-at", "142", "--fake-creep")
    assert r["ended"] == "aborted" and "눌림" in r["abort_reason"], (r.get("abort_reason"), p.stdout[-400:])
    t = r["block_trials"][0]; assert t["crush"] and t["return_final"] in (99, 100, 101)
    assert [s["angle"] for s in t["steps"]][-1] in (150, 155), "눌림 단계에서 멈춤"


def test_short_hold_only_for_pressing_steps():
    """I3의 한 단계 총 유지 2.0 s 제한은 블록을 누르는 150·155 단계에만 적용된다(검토 반영). 다른 단계는 정착하면 일찍 끝낸다."""
    p, r = run(FULL, "--fake-stall-at", "142")
    hold = (1500 + 500) / 40.0
    for t in r["block_trials"]:
        for s in t["steps"]:
            if s["angle"] >= 150: assert s["series"][-1][0] <= hold + 25, (s["angle"], s["series"][-1][0], hold)


def test_slow_servo_does_not_abort_open_or_finish():
    """실측: 1500 ms 명령이 3.2~3.9 s 걸린 이동이 있었다. 2배 느린 서보에서도 열기·마무리·블록 없음 구간이 도착 실패·접촉 의심으로 중단되지 않는다."""
    p, r = run(FULL, "--fake-stall-at", "142", "--fake-slow", "2.0")
    assert r["ended"] == "completed", (r.get("abort_reason"), p.stdout[-400:])
    assert r["summary"]["n_ok"] is True, r["summary"]


def test_dry_run_needs_one_input_only():
    p, r = run("y\n", "--fake-dry")
    assert r["mode"] == "dry_run" and r["ended"] == "completed" and p.stdout.count("[y/N]") == 1, p.stdout[-300:]


def test_wrong_start_refused():
    p, r = run("y\n", "--fake-start", "150")
    assert r["ended"] == "aborted" and not r["phases"], r


def test_early_contact_returns_automatically():
    """검토 반영: 블록 시도의 150 미만 단계(140)에서 조기 접촉(홀더 위치 차이)이 나면 질문 없이 열린 쪽으로 자동 복귀하고, 그 단계의 읽기는 전체 창보다 짧게 끝난다."""
    p, r = run("y\ny\ny\n" + "y\n", "--fake-stall-at", "136")
    assert r["ended"] == "aborted" and "조기 접촉" in r["abort_reason"], (r.get("abort_reason"), p.stdout[-500:])
    t = r["block_trials"][0]; assert t["early_contact"] and t["return_final"] in (99, 100, 101), t
    st = t["steps"][-1]; assert st["angle"] == 140 and st["final_readback"] == 136
    assert st["series"][-1][0] < 0.8 * ((2 * 1500 + 1000) / 40.0), st["series"][-1][0]
    assert r.get("returned_to_start") is None, "자동 복귀 뒤 복귀 질문이 다시 나오지 않는다"


def test_start_open_skips_open_phase():
    """이전 시험이 중단돼 100에 남은 경우: 열기 구간을 생략하고 블록 없음부터 시작한다(쓰기 29회)."""
    p, r = run(FULL, "--fake-start", "100", "--fake-stall-at", "142")
    assert r["ended"] == "completed", (r.get("abort_reason"), p.stdout[-400:])
    assert r["phases"][0]["phase"] == "open_skipped" and r["writes_total"] == 33 - 4, r["writes_total"]


def test_block_placed_early_is_detected_returns_automatically_and_steps_are_kept():
    """P2i 1차 실행의 실수(블록을 일찍 둠): 블록 없음 구간에서 150 회독이 막히면 **질문 없이 자동으로 열린 쪽 복귀**하고(검토 반영), 읽은 단계는 결과에 남는다."""
    p, r = run("y\ny\n", "--fake-block-early", "146")
    assert r["ended"] == "aborted" and "블록 없음" in r["abort_reason"] and "열린 쪽으로 자동 복귀" in r["abort_reason"], (r.get("abort_reason"), p.stdout[-300:])
    assert p.stdout.count("[y/N]") == 2, "사전 확인·목록 확인 두 번뿐이다(복귀 질문이 나오면 안 된다): " + p.stdout[-300:]
    t = r["free_trials"][0]; assert t["suspect"] and t["return_final"] in (99, 100, 101), t
    st = t["steps"][-1]; assert st["angle"] == 150 and st["final_readback"] == 146
    assert st["series"][-1][0] < ((2.7 * 1500 + 600) / 40.0) - 2, st["series"][-1][0]   # 접촉 의심은 이동 시간의 2.7배(약 101 ms 스케일) 뒤 판정, 전체 창 끝보다 먼저 끝난다
    kept = [x for pt in r["partial_steps"] for x in pt["steps"]]; assert any(x["angle"] == 150 and x["final_readback"] == 146 for x in kept)
    assert r.get("returned_to_start") is None


def test_block_timing_messages_present():
    src = (SCR / "g01_p2i_gripper_test.py").read_text(encoding="utf-8")
    assert "블록을 아직 두지 마세요" in src and "지금 블록을 홀더에 고정" in src
