"""P-B J6 연속 누름 스크립트(scripts/g01_pb_press_test.py) 가짜 서보 시험: 판정 규칙, 정상 사다리, 눌림, E3, None, PressBudget 상한, 사용자 이상 보고, dry run."""
import json, subprocess, sys, tempfile
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]; SCR = ROOT / "scripts"
sys.path.insert(0, str(SCR))
import g01_pb_press_test as P


def run(stdin, *extra, timeout=300):
    with tempfile.TemporaryDirectory() as d:
        out = Path(d) / "r.json"
        p = subprocess.run([sys.executable, "-I", str(SCR / "g01_pb_press_test.py"), "--fake", "--fake-speed", "40", "--out", str(out), *extra], input=stdin, capture_output=True, text=True, timeout=timeout, cwd=str(ROOT))
        return p, json.loads(out.read_text(encoding="utf-8")) if out.exists() else None


FULL = "y\n" + "y\n" + "y\n" + "y\ny\ny\ny\n" + "y\n"    # 사전 확인, 목록 확인, 블록 확인 1회, 시도 직후 발열 질문 4번(대조+3), 블록 제거


def test_hold_check_rules():
    assert P.hold_check(146, [146, 147, 146, 147, 146])[0] == "ok"                 # Run 3의 146↔147 흔들림은 정상
    assert P.hold_check(146, [146, 146, 147, 147, 148])[0] == "crush"              # 단조 증가, 정체값 +2
    assert P.hold_check(146, [146, 146, 146, 147, 147])[0] == "ok"                 # +1은 눌림 아님
    assert P.hold_check(146, [146, 147, 146])[0] == "ok"                           # 읽기 5회 미만
    assert P.hold_check(146, [146, None])[0] == "read_none"
    assert P.e3_range([146, 147, 146]) == 1 and P.e3_range([146, 148, 146]) == 2 and P.e3_range([None]) is None


def test_limits_obey_frozen_values():
    seq = [100] + P.CLOSE_STEPS
    assert all(60 * abs(b - a) <= 1500 for a, b in zip(seq[:-1], seq[1:]))
    assert max(P.CLOSE_STEPS) == P.WIN_PRESS[1] == 150 and P.WIN_PRESS[0] == 95
    assert P.LADDER_S == [10, 20, 30] and P.CONTROL_S == 10 and P.REST_S == 90 and P.CRUSH_RISE == 2 and P.E3_MAX == 1
    assert 60 * (150 - 100) <= P.RETURN_MS == 3000
    assert P.SETTLE_S == 2.0 and P.BUDGET_EXTRA_S == 1.0


def test_normal_ladder_completes():
    p, r = run(FULL, "--fake-stall-at", "146")
    assert r["ended"] == "completed" and r["ladder_stop"] is None, (r.get("abort_reason"), p.stdout[-600:])
    assert [t["T_s"] for t in r["trials"]] == [10, 10, 20, 30] and [t["block"] for t in r["trials"]] == [False, True, True, True]
    for t in r["trials"]:
        assert t["return_final"] in (99, 100, 101) and t["abort"] is None and not t["e3_exceeded"], t
    assert all(t["stall_step"] == 150 for t in r["trials"][1:]) and r["trials"][0]["stall_step"] is None
    assert r["writes_total"] == 4 + 4 * 7 + 4 and len(r["adapter_logs"]) == 1 + 4 + 1


def test_crush_opens_immediately_and_aborts():
    p, r = run("y\ny\ny\ny\ny\n", "--fake-stall-at", "146", "--fake-creep")
    assert r["ended"] == "aborted" and "정체값" in r["abort_reason"], r.get("abort_reason")
    t = r["trials"][-1]; assert t["crush"] and t["return_final"] in (99, 100, 101) and t["block"], t


def test_e3_exceeded_stops_ladder_after_opening():
    p, r = run(FULL, "--fake-stall-at", "146", "--fake-wobble", "2")
    assert r["ended"] == "completed" and r["ladder_stop"] and "E3 초과" in r["ladder_stop"], (r.get("abort_reason"), r["ladder_stop"])
    blk = [t for t in r["trials"] if t["block"]]; assert len(blk) == 1 and blk[0]["e3_exceeded"] and blk[0]["return_final"] in (99, 100, 101)


def test_read_none_opens_and_aborts():
    p, r = run("y\ny\ny\ny\ny\n", "--fake-stall-at", "146", "--fake-none-after", "5")
    assert r["ended"] == "aborted" and "None" in r["abort_reason"], r.get("abort_reason")
    assert r["trials"][-1]["return_final"] in (99, 100, 101)


def test_press_budget_backstop():
    p, r = run("y\ny\ny\ny\ny\n", "--fake-stall-at", "146", "--fake-no-hold-end")
    assert r["ended"] == "aborted" and "총 누름 시간" in r["abort_reason"], r.get("abort_reason")
    t = r["trials"][-1]; assert t["hold"]["used_s_scaled"] <= t["T_s"] + P.BUDGET_EXTRA_S + 1.0, t["hold"]["used_s_scaled"]   # 실제 시간 오차 여유 1 s
    assert t["return_final"] in (99, 100, 101)


def test_user_reports_problem_stops_ladder():
    p, r = run("y\ny\ny\ny\nn\ny\n", "--fake-stall-at", "146")       # 대조 y, 블록 확인 y, 블록 10 s 뒤 n
    assert r["ended"] == "completed" and r["ladder_stop"] and "사용자 이상 보고" in r["ladder_stop"] and len([t for t in r["trials"] if t["block"]]) == 1, (r["ladder_stop"], p.stdout[-400:])


def test_dry_run_writes_nothing():
    p, r = run("y\n", "--fake-dry")
    assert r["ended"] == "completed" and r["writes_total"] == 0 and r["phases"][0]["phase"] == "dry_run_plan"


def test_early_contact_below_150_opens_and_aborts():
    p, r = run("y\ny\ny\ny\ny\n", "--fake-stall-at", "135", "--no-control")   # 135에서 막힘: 150 미만 정체 → 누르지 않고 열기
    assert r["ended"] == "aborted" and "조기 접촉" in r["abort_reason"], r.get("abort_reason")
    t = r["trials"][-1]; assert t["hold"] is None and t["return_final"] in (99, 100, 101), t
    assert all(s["angle"] <= 140 for s in t["steps"]), [s["angle"] for s in t["steps"]]


def test_block_not_touching_does_not_press():
    p, r = run("y\ny\ny\ny\ny\n", "--no-control")                                # 블록 있음인데 막히지 않음(블록 미접촉)
    assert r["ended"] == "aborted" and "블록 미접촉" in r["abort_reason"], r.get("abort_reason")
    t = r["trials"][-1]; assert t["hold"] is None and t["return_final"] in (99, 100, 101), t


def test_total_press_not_over_T_plus_backstop_in_normal_run():
    p, r = run(FULL, "--fake-stall-at", "146")
    for t in r["trials"]:
        assert t["hold"]["used_s_scaled"] <= t["T_s"] + P.BUDGET_EXTRA_S + 1.0, (t["label"], t["hold"]["used_s_scaled"])   # 실제 시간 환산 오차 1 s 여유


def test_open_write_dropped_once_recovers_by_resend():
    p, r = run(FULL, "--fake-stall-at", "146", "--fake-drop-open", "1")
    assert r["ended"] == "completed" and r["open_retry"] == 1, (r.get("abort_reason"), p.stdout[-500:])
    assert all(t["return_final"] in (99, 100, 101) for t in r["trials"]) and r["trials"][0]["return_final"] in (99, 100, 101)


def test_open_unconfirmed_stops_everything():
    p, r = run("y\ny\ny\ny\ny\ny\ny\ny\n", "--fake-stall-at", "146", "--fake-drop-open", "2")
    assert r["ended"] == "open_unconfirmed" and r["open_unconfirmed"] and "열기 미확인" in r["abort_reason"], (r["ended"], r.get("abort_reason"))
    assert len(r["trials"]) == 1 and r.get("returned_to_start") is not True, "다음 시도·휴식·복귀 질문 없이 멈춰야 한다"
    assert "손을 넣지 마세요" in p.stdout


def test_sigterm_during_press_with_dropped_open_prints_emergency():
    import signal, time as _t
    with tempfile.TemporaryDirectory() as d:
        out = Path(d) / "r.json"
        pr = subprocess.Popen([sys.executable, "-I", "-u", str(SCR / "g01_pb_press_test.py"), "--fake", "--fake-speed", "5", "--fake-stall-at", "146", "--fake-drop-open", "2", "--no-control", "--out", str(out)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=str(ROOT))
        pr.stdin.write("y\ny\ny\ny\ny\ny\n"); pr.stdin.flush()
        seen = []; t0 = _t.time(); sent = False
        for line in pr.stdout:
            seen.append(line)
            if not sent and "이동 6/6: 150" in line: pr.send_signal(signal.SIGTERM); sent = True
            if _t.time() - t0 > 120: pr.kill(); break
        pr.wait(timeout=60); txt = "".join(seen)
        r = json.loads(out.read_text(encoding="utf-8"))
        assert sent and r["ended"] == "hangup", (r["ended"], txt[-600:])
        assert r.get("open_unconfirmed") is True and "손을 넣지 마세요" in txt and "복귀 미확인" in txt, txt[-800:]


def test_closing_return_unconfirmed_is_not_resent():
    import signal, time as _t
    with tempfile.TemporaryDirectory() as d:
        out = Path(d) / "r.json"
        pr = subprocess.Popen([sys.executable, "-I", "-u", str(SCR / "g01_pb_press_test.py"), "--fake", "--fake-speed", "5", "--fake-drop-close", "2", "--out", str(out)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=str(ROOT))
        pr.stdin.write("y\ny\ny\ny\n"); pr.stdin.flush()
        seen = []; t0 = _t.time(); sent = False
        for line in pr.stdout:
            seen.append(line)
            if not sent and "[열기] 이동 2/4" in line: pr.send_signal(signal.SIGTERM); sent = True
            if _t.time() - t0 > 120: pr.kill(); break
        pr.wait(timeout=60); txt = "".join(seen); r = json.loads(out.read_text(encoding="utf-8"))
        assert sent and r.get("closing_unconfirmed") is True and "닫는 쪽 복귀 미확인" in txt, txt[-800:]
        assert sum(1 for x in r["adapter_logs"] for l in x if l[0] == "write" and l[1] == 179) == 1, "닫는 쓰기는 한 번만 보내야 한다"
        assert r.get("returned_to_start") is not True


def test_stall_at_145_is_normal_contact_and_continues():
    p, r = run("y\ny\ny\ny\ny\ny\ny\ny\n", "--fake-stall-at", "142", "--no-control")   # P2h처럼 145에서 142로 정체
    assert r["ended"] == "completed" and r["trials"][0]["stall_step"] == 145, (r.get("abort_reason"), r["trials"][0].get("stall_step"))
    assert r["trials"][0]["abort"] is None and r["trials"][0]["hold"] is not None and not r["trials"][0]["steps"][3].get("early_contact")


def test_early_below_constant_is_145():
    assert P.EARLY_BELOW == 145
