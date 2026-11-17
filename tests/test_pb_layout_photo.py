"""배치 사진용 J6 열기·대기·닫기 도구(scripts/g01_pb_layout_photo.py) 가짜 서보 시험. pytest 방식(함수형, unittest discover에는 잡히지 않는다)."""
import json, subprocess, sys, tempfile
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]; SCR = ROOT / "scripts"
sys.path.insert(0, str(SCR))
import g01_pb_layout_photo as L
import g01_5a_gripper_test as G5
import g01_p2h_gripper_test as H


def run(stdin, *extra, timeout=120):
    with tempfile.TemporaryDirectory() as d:
        out = Path(d) / "r.json"
        p = subprocess.run([sys.executable, "-I", str(SCR / "g01_pb_layout_photo.py"), "--fake", "--fake-speed", "40", "--out", str(out), *extra], input=stdin, capture_output=True, text=True, timeout=timeout, cwd=str(ROOT))
        return p, json.loads(out.read_text(encoding="utf-8")) if out.exists() else None


def writes(r): return [(l[1], l[2]) for x in r["adapter_logs"] for l in x if l[0] == "write"]


def test_normal_176_opens_waits_closes_8_writes():
    p, r = run("y\ny\ny\n", "--fake-start", "176")
    assert r["ended"] == "completed" and r["gate_close"] is True, p.stdout[-400:]
    assert r["writes_total"] == 8 and writes(r) == [(160, 3000), (140, 1500), (120, 1500), (100, 1500), (120, 1500), (140, 1500), (160, 1500), (179, 1500)], writes(r)
    assert [ph["phase"] for ph in r["phases"]] == ["open", "finish"]


def test_gate_default_n_leaves_open_no_close():
    p, r = run("y\ny\nn\n", "--fake-start", "176")
    assert r["ended"] == "left_open" and r["gate_close"] is False and r["writes_total"] == 4 and [ph["phase"] for ph in r["phases"]] == ["open"]


def test_gate_empty_answer_is_n():
    p, r = run("y\ny\n\n", "--fake-start", "176")
    assert r["ended"] == "left_open" and r["writes_total"] == 4


def test_input_closed_at_gate_does_not_close():
    p, r = run("y\ny\n", "--fake-start", "176")
    assert r["ended"] == "input_closed" and r["writes_total"] == 4 and max(a for _, a in [(0, w[0]) for w in writes(r)]) == 160


def test_start_outside_range_rejected_no_write():
    p, r = run("y\n", "--fake-start", "150")
    assert r["ended"] == "aborted" and r["writes_total"] == 0


def test_start_177_179_accepted_and_start_175_rejected():
    for s in (176, 177, 179): assert run("y\ny\nn\n", "--fake-start", str(s))[1]["writes_total"] == 4
    assert run("y\n", "--fake-start", "175")[1]["ended"] == "aborted"


def test_start_open_skips_open_and_only_closes():
    p, r = run("y\ny\ny\n", "--fake-start", "100")
    assert r["ended"] == "completed" and r["writes_total"] == 4 and r["phases"][0]["phase"] == "open_skipped" and writes(r)[-1] == (179, 1500)


def test_dry_run_writes_zero_and_prints_plan():
    p, r = run("y\n", "--fake-dry", "--fake-start", "176")
    assert r["mode"] == "dry_run" and r["ended"] == "completed" and r["writes_total"] == 0 and r["phases"][0]["open"] == [176, 160, 140, 120, 100]
    assert "[100, 120, 140, 160, 179]" in p.stdout


def test_pre_check_no_stops_without_write():
    p, r = run("n\n", "--fake-start", "176")
    assert r["ended"] == "aborted" and r["writes_total"] == 0


def test_constants_come_from_approved_modules():
    import g01_p2i_gripper_test as I, g01_pb_press_test as P
    assert L.OPEN_STEPS is I.OPEN_STEPS and L.FINISH_STEPS is I.FINISH_STEPS and L.START_CLOSED is P.START_CLOSED
    assert I.OPEN_STEPS == [160, 140, 120, 100] and I.FINISH_STEPS == [120, 140, 160, 179] and P.START_CLOSED == (176, 177, 178, 179)


def _inproc(monkeypatch, tmp_path, answers, exc, start=176):
    """게이트에서 Ctrl-C·신호가 들어오는 경우를 프로세스 안에서 재현한다."""
    it = iter(answers); H.ADAPTERS.clear()
    def fake_ask(msg, default=False):
        try: return next(it)
        except StopIteration: raise exc
    monkeypatch.setattr(L, "ask", fake_ask); out = tmp_path / "r.json"
    monkeypatch.setattr(sys, "argv", ["x", "--fake", "--fake-speed", "40", "--fake-start", str(start), "--out", str(out)])
    L.main(); return json.loads(out.read_text(encoding="utf-8"))


def test_ctrl_c_at_gate_does_not_close(monkeypatch, tmp_path):
    r = _inproc(monkeypatch, tmp_path, [True, True], KeyboardInterrupt())
    assert r["ended"] == "interrupted" and r["writes_total"] == 4 and [ph["phase"] for ph in r["phases"]] == ["open"]


def test_hangup_at_gate_does_not_close(monkeypatch, tmp_path):
    r = _inproc(monkeypatch, tmp_path, [True, True], G5.Hangup("신호 15"))
    assert r["ended"] == "hangup" and r["writes_total"] == 4


def test_ctrl_c_during_open_does_not_close(monkeypatch, tmp_path):
    n = {"c": 0}; real_poll = L.poll
    def bad_poll(*a, **k):
        n["c"] += 1
        if n["c"] == 2: raise KeyboardInterrupt()
        return real_poll(*a, **k)
    monkeypatch.setattr(L, "poll", bad_poll)
    r = _inproc(monkeypatch, tmp_path, [True, True, True], KeyboardInterrupt())
    assert r["ended"] == "interrupted" and r["writes_total"] <= 2 and "finish" not in [ph["phase"] for ph in r["phases"]] and not any(a == 179 for a, _ in writes(r))


if __name__ == "__main__":
    import traceback
    ok = fail = 0
    for n, f in list(globals().items()):
        if n.startswith("test_") and not f.__code__.co_varnames[:f.__code__.co_argcount]:
            try: f(); ok += 1; print("OK", n)
            except Exception: fail += 1; print("FAIL", n); traceback.print_exc()
    print("%d 통과, %d 실패 (monkeypatch 사용 시험은 pytest로 실행)" % (ok, fail)); sys.exit(1 if fail else 0)
