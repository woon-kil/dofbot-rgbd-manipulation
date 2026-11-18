"""가상 시계 하네스(시험 전용): time.sleep이 시계를 앞으로 보내고 time.monotonic·time.time이 그 시계를 돌려준다. 실시간 대기가 많은 가짜 러너 흐름을 결정적으로 빠르게 돌린다."""
import contextlib, time


@contextlib.contextmanager
def virtual_clock(start=1000.0):
    real = (time.sleep, time.monotonic, time.time); t = [start]
    time.sleep = lambda d: t.__setitem__(0, t[0] + max(0.0, float(d)))
    time.monotonic = lambda: t[0]; time.time = lambda: 1.7e9 + t[0]
    try: yield t
    finally: time.sleep, time.monotonic, time.time = real


def run_runner_fake(plan_path, extra=()):
    """g01_p2k_test.main()을 가짜 팔·가상 시계로 실행하고 결과 JSON(dict)을 돌려준다(안전 질문은 모두 y). 시험 전용."""
    import builtins, json, sys, tempfile, os
    import g01_p2k_test as R
    old_in, old_argv = builtins.input, sys.argv; out = tempfile.mktemp(suffix=".json")
    builtins.input = lambda *a, **k: "y"
    sys.argv = ["g01_p2k_test.py", "--plan", str(plan_path), "--fake", "--run-id", "reg", "--out", out, *extra]
    R.PROG.update({"sent": 0, "return_started": False}); R.VISITS.clear(); R.RETRIES["n"] = 0
    R.P2K.update({"ad6": None, "start6": None, "cur6": None, "low": False, "contact": False, "logs": {}})
    try:
        with virtual_clock():
            R.main()
        return json.load(open(out, encoding="utf-8"))
    finally:
        builtins.input, sys.argv = old_in, old_argv
        if os.path.exists(out): os.remove(out)


def digest(result):
    """회귀 비교용 결정적 요약: 이동 목록·어댑터 쓰기 로그·접촉 분기 결과·종료 상태. 시각·경로는 제외한다."""
    def rnd(o):
        if isinstance(o, float): return round(o, 3)
        if isinstance(o, dict): return {k: rnd(v) for k, v in o.items()}
        if isinstance(o, (list, tuple)): return [rnd(x) for x in o]
        return o
    d = {"ended": result.get("ended"), "returned_to_start": result.get("returned_to_start"), "overall_ok": result.get("overall_ok"),
         "steps": [{k: s.get(k) for k in ("n", "joint", "target", "time_ms", "phase", "pose_id", "cmd_before", "final", "retried")} for s in result.get("steps", [])],
         "return_steps": [{k: s.get(k) for k in ("n", "joint", "target", "time_ms", "final")} for s in result.get("return_steps", [])],
         "adapter_log": result.get("adapter_log"), "j6_adapter_log": result.get("j6_adapter_log"), "j6_adapter_logs": result.get("j6_adapter_logs"),
         "poses": [{"pose_id": p.get("pose_id"), "commanded": p.get("commanded"), "readback": p.get("readback"), "run2": {k: v for k, v in (p.get("run2") or {}).items() if not k.startswith("hold_") and k != "series_tail"}} for p in result.get("poses", [])]}
    return rnd(d)
