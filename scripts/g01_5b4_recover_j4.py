#!/usr/bin/env python3
"""G01.5b-4 S3 이슈 후속: J4만 직립(시작값)으로 단독 복귀 쓰기 1회(8000 ms). 쓰기 직후 J4가 움직이기 시작하는지 읽어 확인하고, 반응이 없으면 추가 쓰기 없이 멈춘다.
사람이 입회해 직접 실행(무인 실행 금지). --fake로 가짜 서보 시험. 쓰기는 서보 id 4에 1회뿐(J1~J3·J5·그리퍼 읽기만).
사용: python3 g01_5b4_recover_j4.py [--real] [--target 91]   (기본 dry_run: 읽기만)
"""
import argparse, builtins, datetime, json, signal, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gripper_adapter import GripperTestAdapter, GripperTestError
from g01_5b4_joint_test import FakeArm, Hangup, _on_signal, read_valid, last_valid

TIME_MS = 8000; COUNTDOWN_S = 3; POLL_MS = 100; REACT_WAIT_MS = 3000; TOL = 3; OTHER_TOL = 3; MAX_CONSEC_NONE = 3
def say(*a, **k): builtins.print(*a, **k, flush=True)
def inp(p): return builtins.input("\a" + p)
def ask(msg): return inp(msg + " [y/N] ").strip().lower() in ("y", "yes")
class Abort(Exception): pass


class FakeArm3(FakeArm):
    def __init__(self, pos, ignore_ids=()): super().__init__(pos); self.ignore = set(ignore_ids)
    def Arm_serial_servo_write(self, id, angle, time_ms):
        if id in self.ignore: self.writes.append((id, angle, time_ms)); return   # 쓰기를 받고도 무시(미응답 시험)
        super().Arm_serial_servo_write(id, angle, time_ms)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--fake", action="store_true"); ap.add_argument("--real", action="store_true"); ap.add_argument("--target", type=int, default=91)
    ap.add_argument("--fake-s4", type=int, default=72); ap.add_argument("--fake-ignore4", action="store_true"); ap.add_argument("--run-id", default="local"); ap.add_argument("--out", default="/tmp/g01_5b4_recover_result.json"); a = ap.parse_args()
    if a.fake and a.real: sys.exit("--fake와 --real은 함께 쓸 수 없다")
    mode = "fake" if a.fake else ("real" if a.real else "dry_run")
    result = {"run_id": a.run_id, "mode": mode, "stage": "G01.5b-4 S3 이슈 후속 J4 단독 복귀", "target": a.target, "time_ms": TIME_MS, "started_at": datetime.datetime.now().isoformat(timespec="seconds"), "state0": None, "series": [], "ended": None}
    ad = None; wrote = False
    for sg in (signal.SIGHUP, signal.SIGTERM): signal.signal(sg, _on_signal)
    try:
        say("G01.5b-4 S3 후속: J4 단독 복귀(서보 4에 쓰기 1회, %d ms, 모드 %s). 다른 서보는 읽기만." % (TIME_MS, mode))
        arm = FakeArm3({1: 90, 2: 91, 3: 91, 4: a.fake_s4, 5: 90, 6: 99}, (4,) if a.fake_ignore4 else ()) if a.fake else __import__("Arm_Lib").Arm_Device()
        state = {i: read_valid(arm, i) for i in range(1, 7)}; result["state0"] = state
        say("현재 회독(SDK 각도): " + ", ".join("J%d=%s" % (i, v) for i, v in state.items()))
        if any(state[i] is None for i in range(1, 6)): raise Abort("J1~J5 중 회독 None이 있다. 쓰기 없음.")
        if abs(state[1] - 90) > OTHER_TOL or abs(state[5] - 90) > OTHER_TOL or abs(state[2] - 91) > OTHER_TOL or abs(state[3] - 91) > OTHER_TOL: raise Abort("J1·J2·J3·J5가 직립 근처(90~91)가 아니다. 쓰기 없음.")
        s4 = state[4]
        if not 60 <= s4 <= 100: raise Abort("J4 회독 %s가 복귀 허용 시작 범위(60~100) 밖이다. 쓰기 없음." % s4)
        if abs(s4 - a.target) <= TOL: raise Abort("J4가 이미 목표 %d 근처(%s)다. 복귀 필요 없음. 쓰기 없음." % (a.target, s4))
        d = a.target - s4
        today = datetime.date.today()
        ad = GripperTestAdapter(arm, {"approved": True, "expires": today.isoformat(), "allowed_times_ms": [TIME_MS], "first_move_min_ms": TIME_MS, "max_writes": 1, "delta_deg": abs(d), "joint_id": 4, "direction": 1 if d > 0 else -1, "angle_window": [min(s4, a.target) - 2, max(s4, a.target) + 2], "pattern": "pair"}, today=today, dry_run=(mode == "dry_run"))
        s = ad.read_start(); ad.set_targets([s, a.target, s, a.target, s]); result["start_j4"] = s
        say("J4 %d -> %d (%d°, 약 TCP 58 mm 이동, %d ms = %d ms/°). 쓰기 1회, 다른 서보는 읽기만." % (s, a.target, abs(d), TIME_MS, TIME_MS // abs(d)))
        say("\n== 안전 확인(1회) ==")
        for k, q in enumerate(("전원 차단 담당자가 현재 자리에 있습니까?", "낙하 받침이 팔 앞쪽에 있고, 팔 주변(약 70 mm)과 보드 위에 손·물체·케이블이 없습니까?", "J4 관절에서 이상한 소리·발열·떨림이 없습니까(만져서 뜨겁지 않습니까)?", "J4가 앞쪽으로 약 %d° 접힌 지금 자세가 맞습니까? (눈으로 보기에 이보다 훨씬 더 접혀 있으면, 예: 약 40°, N을 답해 쓰지 마세요)" % abs(d)), 1): say("  %d. %s" % (k, q))
        if not ask("위 4개를 모두 확인했습니까? (y면 소리 뒤 3초 뒤 J4가 직립 쪽으로 움직입니다. 쓰기 직후 J4가 반응하는지 읽어 확인합니다)"): raise Abort("사전 확인 미충족. 쓰기 없음.")
        say("\n[이동 1/1] J4->%d (time %d ms): %d초 뒤 이동합니다. 육안으로 보다가 이상하면 Ctrl-C." % (a.target, TIME_MS, COUNTDOWN_S)); time.sleep(COUNTDOWN_S)
        if mode == "dry_run": say("  dry_run: 쓰기 없음"); result["ended"] = "completed"; return
        ad.confirm_hands_clear(); ad.move(a.target, TIME_MS); wrote = True
        t0 = time.monotonic(); cons = 0; reacted = False; unresp = False; total = int(1.5 * TIME_MS) + 1000
        while (time.monotonic() - t0) * 1000.0 < total:
            t = round((time.monotonic() - t0) * 1000.0); v = ad.read_position(); result["series"].append((t, v)); cons = cons + 1 if v is None else 0
            if cons >= MAX_CONSEC_NONE: raise Abort("J4 회독 None이 연속 %d회. 추가 쓰기 없음." % cons)
            if v is not None and v != s4: reacted = True
            if not reacted and not unresp and t >= REACT_WAIT_MS:
                unresp = True; say("\n! 쓰기 후 %d ms 동안 J4 회독이 %d에서 변하지 않았다(회독 기준 미응답). 추가 쓰기는 없다. **명령은 이미 보냈으므로 약 10초 동안 팔에 다가가지 마세요**(서보가 늦게 움직일 수 있음). 계속 읽습니다..." % (REACT_WAIT_MS, s4))
            time.sleep(POLL_MS / 1000.0)
        if unresp:
            if not any(v is not None and v != s4 for _, v in result["series"]): result["j4_reading_changed"] = False
            else: result["j4_reading_changed"] = True
            result["visual_j4"] = inp("진단용 질문 1개(원인 구분에 필요): 지금까지 J4가 눈으로 보기에 움직였습니까? [움직임/안 움직임/모름]: ").strip()
            if not result["j4_reading_changed"]: raise Abort("쓰기 후 회독이 %d에서 변하지 않았다(미응답). 육안 답: %s. 추가 쓰기 없음." % (s4, result["visual_j4"] or "없음"))
            say("(3초 뒤 회독이 늦게 변했다: 지연 응답. 도착 확인으로 계속)")
        fin = last_valid(result["series"]); result["final_j4"] = fin; others = {i: read_valid(arm, i) for i in (1, 2, 3, 5)}; result["others_final"] = others
        say("-> J4 최종 회독 %s (목표 %d), J1=%s, J2=%s, J3=%s, J5=%s" % (fin, a.target, others[1], others[2], others[3], others[5]))
        if fin is None or abs(fin - a.target) > TOL: raise Abort("J4 도착 실패(마지막 유효 회독 %s, 목표 %d, 허용 %d°). 추가 쓰기 없음." % (fin, a.target, TOL))
        result["ended"] = "completed"
    except Abort as e: result["ended"] = "aborted"; result["abort_reason"] = str(e); say("\n중단:", e)
    except Hangup as e: result["ended"] = "hangup"; result["abort_reason"] = str(e); say("\n중단:", e)
    except KeyboardInterrupt: result["interrupted"] = True; result["ended"] = "interrupted"; say("\n중단(Ctrl-C): 이후 쓰기 없음")
    except EOFError: result["ended"] = "input_closed"; say("\n중단: 입력이 끊겼다. 이후 쓰기 없음")
    except GripperTestError as e: result["ended"] = "adapter_refused"; result["error"] = str(e); say("\n어댑터 거부:", e)
    except Exception as e: result["ended"] = "error"; result["error"] = repr(e); say("\n오류:", repr(e))
    finally:
        result["wrote"] = wrote; result["adapter_log"] = [list(l) for l in (ad.log if ad else [])]; result["finished_at"] = datetime.datetime.now().isoformat(timespec="seconds")
        Path(a.out).write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"); say("기록 저장:", a.out)


if __name__ == "__main__": main()
