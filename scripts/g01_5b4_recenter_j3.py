#!/usr/bin/env python3
"""G01.5b-4 S3 이슈 후속: J3만 직립(시작값)으로 단독 복귀 쓰기 1회(8000 ms). 쓰기 직후 J3가 움직이기 시작하는지 읽어 확인하고, 반응이 없으면 추가 쓰기 없이 멈춘다.
사람이 입회해 직접 실행(무인 실행 금지). --fake로 가짜 서보 시험. 쓰기는 서보 id 4에 1회뿐(J1~J3·J5·그리퍼 읽기만).
사용: python3 g01_5b4_recenter_j3.py [--real] [--target 91]   (기본 dry_run: 읽기만)
"""
import argparse, builtins, datetime, json, signal, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gripper_adapter import GripperTestAdapter, GripperTestError
from g01_5b4_joint_test import FakeArm, Hangup, _on_signal, read_valid, last_valid

TIME_MS = 8000; COUNTDOWN_S = 3; POLL_MS = 100; REACT_WAIT_MS = 3000; TOL = 3; OTHER_TOL = 3; MAX_CONSEC_NONE = 3
JN = {"j": 3}   # 이동 대상 관절(1 또는 3). 안내 문구의 "J3"는 이 값으로 바꿔 출력한다.
def _j(x): return str(x).replace("서보 3", "서보 %d" % JN["j"]).replace("J3", "J%d" % JN["j"])
def say(*a, **k): builtins.print(*[_j(x) for x in a], **k, flush=True)
def inp(p): return builtins.input("\a" + _j(p))
def ask(msg): return inp(msg + " [y/N] ").strip().lower() in ("y", "yes")
class Abort(Exception): pass


class FakeArm3(FakeArm):
    def __init__(self, pos, ignore_ids=()): super().__init__(pos); self.ignore = set(ignore_ids)
    def Arm_serial_servo_write(self, id, angle, time_ms):
        if id in self.ignore: self.writes.append((id, angle, time_ms)); return   # 쓰기를 받고도 무시(미응답 시험)
        super().Arm_serial_servo_write(id, angle, time_ms)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--fake", action="store_true"); ap.add_argument("--real", action="store_true"); ap.add_argument("--target", type=int, default=None); ap.add_argument("--joint", type=int, choices=[1, 2, 3], default=3)
    ap.add_argument("--fake-s0", type=int, default=95); ap.add_argument("--fake-ignore4", action="store_true"); ap.add_argument("--run-id", default="local"); ap.add_argument("--out", default="/tmp/g01_5b4_recover_result.json"); a = ap.parse_args()
    if a.fake and a.real: sys.exit("--fake와 --real은 함께 쓸 수 없다")
    mode = "fake" if a.fake else ("real" if a.real else "dry_run")
    J = a.joint; JN["j"] = J
    if a.target is None: a.target = 90 if J == 1 else 91   # J2·J3 기본 91(직립)
    result = {"run_id": a.run_id, "mode": mode, "stage": "G01.5b-4 재중심 J%d" % J, "target": a.target, "time_ms": TIME_MS, "started_at": datetime.datetime.now().isoformat(timespec="seconds"), "state0": None, "series": [], "ended": None}
    ad = None; wrote = False
    for sg in (signal.SIGHUP, signal.SIGTERM): signal.signal(sg, _on_signal)
    try:
        say("G01.5b-4 S3 후속: J3 재중심(서보 3에 쓰기 1회, %d ms, 모드 %s). 다른 서보는 읽기만." % (TIME_MS, mode))
        arm = FakeArm3({1: 90, 2: 91, 3: 91, 4: 91, 5: 90, 6: 99, **{J: a.fake_s0}}, (J,) if a.fake_ignore4 else ()) if a.fake else __import__("Arm_Lib").Arm_Device()
        state = {i: read_valid(arm, i) for i in range(1, 7)}; result["state0"] = state
        builtins.print("현재 회독(SDK 각도): " + ", ".join("J%d=%s" % (i, v) for i, v in state.items()), flush=True)   # 치환 없이 출력
        if any(state[i] is None for i in range(1, 6)): raise Abort("J1~J5 중 회독 None이 있다. 쓰기 없음.")
        nom = {1: 90, 2: 91, 4: 91, 5: 90}
        if J == 1: nom[3] = 94   # J3는 재중심 93 부근(91~97 허용: 준비 단계에서 J3를 93으로 맞춘 뒤 약간 변한다)
        if J == 2: nom = {1: 90, 3: 94, 4: 91, 5: 90}   # J2 재중심: 직립 근처인 J1·J4·J5와, 93 부근 J3(91~97)를 요구(이전 취득 뒤 J2가 매번 +1° 밀리는 현상 보정)
        if any(abs(state[k] - v) > OTHER_TOL for k, v in nom.items() if k != J): raise Abort("이동하지 않는 관절이 직립 근처(J1·J5 90, J2·J4 91%s)가 아니다. 쓰기 없음." % (", J3 91" if J == 1 else ""))
        s0 = state[J]
        if not 60 <= s0 <= 100: raise Abort("J3 회독 %s가 복귀 허용 시작 범위(60~100) 밖이다. 쓰기 없음." % s0)
        if abs(s0 - a.target) <= 1: raise Abort("J3가 이미 목표 %d 근처(%s)다. 복귀 필요 없음. 쓰기 없음." % (a.target, s0))
        d = a.target - s0
        today = datetime.date.today()
        ad = GripperTestAdapter(arm, {"approved": True, "expires": today.isoformat(), "allowed_times_ms": [TIME_MS], "first_move_min_ms": TIME_MS, "max_writes": 1, "delta_deg": abs(d), "joint_id": J, "direction": 1 if d > 0 else -1, "angle_window": [min(s0, a.target) - 2, max(s0, a.target) + 2], "pattern": "pair"}, today=today, dry_run=(mode == "dry_run"))
        s = ad.read_start(); ad.set_targets([s, a.target, s, a.target, s]); result["start_j3"] = s
        say("J3 %d -> %d (%d°, %d ms = %d ms/°). 쓰기 1회, 다른 서보는 읽기만." % (s, a.target, abs(d), TIME_MS, TIME_MS // abs(d)))
        say("\n== 안전 확인(1회) ==")
        for k, q in enumerate(("전원 차단 담당자가 현재 자리에 있습니까?", "낙하 받침이 팔 앞쪽에 있고, 팔 주변(약 40 mm)과 보드 위에 손·물체·케이블이 없습니까?", "J3 관절에서 이상한 소리·발열·떨림이 없습니까(만져서 뜨겁지 않습니까)?", "J3의 현재 회독 %d에서 목표 %d로 %+d° 움직입니다(SDK %s 방향). 눈으로 보기에 지금 자세가 이 숫자와 크게 다르면 N을 답해 쓰지 마세요." % (s0, a.target, d, "증가" if d > 0 else "감소"), "J4(손목)가 아래팔과 일직선입니까?"), 1): say("  %d. %s" % (k, q))
        if not ask("위 5개를 모두 확인했습니까? (y면 소리 뒤 3초 뒤 J3가 목표(%d) 쪽으로 움직입니다. 쓰기 직후 J3가 반응하는지 읽어 확인합니다)" % a.target): raise Abort("사전 확인 미충족. 쓰기 없음.")
        say("\n[이동 1/1] J3->%d (time %d ms): %d초 뒤 이동합니다. 육안으로 보다가 이상하면 Ctrl-C." % (a.target, TIME_MS, COUNTDOWN_S)); time.sleep(COUNTDOWN_S)
        if mode == "dry_run": say("  dry_run: 쓰기 없음"); result["ended"] = "completed"; return
        ad.confirm_hands_clear(); ad.move(a.target, TIME_MS); wrote = True
        t0 = time.monotonic(); cons = 0; reacted = False; unresp = False; total = int(1.5 * TIME_MS) + 1000
        while (time.monotonic() - t0) * 1000.0 < total:
            t = round((time.monotonic() - t0) * 1000.0); v = ad.read_position(); result["series"].append((t, v)); cons = cons + 1 if v is None else 0
            if cons >= MAX_CONSEC_NONE: raise Abort("J3 회독 None이 연속 %d회. 추가 쓰기 없음." % cons)
            if v is not None and v != s0: reacted = True
            if not reacted and not unresp and t >= REACT_WAIT_MS:
                unresp = True; say("\n! 쓰기 후 %d ms 동안 J3 회독이 %d에서 변하지 않았다(회독 기준 미응답). 추가 쓰기는 없다. **명령은 이미 보냈으므로 약 10초 동안 팔에 다가가지 마세요**(서보가 늦게 움직일 수 있음). 계속 읽습니다..." % (REACT_WAIT_MS, s0))
            time.sleep(POLL_MS / 1000.0)
        if unresp:
            if not any(v is not None and v != s0 for _, v in result["series"]): result["j3_reading_changed"] = False
            else: result["j3_reading_changed"] = True
            result["visual_j3"] = inp("진단용 질문 1개(원인 구분에 필요): 지금까지 J3가 눈으로 보기에 움직였습니까? [움직임/안 움직임/모름]: ").strip()
            if not result["j3_reading_changed"]: raise Abort("쓰기 후 회독이 %d에서 변하지 않았다(미응답). 육안 답: %s. 추가 쓰기 없음." % (s0, result["visual_j3"] or "없음"))
            say("(3초 뒤 회독이 늦게 변했다: 지연 응답. 도착 확인으로 계속)")
        fin = last_valid(result["series"]); result["final_j3"] = fin; others = {i: read_valid(arm, i) for i in (1, 2, 3, 4, 5) if i != J}; result["others_final"] = others
        say("-> J3 최종 회독 %s (목표 %d), 나머지 %s" % (fin, a.target, others))
        if fin is None or abs(fin - a.target) > TOL: raise Abort("J3 도착 실패(마지막 유효 회독 %s, 목표 %d, 허용 %d°). 추가 쓰기 없음." % (fin, a.target, TOL))
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
