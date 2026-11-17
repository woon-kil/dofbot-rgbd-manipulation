#!/usr/bin/env python3
"""G01.5b-4 S3: J3+J4를 함께 20° 계단(20→40→60→80°)으로 접고 같은 계단으로 되돌린다(J2·J1·J5는 읽기만). 사람이 입회해 직접 실행하는 대화형 스크립트.
사용: python3 g01_5b4_s3_test.py --fake [--fake-floor3 N --fake-floor4 N --fake-j2-drift N]   (가짜 서보)
      python3 g01_5b4_s3_test.py            (실제 읽기, 쓰기 0회 dry_run)
      python3 g01_5b4_s3_test.py --real     (실제 쓰기)
빠른 방식(사용자 개입 최소화): 시작 때 안전 확인 1회, 이동 직전 안내 줄(+소리) + 3초 대기(Ctrl-C로 취소), 끝에 종합 확인 1회. --gate80을 주면 80° 진입 앞에 y/N 1회가 추가된다.
자동 중단: 도착 실패(허용 3°), 회독 None 3연속, 명령 관절 정착 뒤 1초 변화 2° 이상, J1·J2·J5가 시작값에서 3° 이상 벗어남. 쓰기는 서보 id 3·4에만 개별로 보낸다(id 0·6축 동시 쓰기 없음).
"""
import argparse, builtins, datetime, json, signal, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gripper_adapter import GripperTestAdapter, GripperTestError
from g01_5b4_joint_test import FakeArm, Hangup, _on_signal, read_valid, last_valid

DELTA = 20; STAIRS = 4; WINDOW = [10, 105]                       # 목표 S-20 … S-80, 하한 10은 D6
TIMES = [8000, 1500, 1500, 3000, 3000, 1500, 1500, 1500]         # 접기 4번(첫 이동 8000, 80° 진입 3000), 펴기 4번(첫 3000)
RETURN_MS = 8000                                                  # 중단 후 복귀: 80°라도 100 ms/° (동결 상한 60 ms/° 이상)
POLL_MS = 100; COUNTDOWN_S = 3; MAX_CONSEC_NONE = 3; TOL = 3; DRIFT_LIMIT = 2; OTHER_TOL = 3
GATE_STEP = 3                                                     # 0부터. 4번째 이동(S-80 진입) 앞에서 y/N


def say(*a, **k): builtins.print(*a, **k, flush=True)
def inp(p): return builtins.input("\a" + p)
def ask(msg, default=False):
    ans = inp(msg + (" [Y/n] " if default else " [y/N] ")).strip().lower()
    return default if ans == "" else ans in ("y", "yes")


class Abort(Exception): pass


class FakeArm2(FakeArm):
    """가짜 팔 확장: 서보별 바닥값(도착 실패 시험), J2 드리프트(쓰기 N번째 뒤 J2가 이동)."""
    def __init__(self, pos, floors=None, j2_drift=None):
        super().__init__(pos); self.floors = dict(floors or {}); self.j2_drift = j2_drift; self.nw = 0
    def Arm_serial_servo_write(self, id, angle, time_ms):
        tgt = angle
        if id in self.floors and angle < self.floors[id]: tgt = self.floors[id]
        super().Arm_serial_servo_write(id, tgt, time_ms); self.writes[-1] = (id, angle, time_ms); self.nw += 1
        if self.j2_drift and self.nw == self.j2_drift[0]: self.v0[2] = self._p(2); self.v1[2] = self.pos[2] + self.j2_drift[1]; self.t0[2] = time.monotonic(); self.dur[2] = 800


def targets_for(s):
    return [s] + [s - DELTA * i for i in range(1, STAIRS + 1)] + [s - DELTA * i for i in range(STAIRS - 1, -1, -1)]


def poll(ads, arm, total_ms):
    """J3·J4(어댑터)와 J2(읽기)를 함께 회독한다. 반환: {3: [(ms, v)], 4: [...], 2: [...]}"""
    ser = {3: [], 4: [], 2: []}; cons = {3: 0, 4: 0, 2: 0}; t0 = time.monotonic()
    while (time.monotonic() - t0) * 1000.0 < total_ms:
        t = round((time.monotonic() - t0) * 1000.0)
        vals = {3: ads[3].read_position(), 4: ads[4].read_position(), 2: arm.Arm_serial_servo_read(2)}
        for j, v in vals.items():
            ser[j].append((t, v)); cons[j] = cons[j] + 1 if v is None else 0
            if cons[j] >= MAX_CONSEC_NONE: raise Abort("J%d 회독 None이 연속 %d회. 추가 이동 없음." % (j, cons[j]))
        time.sleep(POLL_MS / 1000.0)
    return ser


def drift_last_1s(series):
    w = [v for t, v in series if v is not None and t >= series[-1][0] - 1000]; return (max(w) - min(w)) if w else None


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--fake", action="store_true"); ap.add_argument("--real", action="store_true")
    ap.add_argument("--gate80", action="store_true", help="80° 단계 진입 앞에 y/N 1회(기본은 묻지 않음: 사용자 개입 최소화)"); ap.add_argument("--fake-s3", type=int, default=91); ap.add_argument("--fake-s4", type=int, default=91)
    ap.add_argument("--fake-floor3", type=int, default=None); ap.add_argument("--fake-floor4", type=int, default=None); ap.add_argument("--fake-j2-drift", type=int, default=None, help="쓰기 N번째 뒤 J2가 이동")
    ap.add_argument("--run-id", default="local"); ap.add_argument("--out", default="/tmp/g01_5b4_s3_result.json"); a = ap.parse_args()
    if a.fake and a.real: sys.exit("--fake와 --real은 함께 쓸 수 없다")
    mode = "fake" if a.fake else ("real" if a.real else "dry_run")
    result = {"run_id": a.run_id, "mode": mode, "stage": "G01.5b-4 S3 J3+J4 stair", "delta_deg": DELTA, "stairs": STAIRS, "window": WINDOW, "times_ms": TIMES, "return_ms": RETURN_MS, "started_at": datetime.datetime.now().isoformat(timespec="seconds"), "state0": None, "steps": [], "interrupted": False, "ended": None}
    ads = {}; arm = None; writes_done = False
    for sg in (signal.SIGHUP, signal.SIGTERM): signal.signal(sg, _on_signal)
    try:
        say("G01.5b-4 S3 J3+J4 동시 접기 시험(Δ%d° × %d단계, 모드 %s): 서보 3·4만 명령한다(각각 개별 쓰기). J1·J2·J5·그리퍼는 명령하지 않는다. 물체 없음." % (DELTA, STAIRS, mode))
        if a.fake: arm = FakeArm2({1: 90, 2: 91, 3: a.fake_s3, 4: a.fake_s4, 5: 90, 6: 99}, {j: v for j, v in ((3, a.fake_floor3), (4, a.fake_floor4)) if v is not None}, (a.fake_j2_drift, 8) if a.fake_j2_drift else None)
        else:
            from Arm_Lib import Arm_Device; arm = Arm_Device()
        state = {i: read_valid(arm, i) for i in range(1, 7)}; result["state0"] = state
        say("b-4-0 현재 회독(SDK 각도): " + ", ".join("J%d=%s" % (i, v) for i, v in state.items()))
        if any(state[i] is None for i in range(1, 6)): raise Abort("J1~J5 중 회독 None이 있다(J2~J4 None은 R21 사고 이력). 쓰기 없음.")
        for j in (1, 5):
            if abs(state[j] - 90) > OTHER_TOL: raise Abort("J%d 회독 %s가 90에서 %d° 넘게 벗어나 있다. 쓰기 없음." % (j, state[j], OTHER_TOL))
        if abs(state[2] - 91) > OTHER_TOL: raise Abort("J2 회독 %s가 S1·S2 종료 자세(91)에서 %d° 넘게 벗어나 있다. 쓰기 없음." % (state[2], OTHER_TOL))
        for j in (3, 4):
            if not WINDOW[0] + DELTA * STAIRS - 1 <= state[j] <= WINDOW[1]: raise Abort("J%d 회독 %s가 시작 허용값 밖이다(S-80이 하한 %d 아래). 쓰기 없음." % (j, state[j], WINDOW[0]))
        today = datetime.date.today(); T = {}
        for j in (3, 4):
            ads[j] = GripperTestAdapter(arm, {"approved": True, "expires": today.isoformat(), "allowed_times_ms": sorted(set(TIMES + [RETURN_MS])), "first_move_min_ms": 8000, "max_writes": 10, "delta_deg": DELTA, "joint_id": j, "direction": -1, "angle_window": WINDOW, "pattern": "stair", "stair_steps": STAIRS}, today=today, dry_run=(mode == "dry_run"))
            s = ads[j].read_start(); T[j] = targets_for(s); ads[j].set_targets(T[j])
        result["targets"] = {"J3": T[3], "J4": T[4]}; s2 = state[2]
        say("시작 J3=%d, J4=%d(J2=%d, J1=%d, J5=%d). 목표(접기→펴기): J3 %s, J4 %s" % (T[3][0], T[4][0], s2, state[1], state[5], T[3][1:], T[4][1:]))
        say("\n== 안전 확인(시작 시 1회) ==")
        for k, q in enumerate(("전원 차단 담당자가 현재 자리에 있습니까?", "낙하 받침을 준비했고, 접힌 팔(앞쪽, 보드 위)의 낙하를 받치되 팔 경로(TCP y 약 -15~220 mm, 높이 약 70~460 mm) 안에 들어오지 않습니까?", "팔이 앞쪽으로 접히는 동안(TCP 약 440 mm 높이에서 보드 위를 지나 약 70 mm 높이까지) 닿을 물체·손·사람·케이블이 팔 주변과 작업대·보드 위에 없습니까?", "팔이 직립(대기) 자세입니까?", "DaBai·그리퍼 케이블이 J3·J4 약 80° 접힘에 당겨지지 않고 여유가 있어 보입니까?", "위 회독값(J1·J5 약 90, J2 약 91)과 목표 목록이 맞다."), 1): say("  %d. %s" % (k, q))
        if not ask("위 6개를 모두 확인했습니까? (y면 이동 시작, 이후 이동마다 소리 뒤 3초 대기, 마지막에 종합 확인 1회)"): raise Abort("사전 확인 미충족. 쓰기 없음.")
        for i in range(2 * STAIRS):
            tg = {j: T[j][i + 1] for j in (3, 4)}; ms = TIMES[i]
            if i == GATE_STEP and a.gate80 and mode != "dry_run":
                say("\n[80° 진입 확인] 다음은 가장 깊은 단계입니다(모델: TCP 높이 약 69 mm, 보드 앞 가장자리까지 약 53 mm).")
                if not ask("손가락 끝·DaBai와 작업대·보드 간격, 케이블 장력, J2 처짐이 괜찮습니까? 진행합니까?"): raise Abort("80° 진입 거부(사용자 N). 추가 이동 없음.")
            say("\n[이동 %d/%d] J3->%d, J4->%d (time %d ms): %d초 뒤 이동합니다. 육안으로 보다가 이상하면 Ctrl-C." % (i + 1, 2 * STAIRS, tg[3], tg[4], ms, COUNTDOWN_S)); time.sleep(COUNTDOWN_S)
            step = {"index": i + 1, "targets": tg, "time_ms": ms}
            if mode == "dry_run": step["point"] = "dry_run_move"; result["steps"].append(step); say("  dry_run: 이동 없음(쓰기 0회, 도착 검사 생략)"); continue
            ads[3].confirm_hands_clear(); ads[3].move(tg[3], ms); writes_done = True
            ads[4].confirm_hands_clear(); ads[4].move(tg[4], ms)
            ser = poll(ads, arm, int(1.5 * ms) + 1000); step["point"] = "move"; step["series"] = {str(j): v for j, v in ser.items()}
            finals = {j: last_valid(ser[j]) for j in (3, 4, 2)}; step["final"] = {str(j): v for j, v in finals.items()}
            step["drift_last_1s"] = {str(j): drift_last_1s(ser[j]) for j in (3, 4)}
            step["other"] = {"1": read_valid(arm, 1), "5": read_valid(arm, 5)}; result["steps"].append(step)
            say("  -> 최종 회독 J3=%s(목표 %d), J4=%s(목표 %d), J2=%s, J1=%s, J5=%s" % (finals[3], tg[3], finals[4], tg[4], finals[2], step["other"]["1"], step["other"]["5"]))
            for j in (3, 4):
                if finals[j] is None or abs(finals[j] - tg[j]) > TOL: step["arrival_failed"] = j; raise Abort("J%d 도착 실패(마지막 유효 회독 %s, 목표 %d, 허용 %d°). 서보는 마지막 목표를 유지 중일 수 있다. 추가 이동 없음." % (j, finals[j], tg[j], TOL))
                if step["drift_last_1s"][str(j)] is not None and step["drift_last_1s"][str(j)] >= DRIFT_LIMIT: raise Abort("J%d 정착 뒤 1초 회독 변화 %d° (한계 %d°, 헌팅·처짐 의심). 추가 이동 없음." % (j, step["drift_last_1s"][str(j)], DRIFT_LIMIT))
            if finals[2] is None or abs(finals[2] - s2) > OTHER_TOL: raise Abort("J2(명령하지 않은 관절) 회독 %s가 시작값 %d에서 %d° 넘게 벗어났다(하중 변화 처짐 의심). 추가 이동 없음." % (finals[2], s2, OTHER_TOL))
            for j in ("1", "5"):
                if step["other"][j] is None or abs(step["other"][j] - 90) > OTHER_TOL: raise Abort("J%s 회독 %s가 90에서 %d° 넘게 벗어났다. 추가 이동 없음." % (j, step["other"][j], OTHER_TOL))
        if mode != "dry_run":
            result["overall_ok"] = ask("마지막 확인 1회: 전체 이동에서 소리·떨림·냄새, 처짐·흔들림, 접촉·걸림, 케이블 당김이 모두 없었습니까?")
            if not result["overall_ok"]: raise Abort("종합 확인에서 이상 보고(또는 명시적 y 없음). 추가 이동 없음.")
        result["ended"] = "completed"
    except Abort as e: result["ended"] = "aborted"; result["abort_reason"] = str(e); say("\n중단:", e)
    except Hangup as e: result["ended"] = "hangup"; result["abort_reason"] = str(e); say("\n중단:", e)
    except KeyboardInterrupt: result["interrupted"] = True; result["ended"] = "interrupted"; say("\n중단(Ctrl-C): 이후 명령 없음(이미 보낸 이동은 서보가 끝까지 간다)")
    except EOFError: result["ended"] = "input_closed"; say("\n중단: 입력이 끊겼다. 이후 명령 없음")
    except GripperTestError as e: result["ended"] = "adapter_refused"; result["error"] = str(e); say("\n어댑터 거부:", e)
    except Exception as e: result["ended"] = "error"; result["error"] = repr(e); say("\n오류:", repr(e))
    finally:
        if ads and writes_done and result["ended"] != "completed":
            try:
                if ask("J3·J4가 시작값이 아닐 수 있습니다. 두 관절을 시작값으로 복귀(각 1회, %d ms)할까요? (손·도구를 뗀 뒤에만 y)" % RETURN_MS):
                    done = []
                    for j in (3, 4):
                        try: ads[j].confirm_hands_clear(); ads[j].return_to_start(RETURN_MS); done.append(j)
                        except GripperTestError as e: say("J%d 복귀 거부: %s" % (j, e)); result.setdefault("return_refused", {})[str(j)] = str(e)
                    result["returned_to_start"] = done
                    if done:
                        say("복귀 명령 전송(J%s), 복귀 후 회독 확인 중..." % ",".join(map(str, done))); ser = poll(ads, arm, int(1.5 * RETURN_MS) + 1000)
                        result["return_readback"] = {str(j): last_valid(ser[j]) for j in (3, 4)}; say("복귀 후 마지막 유효 회독:", result["return_readback"], "(시작 J3=%d, J4=%d)" % (T[3][0], T[4][0]))
                else: result["returned_to_start"] = False
            except BaseException as e: result["return_error"] = repr(e); say("복귀 처리 중단:", repr(e))
        result["adapter_log"] = {str(j): [(["write", *l[1:3], "fake" if (a.fake and l[0] == "write") else l[3], *l[4:]] if l[0] == "write" else list(l)) for l in ads[j].log] for j in ads}
        result["finished_at"] = datetime.datetime.now().isoformat(timespec="seconds")
        Path(a.out).write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"); say("기록 저장:", a.out)


if __name__ == "__main__": main()
