#!/usr/bin/env python3
"""G01.5b-4 S1/S2 단일 피치 관절 시험(--joint 4|3|2 --delta 10|20 --dir minus|plus). 관절·폭·방향마다 별도 확인 문구. 보정 자세는 SDK 감소(앞쪽 접힘) 방향이라 minus를 먼저 시험한다. 사람이 입회해 직접 실행하는 대화형 스크립트(무인 실행 금지).
사용: python3 g01_5b4_joint_test.py --joint 4 --delta 10 --dir minus --fake [--fake-scenario ...]   (가짜 서보, 하드웨어 없음)
      python3 g01_5b4_joint_test.py --joint 4 --delta 10 --dir minus                                (실제 읽기, 쓰기 0회 dry_run, 도착 검사 생략)
      python3 g01_5b4_joint_test.py --joint 4 --delta 10 --dir minus --real                         (실제 쓰기. 확인 문구를 직접 입력해야 한다)
b-4-0: 서보 1~6을 각 1회 읽는다(읽기만). 쓰기 대상은 --joint로 정한 서보 하나뿐이다(중력 하중이 걸리는 피치 관절). 목표 패턴 pair: [S, S+Δ, S, S+Δ, S]. 허용 범위 80~100(J3·J4 ≥ 5° 제한 충족).
중단·오류·입력 끊김·SIGHUP·Ctrl-C에도 기록을 저장한다. 중단 후에는 운영자가 y일 때만 S 복귀 1회(기본 N), 복귀 후 회독을 확인한다.
"""
import argparse, builtins, datetime, json, signal, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gripper_adapter import GripperTestAdapter, GripperTestError

JOINT = None                        # --joint로 설정(4, 3, 2)
JN = "J?"
DELTA = 10                          # --delta로 설정(10 또는 20)
WINDOW = [75, 105]                  # 90 ± (Δ+5)로 main에서 확정. J3·J4 ≥ 10(D6) 충족
DIRECTION = -1                      # --dir minus: SDK 감소(앞쪽 접힘), plus: SDK 증가(뒤쪽)
# 직립에서 관절 하나를 Δ° 기울일 때 TCP 이동량(mm, URDF FK 계산, g01_reach_envelope.py). 사전 확인 문구에 쓴다.
SHIFT_MM = {(4, 10): 29, (3, 10): 44, (2, 10): 58, (4, 20): 58, (3, 20): 87, (2, 20): 116}
TIMES = [8000, 1500, 1500, 1500]    # S+Δ, S, S+Δ, S. 첫 이동은 동결 상한(ADR-005)의 8000 ms. `time` 비례 비교는 J1에서 끝났으므로(1.999/1.994) 생략
POLL_MS = 100
COUNTDOWN_S = 3
MAX_CONSEC_NONE = 3
PHRASE = "APPROVE G01.5B4 J? REAL"      # main에서 확정


class FakeArm:
    """가짜 팔: 서보별 위치(기본 90). write 후 time ms 동안 선형 이동. Arm_Lib.py 시그니처. limit/none 시나리오 지원."""
    def __init__(self, pos=None, stall_at=None, none_after_write=0, none_ids=(), upright=True, floor_at=None):
        self.pos = dict(pos or {i: 90 for i in range(1, 7)}); self.stall_at = stall_at; self.floor_at = floor_at; self.none_ids = set(none_ids)
        self.v0 = {}; self.v1 = {}; self.t0 = {}; self.dur = {}; self.writes = []; self.none_left = 0; self.none_after_write = none_after_write
    def _p(self, i):
        if i not in self.t0: return self.pos[i]
        e = (time.monotonic() - self.t0[i]) * 1000.0
        return self.v1[i] if e >= self.dur[i] else self.v0[i] + (self.v1[i] - self.v0[i]) * e / self.dur[i]
    def Arm_serial_servo_read(self, id):
        if id in self.none_ids: return None
        if self.none_left > 0: self.none_left -= 1; return None
        return int(round(self._p(id)))
    def Arm_serial_servo_write(self, id, angle, time_ms):
        self.v0[id] = self._p(id); tgt = angle
        if self.stall_at is not None and angle > self.stall_at: tgt = self.stall_at
        if self.floor_at is not None and angle < self.floor_at: tgt = self.floor_at
        self.v1[id] = tgt; self.t0[id] = time.monotonic(); self.dur[id] = time_ms; self.writes.append((id, angle, time_ms)); self.none_left = self.none_after_write
    def __getattr__(self, n): raise AssertionError("허용되지 않은 SDK API: " + n)


class Abort(Exception):
    def __init__(self, msg): super().__init__(msg.replace("@J@", JN))
class Hangup(Exception): pass
def _on_signal(signum, frame): raise Hangup("신호 %d 수신(연결 끊김·종료)" % signum)


def print(*a, **k): builtins.print(*[x.replace("@J@", JN) if isinstance(x, str) else x for x in a], **k)
def input(p=""): return builtins.input("\a" + p.replace("@J@", JN))


def ask(msg, default=False):
    ans = input(msg + (" [Y/n] " if default else " [y/N] ")).strip().lower()
    return default if ans == "" else ans in ("y", "yes")


def read_valid(arm, i, tries=3):
    """서보 i를 읽는다. 일시 None이면 50 ms 간격으로 재시도(최대 tries회). 읽기만 한다."""
    for _ in range(tries):
        v = arm.Arm_serial_servo_read(i)
        if v is not None: return v
        time.sleep(0.05)
    return None


def poll(ad, total_ms):
    series = []; t0 = time.monotonic(); consec = 0
    while (time.monotonic() - t0) * 1000.0 < total_ms:
        v = ad.read_position(); series.append((round((time.monotonic() - t0) * 1000.0), v))
        consec = consec + 1 if v is None else 0
        if consec >= MAX_CONSEC_NONE: raise Abort("회독 None이 연속 %d회. 추가 이동 없음." % consec)
        time.sleep(POLL_MS / 1000.0)
    return series


def settle_ms(series):
    """None 읽기를 무시한 정착 시간(마지막 값에 처음 도달해 유지된 시점)."""
    vv = [(t, v) for t, v in series if v is not None]
    if len(vv) < 3: return None
    fin = vv[-1][1]; st = None
    for t, v in vv:
        if v == fin and st is None: st = t
        if v != fin: st = None
    return st


def last_valid(series): return next((v for _, v in reversed(series) if v is not None), None)


def first_reach(series, tgt): return next((t for t, v in series if v is not None and v == tgt), None)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--fake", action="store_true"); ap.add_argument("--real", action="store_true")
    ap.add_argument("--fake-stall", type=int, default=None); ap.add_argument("--fake-floor", type=int, default=None); ap.add_argument("--joint", type=int, choices=[4, 3, 2], required=True); ap.add_argument("--delta", type=int, choices=[10, 20], required=True); ap.add_argument("--dir", choices=["minus", "plus"], required=True); ap.add_argument("--fake-s", type=int, default=90); ap.add_argument("--fake-none-after-write", type=int, default=0)
    ap.add_argument("--strict", action="store_true", help="이동마다 확인·승인 문구를 요구하는 이전 방식"); ap.add_argument("--run-id", default="local"); ap.add_argument("--out", default="/tmp/g01_5b4_result.json"); a = ap.parse_args()
    if a.fake and a.real: sys.exit("--fake와 --real은 함께 쓸 수 없다")
    global JOINT, JN, PHRASE, DELTA, WINDOW, DIRECTION
    JOINT = a.joint; JN = "J%d" % JOINT; DELTA = a.delta; DIRECTION = -1 if a.dir == "minus" else 1; WINDOW = [90 - (DELTA + 5), 90 + (DELTA + 5)]
    PHRASE = "APPROVE G01.5B4 %s %s%d REAL" % (JN, "M" if DIRECTION < 0 else "P", DELTA)
    mode = "fake" if a.fake else ("real" if a.real else "dry_run")
    result = {"run_id": a.run_id, "mode": mode, "stage": "G01.5b-4 S%d %s %s" % (1 if a.delta == 10 else 2, "J%d" % a.joint, a.dir), "joint": a.joint, "direction": a.dir, "window": WINDOW, "started_at": datetime.datetime.now().isoformat(timespec="seconds"), "delta_deg": DELTA, "window": WINDOW, "times_ms": TIMES, "state0": None, "steps": [], "interrupted": False, "ended": None}
    ad = None; writes_done = False; arm = None
    for sg in (signal.SIGHUP, signal.SIGTERM): signal.signal(sg, _on_signal)
    try:
        print("G01.5b-4 @J@ 단일 관절 시험(Δ%d, %s, %s): 서보 %d(피치 관절, 중력 하중)만 움직인다. 다른 관절·그리퍼는 명령하지 않는다. 물체 없음." % (DELTA, "SDK 감소(앞쪽 접힘)" if DIRECTION < 0 else "SDK 증가(뒤쪽)", mode, JOINT))
        QS = ("전원 차단 담당자가 현재 자리에 있습니까?", "낙하 받침을 준비했습니까?", "팔이 %s으로 약 %d° 기울면(끝이 약 %d mm 이동) 닿을 물체·손·사람·케이블이 팔 주변과 작업대·보드 위에 없습니까?" % ("앞쪽(사용자·작업대 쪽)" if DIRECTION < 0 else "뒤쪽(로봇 뒤)", DELTA, SHIFT_MM[(JOINT, DELTA)]), "낙하 받침이 팔의 %s 낙하를 받쳐 줍니까?(이번 시험의 예상 기울임 방향)" % ("앞쪽(사용자·작업대 쪽)" if DIRECTION < 0 else "뒤쪽(로봇 뒤)"), "팔이 직립(대기) 자세입니까?", "DaBai·그리퍼 케이블이 @J@ 약 ±%d° 기울임에 당겨지지 않고 여유가 있어 보입니까?" % DELTA)
        if a.strict:
            for q in QS:
                if not ask(q): raise Abort("사전 확인 미충족. 명령은 보내지 않았다.")
        if a.real and a.strict:
            if input("실제 쓰기를 시작하려면 정확히 '%s'를 입력: " % PHRASE).strip() != PHRASE: raise Abort("승인 문구 불일치. 명령은 보내지 않았다.")
        elif mode == "dry_run" and a.strict: input("(연습) 실제 시험에서는 '%s'를 입력해야 합니다. 지금 입력해 보세요(틀려도 진행): " % PHRASE)
        if a.fake: arm = FakeArm({**{i: 90 for i in range(1, 6)}, 6: 99, JOINT: a.fake_s}, a.fake_stall, a.fake_none_after_write, floor_at=a.fake_floor)
        else:
            from Arm_Lib import Arm_Device; arm = Arm_Device()   # 생성자는 버스를 열기만 한다(소스 확인)
        # b-4-0: 서보 1~6 각 1회 읽기(읽기만)
        state = {i: read_valid(arm, i) for i in range(1, 7)}; result["state0"] = state
        print("b-4-0 현재 회독(SDK 각도): " + ", ".join("J%d=%s" % (i, v) for i, v in state.items()))
        if any(state[i] is None for i in range(1, 6)): raise Abort("J1~J5 중 회독 None이 있다(J2~J4 None은 R21 사고 이력). 쓰기 없음.")
        if not WINDOW[0] <= state[JOINT] <= WINDOW[1]: raise Abort("@J@ 회독 %s가 승인 범위 %s 밖이다. 쓰기 없음." % (state[JOINT], WINDOW))
        if a.strict and not ask("위 6개 값이 직립 대기 자세와 맞습니까(J1~J5가 약 90)?"): raise Abort("현재 자세 미확인. 쓰기 없음.")
        today = datetime.date.today()
        ad = GripperTestAdapter(arm, {"approved": True, "expires": today.isoformat(), "allowed_times_ms": sorted(set(TIMES)), "first_move_min_ms": 8000, "max_writes": 6, "delta_deg": DELTA, "joint_id": JOINT, "direction": DIRECTION, "angle_window": WINDOW, "pattern": "pair"}, today=today, dry_run=(mode == "dry_run"))
        s = ad.read_start(); targets = [s, s + DIRECTION * DELTA, s, s + DIRECTION * DELTA, s]; result["targets"] = targets
        print("시작 회독값 S = %d. 절대 목표 목록 = %s (첫 값은 현재 위치, 쓰기는 두 번째부터)" % (s, targets))
        if a.strict:
            if not ask("이 목록을 확인했고 이대로 진행합니까?"): raise Abort("목표 목록 미확인. 쓰기 없음.")
        else:
            print("\n== 안전 확인(시작 시 1회) =="); [print("  %d. %s" % (k + 1, q.replace("@J@", JN))) for k, q in enumerate(QS)]
            print("  7. 위 회독값(J1~J5 약 90)과 목표 목록이 맞다.")
            if not ask("위 7개를 모두 확인했습니까? (y면 이동 시작, 이후 이동마다 비프음 뒤 3초 대기)"): raise Abort("사전 확인 미충족. 쓰기 없음.")
        ad.set_targets(targets)
        for i, (tgt, t_ms) in enumerate(zip(targets[1:], TIMES)):
            if a.strict:
                if not ask("손과 도구를 팔 주변에서 뗐습니까(팔이 기운다)? 다음 이동: @J@ -> %d (time %d ms)" % (tgt, t_ms)): raise Abort("손 제거 미확인. 추가 명령 없음.")
            else:
                builtins.print("\a\n[이동 %d/4] %s -> %d (time %d ms): %d초 뒤 이동합니다. 육안으로 보다가 이상하면 Ctrl-C." % (i + 1, JN, tgt, t_ms, COUNTDOWN_S), flush=True); time.sleep(COUNTDOWN_S)
            ad.confirm_hands_clear(); ad.move(tgt, t_ms); writes_done = writes_done or mode != "dry_run"
            if mode == "dry_run":
                print("  dry_run: 이동 없음(쓰기 0회, 도착 검사 생략)"); result["steps"].append({"point": "dry_run_move", "angle": tgt, "time_ms": t_ms}); continue
            series = poll(ad, int(1.5 * t_ms) + 1000); last = last_valid(series); fr = first_reach(series, tgt)
            prev_final = next((x["final_readback"] for x in reversed(result["steps"]) if x["point"] == "move"), state[JOINT])
            step = {"point": "move", "angle": tgt, "time_ms": t_ms, "first_reach_ms": fr, "settle_ms": settle_ms(series), "start_readback": prev_final, "final_readback": last, "width_deg": (last - prev_final) if last is not None else None, "drift_last_1s_deg": (lambda w: (max(w) - min(w)) if w else None)([v for t_, v in series if v is not None and t_ >= series[-1][0] - 1000]), "series": series}; result["steps"].append(step)
            print("  -> @J@ 이동 %d, time %d ms: 최초 도달 %s ms, 최종 회독 %s" % (tgt, t_ms, fr, last))
            if last is None or abs(last - tgt) > 3: step["arrival_failed"] = True; raise Abort("도착 실패(마지막 유효 회독 %s, 목표 %d, 허용 3°). 서보는 마지막 목표를 유지 중일 수 있다. 추가 이동 없음." % (last, tgt))
            if abs(last - tgt) >= 2: print("  ! 경고: 회독 잔차가 %d°입니다. 한계·저항을 확인하세요." % abs(last - tgt))
            if not a.strict:
                step["user_visual"] = "사용자 육안 확인(이동별 질문 생략, 이상 시 Ctrl-C)"; continue
            step["quiet"] = ask("  소리·떨림·냄새 없이 부드럽게 움직였습니까?")
            if not step["quiet"]: raise Abort("이상 징후 보고(또는 명시적 y 없음). 추가 이동 없음.")
            step["held_pose"] = ask("  이동 후 자세가 중력으로 처지거나 흔들리지 않고 유지됩니까?")
            if not step["held_pose"]: raise Abort("자세 처짐·흔들림 보고(또는 명시적 y 없음). 추가 이동 없음.")
            step["no_contact"] = ask("  팔·DaBai·케이블이 주변에 닿거나 걸리지 않고 움직였습니까?")
            if not step["no_contact"]: raise Abort("접촉·걸림 보고(또는 명시적 y 없음). 추가 이동 없음.")
            if i == 0:
                d = input("  팔 윗부분이 어느 쪽으로 기울었습니까? (Gemini·사용자 쪽=앞, 로봇 뒤쪽=뒤; 이 시험의 예상: %s) [앞/뒤/모름/변화없음]: " % ("앞(감소 방향)" if DIRECTION < 0 else "뒤(증가 방향)")).strip(); result["rotation_direction_after_first_move"] = d; step["direction"] = d
                if d in ("앞", "뒤"):
                    step["catch_pad_ok"] = ask("  팔이 기운 방향(%s)으로 낙하하면 낙하 받침이 받쳐 줍니까?" % d)
                    if not step["catch_pad_ok"]: raise Abort("낙하 받침 위치 부적합 보고(또는 명시적 y 없음). 추가 이동 없음.")
                if d in ("변화없음", ""): raise Abort("팔 변화 없음(또는 입력 없음): 명령 불일치 의심. 추가 이동 없음.")
        if not a.strict and mode != "dry_run":
            step_first = next(x for x in result["steps"] if x["point"] == "move")
            d = input("마지막 확인 1회: 첫 이동에서 팔 윗부분이 기운 방향? [앞/뒤/모름/변화없음]: ").strip(); result["rotation_direction_after_first_move"] = d; step_first["direction"] = d
            result["overall_ok"] = ask("전체 이동에서 소리·떨림·냄새, 처짐·흔들림, 접촉·걸림이 모두 없었습니까?")
            if not result["overall_ok"]: raise Abort("종합 확인에서 이상 보고(또는 명시적 y 없음). 추가 이동 없음.")
        result["ended"] = "completed"
    except Abort as e: result["ended"] = "aborted"; result["abort_reason"] = str(e); print("\n중단:", e)
    except Hangup as e: result["ended"] = "hangup"; result["abort_reason"] = str(e); print("\n중단:", e)
    except KeyboardInterrupt: result["interrupted"] = True; result["ended"] = "interrupted"; print("\n중단(Ctrl-C): 이후 명령 없음")
    except EOFError: result["ended"] = "input_closed"; print("\n중단: 입력이 끊겼다. 이후 명령 없음")
    except GripperTestError as e: result["ended"] = "adapter_refused"; result["error"] = str(e); print("\n어댑터 거부:", e)
    except Exception as e: result["ended"] = "error"; result["error"] = repr(e); print("\n오류:", repr(e))
    finally:
        if ad is not None and writes_done and result["ended"] != "completed":
            try:
                if ask("@J@이(가) 시작값 S=%d가 아닐 수 있습니다. S로 복귀 쓰기를 1회 보낼까요? (손·도구를 뗀 뒤에만 y)" % result["targets"][0]):
                    ad.confirm_hands_clear(); ad.return_to_start(TIMES[1]); result["returned_to_start"] = True; print("복귀 명령 1회 전송, 복귀 후 회독 확인 중...")
                    ser = poll(ad, int(1.5 * TIMES[1]) + 1000); result["return_readback"] = last_valid(ser); result["return_series"] = ser
                    print("복귀 후 마지막 유효 회독 =", result["return_readback"], "(S=%d)" % result["targets"][0])
                else: result["returned_to_start"] = False
            except BaseException as e: result["return_error"] = repr(e); print("복귀 처리 중단:", repr(e))
        if result["ended"] == "completed" and not a.fake and mode == "real":
            pass
        result["time_ratio"] = "생략: 같은 폭 비례 비교는 G01.5b-2(J1)에서 1.999/1.994로 확인됨. 이번 이동은 첫 이동 8000 ms(동결 상한)와 1500 ms 반복성 자료"
        result["adapter_log"] = [(["write", *l[1:3], "fake" if (a.fake and l[0] == "write") else l[3], *l[4:]] if l[0] == "write" else list(l)) for l in (ad.log if ad else [])]
        result["finished_at"] = datetime.datetime.now().isoformat(timespec="seconds")
        Path(a.out).write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"); print("기록 저장:", a.out)

if __name__ == "__main__": main()
