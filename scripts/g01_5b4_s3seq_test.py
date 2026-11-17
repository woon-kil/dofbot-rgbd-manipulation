#!/usr/bin/env python3
"""G01.5b-4 S3 순차 방식: J3와 J4를 **겹치지 않게** 한 서보씩 단일 쓰기(Arm_serial_servo_write)로 20° 계단(20→80°)으로 접고 같은 계단으로 편다.
J3를 쓰고 도착을 확인한 뒤 J4를 쓴다(복귀도 같은 순서). 쓰기마다 3초 반응 확인·도착·정착 검사. 나머지 서보는 읽기만. 사람이 입회해 직접 실행(무인 실행 금지).
(S3 v1은 J3 직후 J4 쓰기, v2는 write6가 J4를 움직이지 못했다. J4 단독 단일 쓰기는 항상 성공했다: `docs/*S3-J4-미응답-이슈보고서.md`.)
사용: python3 g01_5b4_s3seq_test.py [--real] [--fake ...]   (기본 dry_run: 읽기만, 쓰기 0회)
"""
import argparse, builtins, datetime, json, signal, subprocess, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gripper_adapter import GripperTestAdapter, GripperTestError
from g01_5b4_joint_test import FakeArm, Hangup, _on_signal, read_valid, last_valid

DELTA = 20; STAIRS = 4; WINDOW = [10, 105]
TIMES = [8000, 1500, 1500, 3000, 3000, 1500, 1500, 1500]      # 단계별 이동 시간(두 관절 공통). 첫 단계 8000, 가장 깊은 단계 진입·첫 펴기 3000
RETURN_MS = 8000; POLL_MS = 100; COUNTDOWN_S = {3: 3, 4: 2}; MAX_CONSEC_NONE = 3; TOL = 3; DRIFT_LIMIT = 2; OTHER_TOL = 3; REACT_WAIT_MS = 3000; FAST_RATIO = 0.5
SETTLE_S = 3   # 가장 깊은 자세 도착 뒤 촬영 전 정지 대기(초)
CAPTURE_CMD = 'source /opt/ros/humble/setup.bash; source ~/dofbot_pro_ws/install/setup.bash; rm -rf /tmp/g01_s4_view; python3 ~/dofbot-g01_5b4/g01_dabai_probe.py /tmp/g01_s4_view 4 dabai > /dev/null; echo capture_exit=$?'
JOINT_DESC = {3: "J3(팔꿈치: 위팔과 아래팔 사이 피치 관절)", 4: "J4(손목 피치: 그리퍼·DaBai가 달린 마지막 링크를 앞뒤로 꺾는 관절)"}


def say(*a, **k): builtins.print(*a, **k, flush=True)
def inp(p): return builtins.input("\a" + p)
def ask(msg): return inp(msg + " [y/N] ").strip().lower() in ("y", "yes")
class Abort(Exception): pass


class FakeArm4(FakeArm):
    """가짜 팔: ignore_ids는 해당 서보 쓰기를 받고도 무시(미응답), floors는 서보별 바닥값, j2_drift=(N, 변화량)은 N번째 쓰기 뒤 J2 이동."""
    def __init__(self, pos, ignore_ids=(), floors=None, j2_drift=None):
        super().__init__(pos); self.ignore = set(ignore_ids); self.floors = dict(floors or {}); self.j2_drift = j2_drift; self.nw = 0
    def Arm_serial_servo_write(self, id, angle, time_ms):
        self.nw += 1
        if id in self.ignore: self.writes.append((id, angle, time_ms)); return
        tgt = max(angle, self.floors[id]) if id in self.floors and angle < self.floors[id] else angle
        super().Arm_serial_servo_write(id, tgt, time_ms); self.writes[-1] = (id, angle, time_ms)
        if self.j2_drift and self.nw == self.j2_drift[0]: self.v0[2] = self._p(2); self.v1[2] = self.pos[2] + self.j2_drift[1]; self.t0[2] = time.monotonic(); self.dur[2] = 800


def targets_for(s): return [s] + [s - DELTA * i for i in range(1, STAIRS + 1)] + [s - DELTA * i for i in range(STAIRS - 1, -1, -1)]


def watch_joint(ads, arm, j, total_ms, start_pos, hold):
    """관절 j(쓰기 대상)와 J2·hold 관절들을 total_ms 동안 읽는다. 반환: (series dict, 반응 여부, 미응답 경고 여부)."""
    ids = sorted(set((2, j) + tuple(hold))); ser = {k: [] for k in ids}; cons = {k: 0 for k in ids}; t0 = time.monotonic(); react = False; warned = False
    while (time.monotonic() - t0) * 1000.0 < total_ms:
        t = round((time.monotonic() - t0) * 1000.0)
        for k in ids:
            v = ads[k].read_position() if k in ads else arm.Arm_serial_servo_read(k)
            ser[k].append((t, v)); cons[k] = cons[k] + 1 if v is None else 0
            if cons[k] >= MAX_CONSEC_NONE: raise Abort("J%d 회독 None이 연속 %d회. 추가 이동 없음." % (k, cons[k]))
            if k == j and v is not None and start_pos is not None and v != start_pos: react = True
        if not react and not warned and t >= REACT_WAIT_MS:
            warned = True; say("\n! 쓰기 후 %d ms 동안 J%d 회독이 변하지 않았다(회독 기준 미응답). 추가 쓰기는 없다. **명령은 이미 보냈으므로 약 10초 동안 팔에 다가가지 마세요.** 계속 읽습니다..." % (REACT_WAIT_MS, j))
        time.sleep(POLL_MS / 1000.0)
    return ser, react, warned


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--fake", action="store_true"); ap.add_argument("--real", action="store_true")
    ap.add_argument("--fake-ignore", type=int, nargs="*", default=[]); ap.add_argument("--fake-floor3", type=int, default=None); ap.add_argument("--fake-floor4", type=int, default=None); ap.add_argument("--fake-j2-drift", type=int, default=None); ap.add_argument("--fake-s3", type=int, default=92)
    ap.add_argument("--capture", action="store_true", help="가장 깊은 자세(단계 4의 J4 이동 뒤)에서 DaBai 영상 촬영(S4-1). DaBai 드라이버는 래퍼가 미리 띄운다"); ap.add_argument("--sweep", default=None, help="--capture와 함께: 촬영 대신 노출 스윕(예: 15,25,35,50,70,100). DaBai 서비스로 노출을 바꿔 가며 프레임 저장"); ap.add_argument("--run-id", default="local"); ap.add_argument("--out", default="/tmp/g01_5b4_s3seq_result.json"); a = ap.parse_args()
    if a.fake and a.real: sys.exit("--fake와 --real은 함께 쓸 수 없다")
    mode = "fake" if a.fake else ("real" if a.real else "dry_run")
    result = {"run_id": a.run_id, "mode": mode, "kind": "s3seq", "stage": "G01.5b-4 S3 순차(J3 후 J4)" + ((" + S4-1b 노출 스윕 " + a.sweep) if (a.capture and a.sweep) else (" + S4-1 촬영" if a.capture else "")), "capture": None, "delta_deg": DELTA, "stairs": STAIRS, "window": WINDOW, "times_ms": TIMES, "return_ms": RETURN_MS, "started_at": datetime.datetime.now().isoformat(timespec="seconds"), "state0": None, "steps": [], "interrupted": False, "ended": None}
    ads = {}; arm = None; wrote = False; T = {}
    for sg in (signal.SIGHUP, signal.SIGTERM): signal.signal(sg, _on_signal)
    try:
        say("G01.5b-4 S3 순차 방식(Δ%d° × %d단계, 모드 %s): J3를 쓰고 도착을 확인한 뒤 J4를 쓴다(겹치지 않음). 서보 3·4에만 개별 쓰기, 나머지는 읽기만. 물체 없음." % (DELTA, STAIRS, mode))
        arm = FakeArm4({1: 90, 2: 91, 3: a.fake_s3, 4: 91, 5: 90, 6: 98}, a.fake_ignore, {j: v for j, v in ((3, a.fake_floor3), (4, a.fake_floor4)) if v is not None}, (a.fake_j2_drift, 8) if a.fake_j2_drift else None) if a.fake else __import__("Arm_Lib").Arm_Device()
        state = {i: read_valid(arm, i) for i in range(1, 7)}; result["state0"] = state
        say("b-4-0 현재 회독(SDK 각도): " + ", ".join("J%d=%s" % (i, v) for i, v in state.items()))
        if any(state[i] is None for i in range(1, 6)): raise Abort("J1~J5 중 회독 None이 있다. 쓰기 없음.")
        for j in (1, 5):
            if abs(state[j] - 90) > OTHER_TOL: raise Abort("J%d 회독 %s가 90에서 %d° 넘게 벗어나 있다. 쓰기 없음." % (j, state[j], OTHER_TOL))
        for j in (2, 3, 4):
            if abs(state[j] - 91) > OTHER_TOL: raise Abort("J%d 회독 %s가 직립 근처(91±%d)가 아니다. 쓰기 없음." % (j, state[j], OTHER_TOL))
        for j in (3, 4):
            if state[j] - DELTA * STAIRS < WINDOW[0]: raise Abort("J%d 시작값 %d에서 S-%d가 하한 %d 아래다. 쓰기 없음." % (j, state[j], DELTA * STAIRS, WINDOW[0]))
        today = datetime.date.today()
        for j in (3, 4):
            ads[j] = GripperTestAdapter(arm, {"approved": True, "expires": today.isoformat(), "allowed_times_ms": sorted(set(TIMES + [RETURN_MS])), "first_move_min_ms": 8000, "max_writes": 10, "delta_deg": DELTA, "joint_id": j, "direction": -1, "angle_window": WINDOW, "pattern": "stair", "stair_steps": STAIRS}, today=today, dry_run=(mode == "dry_run"))
            s = ads[j].read_start(); T[j] = targets_for(s); ads[j].set_targets(T[j])
        result["targets"] = {"J3": T[3], "J4": T[4]}
        say("목표(접기→펴기): J3 %s, J4 %s. 단계마다 J3 → J4 순서. J1·J2·J5·J6는 읽기만(시작값 J1=%d, J2=%d, J5=%d, J6=%d)." % (T[3][1:], T[4][1:], state[1], state[2], state[5], state[6]))
        say("\n== 안전 확인(시작 시 1회) ==")
        qs = ["전원 차단 담당자가 현재 자리에 있습니까?", "낙하 받침을 준비했고, 접힌 팔(앞쪽, 보드 위)의 낙하를 받치되 팔 경로(TCP y 약 -15~220 mm, 높이 약 70~460 mm) 안에 들어오지 않습니까?", "팔이 앞쪽으로 접히는 동안 닿을 물체·손·사람·케이블이 팔 주변과 작업대·보드 위에 없습니까?", "팔이 직립(대기) 자세입니까? (J4=91, J3 약 91~92)", "**J4(손목)가 아래팔과 일직선이고 그리퍼가 아래팔의 연장선에 있습니까?** (회독이 캐시일 수 있어 눈으로 따로 확인)", "DaBai·그리퍼 케이블이 당겨지지 않고 여유가 있어 보입니까?", "위 회독값과 목표가 맞다."]
        for k, q in enumerate(qs, 1): say("  %d. %s" % (k, q))
        if not ask("위 %d개를 모두 확인했습니까? (y면 이동 시작, 이동마다 소리 뒤 대기 후 움직입니다. J3 이동이 끝나면 J4가 이어집니다)" % len(qs)): raise Abort("사전 확인 미충족. 쓰기 없음.")
        say("\n▶ **지켜볼 관절**: " + JOINT_DESC[3] + " 이동 → 이어서 " + JOINT_DESC[4] + " 이동. 소리가 나면 **그 순간의 관절**을 보세요(J3 때는 팔꿈치, J4 때는 손목). 쓰기마다 소리가 납니다.")
        total_moves = 4 * STAIRS; n = 0
        for i in range(1, 2 * STAIRS + 1):
            ms = TIMES[i - 1]
            for j in (3, 4):
                n += 1; other = 4 if j == 3 else 3; tgt = T[j][i]
                say("\n[이동 %d/%d] 단계 %d의 %s -> %d (time %d ms): %d초 뒤 이동합니다. ▶ **J%d를 지켜보세요.** 이상하면 Ctrl-C." % (n, total_moves, i, "J%d" % j, tgt, ms, COUNTDOWN_S[j], j)); time.sleep(COUNTDOWN_S[j])
                step = {"index": n, "stair": i, "joint": j, "target": tgt, "time_ms": ms}
                if mode == "dry_run": step["point"] = "dry_run_move"; result["steps"].append(step); say("  dry_run: 쓰기 없음"); continue
                before = ads[j].read_position(); hold_before = ads[other].read_position()
                ads[j].confirm_hands_clear(); ads[j].move(tgt, ms); wrote = True
                total = int(1.5 * ms) + 1000
                ser, react, warned = watch_joint(ads, arm, j, total, before, (other,))
                step["point"] = "move"; step["series"] = {str(k): v for k, v in ser.items()}; result["steps"].append(step)
                if warned:
                    step["visual_after_unresponsive"] = inp("진단용 질문 1개: 지금까지 J%d가 눈으로 보기에 움직였습니까? [움직임/안 움직임/모름]: " % j).strip()
                    if not react: raise Abort("J%d 쓰기 후 무반응(회독 기준). 육안 답: %s. 추가 쓰기 없음." % (j, step["visual_after_unresponsive"] or "없음"))
                hold_target = T[4][i - 1] if j == 3 else T[3][i]   # 이번 쓰기에서 명령하지 않은 관절이 유지해야 할 위치
                fin = {k: last_valid(v) for k, v in ser.items()}; step["final"] = {str(k): v for k, v in fin.items()}
                others = {k: read_valid(arm, k) for k in (1, 5, 6)}; step["others"] = {str(k): v for k, v in others.items()}
                say("  -> 최종 회독 J%d=%s(목표 %d), J%d=%s(유지 %d), J2=%s, J1=%s, J5=%s, J6=%s" % (j, fin[j], tgt, other, fin[other], hold_target, fin[2], others[1], others[5], others[6]))
                reach = next((t_ for t_, v in ser[j] if v is not None and abs(v - tgt) <= TOL), None); step["reach_within_tol_ms"] = reach
                if reach is not None and reach < FAST_RATIO * ms: raise Abort("J%d가 목표 ±%d°에 %d ms에 도달했다(명령 %d ms의 %.2f배, 하한 %.1f배). 시간 값이 적용되지 않았을 의심: 추가 이동 없음." % (j, TOL, reach, ms, reach / ms, FAST_RATIO))
                if fin[j] is None or abs(fin[j] - tgt) > TOL: step["arrival_failed"] = j; raise Abort("J%d 도착 실패(마지막 유효 회독 %s, 목표 %d, 허용 %d°). 추가 이동 없음." % (j, fin[j], tgt, TOL))
                w = [v for t_, v in ser[j] if v is not None and t_ >= ser[j][-1][0] - 1000]
                if w and max(w) - min(w) >= DRIFT_LIMIT: raise Abort("J%d 정착 뒤 1초 회독 변화 %d° (한계 %d°, 헌팅·처짐 의심). 추가 이동 없음." % (j, max(w) - min(w), DRIFT_LIMIT))
                if fin[other] is None or abs(fin[other] - hold_target) > OTHER_TOL: raise Abort("명령하지 않은 J%d 회독 %s가 유지 위치 %d에서 %d° 넘게 벗어났다. 추가 이동 없음." % (other, fin[other], hold_target, OTHER_TOL))
                if fin[2] is None or abs(fin[2] - state[2]) > OTHER_TOL: raise Abort("J2(명령하지 않은 관절) 회독 %s가 시작값 %d에서 %d° 넘게 벗어났다(하중 변화 처짐 의심). 추가 이동 없음." % (fin[2], state[2], OTHER_TOL))
                for k in (1, 5, 6):
                    if others[k] is None or abs(others[k] - state[k]) > OTHER_TOL: raise Abort("J%d 회독 %s가 시작값 %d에서 %d° 넘게 벗어났다. 추가 이동 없음." % (k, others[k], state[k], OTHER_TOL))
                if a.capture and i == STAIRS and j == 4:
                    say("\n[촬영] 가장 깊은 자세에서 팔이 정지했다. %d초 정지 뒤 DaBai %s을 촬영합니다(%s). **팔·보드·케이블을 건드리지 마세요.**" % (SETTLE_S, "노출 스윕(%s)" % a.sweep if a.sweep else "영상", "약 25초" if a.sweep else "약 10초")); time.sleep(SETTLE_S)
                    pre = {k: ads[k].read_position() if k in ads else arm.Arm_serial_servo_read(k) for k in (2, 3, 4)}
                    if a.fake: result["capture"] = {"fake": True, "pre_pose": pre}; say("  (가짜) 촬영 생략")
                    else:
                        cmd = CAPTURE_CMD if not a.sweep else 'source /opt/ros/humble/setup.bash; source ~/dofbot_pro_ws/install/setup.bash; rm -rf /tmp/g01_s4_view; python3 ~/dofbot-g01_5b4/g01_dabai_expsweep.py /tmp/g01_s4_view %s dabai > /tmp/g01_s4_sweep.log 2>&1; echo capture_exit=$?' % a.sweep
                        t0 = time.monotonic(); rr = subprocess.run(["bash", "-lc", cmd], capture_output=True, text=True, timeout=120)
                        post = {k: ads[k].read_position() if k in ads else arm.Arm_serial_servo_read(k) for k in (2, 3, 4)}
                        result["capture"] = {"returncode": rr.returncode, "stdout_tail": rr.stdout[-200:], "stderr_tail": rr.stderr[-200:], "seconds": round(time.monotonic() - t0, 1), "pre_pose": pre, "post_pose": post, "settle_s": SETTLE_S}
                        say("  촬영 종료(%s, %.1f초). 자세 촬영 전 %s, 후 %s" % (rr.stdout.strip().splitlines()[-1] if rr.stdout.strip() else "출력 없음", time.monotonic() - t0, pre, post))
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
        if ads and wrote and result["ended"] != "completed":
            try:
                if ask("J3·J4가 시작값이 아닐 수 있습니다. **순서대로** 시작값으로 복귀(J3 → 도착 확인 → J4, 각 %d ms)할까요? (손·도구를 뗀 뒤에만 y)" % RETURN_MS):
                    done = []; rb = {}
                    for j in (3, 4):
                        try:
                            before = ads[j].read_position(); ads[j].confirm_hands_clear(); ads[j].return_to_start(RETURN_MS); done.append(j); say("J%d 복귀 명령 전송, 복귀 확인 중..." % j)
                            t0 = time.monotonic(); last = None; moved = False
                            while (time.monotonic() - t0) * 1000.0 < int(1.5 * RETURN_MS) + 1000:
                                v = ads[j].read_position(); last = v if v is not None else last; moved = moved or (v is not None and before is not None and v != before); time.sleep(POLL_MS / 1000.0)
                            rb[str(j)] = last; say("  J%d 복귀 후 회독 %s (시작 %d)%s" % (j, last, T[j][0], "" if moved or before == T[j][0] else "  ! 회독이 변하지 않았다"))
                        except GripperTestError as e: say("J%d 복귀 거부: %s" % (j, e)); result.setdefault("return_refused", {})[str(j)] = str(e)
                    result["returned_to_start"] = done; result["return_readback"] = rb
                else: result["returned_to_start"] = False
            except BaseException as e: result["return_error"] = repr(e); say("복귀 처리 중단:", repr(e))
        result["adapter_log"] = {str(j): [(["write", *l[1:3], "fake" if (a.fake and l[0] == "write") else l[3], *l[4:]] if l[0] == "write" else list(l)) for l in ads[j].log] for j in ads}
        result["finished_at"] = datetime.datetime.now().isoformat(timespec="seconds")
        Path(a.out).write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"); say("기록 저장:", a.out)


if __name__ == "__main__": main()
