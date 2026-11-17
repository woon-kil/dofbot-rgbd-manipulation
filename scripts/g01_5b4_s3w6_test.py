#!/usr/bin/env python3
"""G01.5b-4 S3(write6 방식): 제조사 Arm_serial_servo_write6로 J3+J4를 20° 계단(20→80°)으로 접고 같은 계단으로 펴며, 움직이지 않는 서보(J1·J2·J5·J6)에는 시작 읽기값을 넣는다.
--noop 은 여섯 서보에 현재값만 넣은 write6 1회(8000 ms)로 아무것도 움직이지 않는지 확인한다(S3 전 필수). 사람이 입회해 직접 실행(무인 실행 금지).
사용: python3 g01_5b4_s3w6_test.py [--noop] [--real] [--fake ...]   (기본 dry_run: 읽기만, 쓰기 0회)
빠른 방식: 시작 안전 확인 1회, 이동 직전 안내 줄(+소리) + 3초 대기, 끝에 종합 확인 1회. 자동 중단: 도착 실패(3°), None 3연속, 쓰기 후 3초 안에 J3·J4 무반응, 정착 뒤 1초 변화 2°, J1·J2·J5·J6가 시작값에서 3° 이탈.
"""
import argparse, builtins, datetime, json, signal, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gripper_adapter import SixServoTestAdapter, GripperTestError
from g01_5b4_joint_test import FakeArm, Hangup, _on_signal, read_valid, last_valid

DELTA = 20; STAIRS = 4; WINDOW = [10, 105]; MOVING = (3, 4)   # --probe 로 바뀐다(진단용 단독/소폭 프로브)
TIMES = [8000, 1500, 1500, 3000, 3000, 1500, 1500, 1500]
JOINT_DESC = {3: "J3(팔꿈치: 위팔과 아래팔 사이 피치 관절)", 4: "J4(손목 피치: 그리퍼·DaBai가 달린 마지막 링크를 앞뒤로 꺾는 관절, 아래팔과 그리퍼 사이)"}
RETURN_MS = 8000; POLL_MS = 100; COUNTDOWN_S = 3; MAX_CONSEC_NONE = 3; TOL = 3; DRIFT_LIMIT = 2; OTHER_TOL = 3; REACT_WAIT_MS = 3000; FAST_RATIO = 0.5; NOOP_TOL = 1; NOOP_WATCH_MS = int(1.5 * 8000) + 1000   # 명령 시간 8000 ms 기준 공식(1.5×+1 s)


def say(*a, **k): builtins.print(*a, **k, flush=True)
def inp(p): return builtins.input("\a" + p)
def ask(msg): return inp(msg + " [y/N] ").strip().lower() in ("y", "yes")
class Abort(Exception): pass


class FakeArm6s(FakeArm):
    """가짜 팔(write6 지원): 여섯 값을 서보별로 적용. ignore_ids는 해당 서보 값을 받고도 무시(미응답 시험), floors는 서보별 바닥값, j2_drift=(N, 변화량)은 N번째 쓰기 뒤 J2 이동."""
    def __init__(self, pos, ignore_ids=(), floors=None, j2_drift=None, fast=False):
        super().__init__(pos); self.fast = fast; self.ignore = set(ignore_ids); self.floors = dict(floors or {}); self.j2_drift = j2_drift; self.nw = 0; self.w6 = []
    def Arm_serial_servo_write6(self, s1, s2, s3, s4, s5, s6, time_ms):
        self.w6.append(((s1, s2, s3, s4, s5, s6), time_ms)); self.nw += 1
        for i, v in enumerate((s1, s2, s3, s4, s5, s6), 1):
            if i in self.ignore: continue
            tgt = max(v, self.floors[i]) if i in self.floors and v < self.floors[i] else v
            self.v0[i] = self._p(i); self.v1[i] = tgt; self.t0[i] = time.monotonic(); self.dur[i] = 100 if self.fast else time_ms
        if self.j2_drift and self.nw == self.j2_drift[0]: self.v1[2] = self.pos[2] + self.j2_drift[1]; self.v0[2] = self._p(2); self.t0[2] = time.monotonic(); self.dur[2] = (self.j2_drift[2] if len(self.j2_drift) > 2 else 800)


def read_all(ad, ids): return {j: ad.read_position(j) for j in ids}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--fake", action="store_true"); ap.add_argument("--real", action="store_true"); ap.add_argument("--noop", action="store_true"); ap.add_argument("--probe", choices=["4", "3", "34"], default=None, help="진단용 소폭 프로브: 해당 관절만(또는 J3+J4) 10°, 8000/1500 ms로 접었다 폄(write6)")
    ap.add_argument("--fake-ignore", type=int, nargs="*", default=[], help="가짜: 해당 서보 id의 write6 값을 무시"); ap.add_argument("--fake-floor3", type=int, default=None); ap.add_argument("--fake-floor4", type=int, default=None); ap.add_argument("--fake-j2-drift", type=int, default=None)
    ap.add_argument("--fake-fast", action="store_true", help="가짜: write6 시간이 무시되어 매우 빠르게 이동"); ap.add_argument("--fake-noop-move", action="store_true", help="가짜: noop 시험에서 J2가 2° 움직이는 시나리오")
    ap.add_argument("--run-id", default="local"); ap.add_argument("--out", default="/tmp/g01_5b4_s3w6_result.json"); a = ap.parse_args()
    if a.fake and a.real: sys.exit("--fake와 --real은 함께 쓸 수 없다")
    global DELTA, STAIRS, MOVING, TIMES
    if a.probe: MOVING = tuple(int(c) for c in a.probe); DELTA = 10; STAIRS = 1; TIMES = [8000, 1500]
    mode = "fake" if a.fake else ("real" if a.real else "dry_run"); kind = "noop6" if a.noop else ("probe" + a.probe if a.probe else "s3w6")
    result = {"run_id": a.run_id, "mode": mode, "kind": kind, "stage": "G01.5b-4 S3 write6 " + kind, "moving": list(MOVING), "delta": DELTA, "stairs": STAIRS, "window": WINDOW, "started_at": datetime.datetime.now().isoformat(timespec="seconds"), "state0": None, "steps": [], "interrupted": False, "ended": None}
    ad = None; wrote = False; start = None
    for sg in (signal.SIGHUP, signal.SIGTERM): signal.signal(sg, _on_signal)
    try:
        say("G01.5b-4 S3 write6 방식 %s(모드 %s): 제조사 write6로 여섯 서보에 한 번에 쓴다. 움직이지 않는 서보에는 시작 읽기값을 넣는다. 물체 없음." % ("현재값 유지 확인(noop)" if a.noop else ("진단 프로브(%s만 Δ%d°)" % (",".join("J%d" % j for j in MOVING), DELTA) if a.probe else "J3+J4 동시 접기(Δ20° × 4단계)"), mode))
        arm = FakeArm6s({1: 90, 2: 91, 3: 91, 4: 91, 5: 90, 6: 99}, a.fake_ignore, {j: v for j, v in ((3, a.fake_floor3), (4, a.fake_floor4)) if v is not None}, (a.fake_j2_drift, 8) if a.fake_j2_drift else None, a.fake_fast) if a.fake else __import__("Arm_Lib").Arm_Device()
        if a.fake and a.fake_noop_move: arm.j2_drift = (1, 2, 8000)   # 8000 ms에 걸쳐 2° 이동(느린 이동 시나리오)
        today = datetime.date.today()
        cfg = {"approved": True, "expires": today.isoformat(), "allowed_times_ms": sorted(set(TIMES + [RETURN_MS])), "first_move_min_ms": 8000, "max_writes": 10, "direction": -1, "angle_window": WINDOW}
        cfg.update({"moving_ids": [], "pattern": "hold", "allowed_times_ms": [8000], "max_writes": 1} if a.noop else {"moving_ids": list(MOVING), "pattern": "stair", "delta_deg": DELTA, "stair_steps": STAIRS})
        ad = SixServoTestAdapter(arm, cfg, today=today, dry_run=(mode == "dry_run"))
        start = ad.read_start6(); result["state0"] = start
        say("b-4-0 현재 회독(SDK 각도): " + ", ".join("J%d=%s" % (i, v) for i, v in start.items()))
        for j in (1, 5):
            if abs(start[j] - 90) > OTHER_TOL: raise Abort("J%d 회독 %s가 90에서 %d° 넘게 벗어나 있다. 쓰기 없음." % (j, start[j], OTHER_TOL))
        for j in (2, 3, 4):
            if abs(start[j] - 91) > OTHER_TOL: raise Abort("J%d 회독 %s가 직립 근처(91±%d)가 아니다. 쓰기 없음." % (j, start[j], OTHER_TOL))
        if not a.noop:
            for j in MOVING:
                if start[j] - DELTA * STAIRS < WINDOW[0]: raise Abort("J%d 시작값 %d에서 S-%d가 하한 %d 아래다. 쓰기 없음." % (j, start[j], DELTA * STAIRS, WINDOW[0]))
        T = ad.expected_targets(); ad.set_targets(T); result["targets"] = T
        say("목표 목록(6축, 접기→펴기): " + ("현재값 그대로 1회" if a.noop else " → ".join("(" + ", ".join("J%d=%d" % (j, t[j - 1]) for j in MOVING) + ")" for t in T[1:])) + "\n나머지 서보는 시작 읽기값 그대로 넣는다: " + ", ".join("J%d=%d" % (j, start[j]) for j in range(1, 7) if j not in MOVING) + ".")
        say("\n== 안전 확인(시작 시 1회) ==")
        qs = ["전원 차단 담당자가 현재 자리에 있습니까?"] + ([] if a.noop else [("낙하 받침을 준비했고, 접힌 팔(앞쪽)의 낙하를 받치되 팔 경로 안에 들어오지 않습니까? (프로브는 10° 소폭이라 경로가 작습니다)" if a.probe else "낙하 받침을 준비했고, 접힌 팔(앞쪽, 보드 위)의 낙하를 받치되 팔 경로(TCP y 약 -15~220 mm, 높이 약 70~460 mm) 안에 들어오지 않습니까?"), "팔이 앞쪽으로 접히는 동안 닿을 물체·손·사람·케이블이 팔 주변과 작업대·보드 위에 없습니까?"]) + ["팔이 직립(대기) 자세입니까? (J4=91, J3 약 91~92)"] + ([] if a.noop else ["**J4(손목)가 아래팔과 일직선이고 그리퍼가 아래팔의 연장선에 있습니까?** (회독이 캐시일 수 있어 눈으로 따로 확인)"]) + [ "DaBai·그리퍼 케이블이 당겨지지 않고 여유가 있어 보입니까?", "위 회독값과 목표가 맞다(J1·J2·J5·J6에도 지금 값을 다시 씁니다)."]
        for k, q in enumerate(qs, 1): say("  %d. %s" % (k, q))
        if not ask("위 %d개를 모두 확인했습니까? (y면 이동 시작, 이후 이동마다 소리 뒤 3초 대기)" % len(qs)): raise Abort("사전 확인 미충족. 쓰기 없음.")
        n = 1 if a.noop else 2 * STAIRS
        if not a.noop: say("\n▶ 이번 시험에서 **지켜볼 관절**(이동 시작 뒤 약 13초 동안 계속 보세요, 육안 질문이 그때 나옵니다): " + ", ".join(JOINT_DESC[j] for j in MOVING) + ". 이동 직전 소리가 나면 그 관절(과 아래팔·그리퍼 방향 변화)을 눈으로 보세요." + (" 첫 이동 직후 그 관절이 움직였는지 묻습니다." if a.probe else ""))
        for i in range(1, n + 1):
            tg = T[i]; ms = 8000 if a.noop else TIMES[i - 1]
            say("\n[이동 %d/%d] %s (time %d ms): %d초 뒤 이동합니다. 육안으로 보다가 이상하면 Ctrl-C." % (i, n, "현재값 유지(write6, 아무것도 안 움직여야 함)" if a.noop else ", ".join("J%d->%d" % (j, tg[j - 1]) for j in MOVING) + " (나머지는 시작값 유지) ▶ " + ", ".join("J%d" % j for j in MOVING) + "를 지켜보세요", ms, COUNTDOWN_S)); time.sleep(COUNTDOWN_S)
            step = {"index": i, "target6": list(tg), "time_ms": ms}
            if mode == "dry_run": step["point"] = "dry_run_move"; result["steps"].append(step); say("  dry_run: 쓰기 없음"); continue
            before = {j: ad.read_position(j) for j in MOVING}
            ad.confirm_hands_clear(); ad.move6(tg, ms); wrote = True
            ser = {j: [] for j in range(1, 7)}; cons = {j: 0 for j in range(1, 7)}; t0 = time.monotonic(); react = {j: False for j in MOVING}; total = (NOOP_WATCH_MS if a.noop else int(1.5 * ms) + 1000); unresp = False
            watch = (1, 2, 3, 4, 5, 6) if a.noop else tuple(sorted(set((2,) + MOVING)))
            while (time.monotonic() - t0) * 1000.0 < total:
                t = round((time.monotonic() - t0) * 1000.0)
                for j in watch:
                    v = ad.read_position(j); ser[j].append((t, v)); cons[j] = cons[j] + 1 if v is None else 0
                    if cons[j] >= MAX_CONSEC_NONE: raise Abort("J%d 회독 None이 연속 %d회. 추가 이동 없음." % (j, cons[j]))
                    if v is not None and not a.noop and j in MOVING and before[j] is not None and v != before[j]: react[j] = True
                if not a.noop and not unresp and t >= REACT_WAIT_MS and not all(react.values()):
                    unresp = True; bad = [j for j in MOVING if not react[j]]
                    say("\n! 쓰기 후 %d ms 동안 J%s 회독이 변하지 않았다(회독 기준 미응답). 추가 쓰기는 없다. **명령은 이미 보냈으므로 약 10초 동안 팔에 다가가지 마세요.** 계속 읽습니다..." % (REACT_WAIT_MS, ",J".join(map(str, bad)))); step["unresponsive"] = bad
                time.sleep(POLL_MS / 1000.0)
            result["steps"].append(step); step["point"] = "move"; step["series"] = {str(j): v for j, v in ser.items() if v}
            if unresp:
                step["visual_after_unresponsive"] = inp("진단용 질문 1개: 지금까지 J%s가 눈으로 보기에 움직였습니까? [움직임/안 움직임/모름]: " % ",J".join(map(str, step["unresponsive"]))).strip()
                if not all(react.values()): raise Abort("J%s 쓰기 후 무반응(회독 기준). 육안 답: %s. 추가 쓰기 없음." % (",J".join(map(str, step["unresponsive"])), step["visual_after_unresponsive"] or "없음"))
            if a.probe and i == 1 and mode != "dry_run":
                step["visual_moving_joints"] = inp("방금 이동에서 %s가 눈으로 보기에 움직였습니까? (아래팔 전체가 기운 것이 아니라 그 관절 자체) [움직임/안 움직임/모름]: " % ", ".join("J%d" % j for j in MOVING)).strip()
            fin = {j: last_valid(ser[j]) for j in watch}
            for j in (1, 2, 3, 4, 5, 6):
                if j not in fin: fin[j] = read_valid(arm, j); step["final"] = {str(j): v for j, v in fin.items()}
            say("  -> 최종 회독 " + ", ".join("J%d=%s" % (j, fin[j]) for j in sorted(fin)))
            for j in [x for x in (1, 2, 3, 4, 5, 6) if x not in MOVING] if not a.noop else (1, 2, 3, 4, 5, 6):
                lim = NOOP_TOL if a.noop else OTHER_TOL
                if fin.get(j) is None or abs(fin[j] - start[j]) > lim: raise Abort("J%d 회독 %s가 시작값 %d에서 %d° 넘게 벗어났다(%s). 추가 이동 없음." % (j, fin.get(j), start[j], lim, "현재값 write6가 서보를 움직임" if a.noop else "하중 변화 또는 현재값 재쓰기 영향 의심"))
            if a.noop:
                step["max_dev"] = {str(j): max(abs(v - start[j]) for _, v in ser[j] if v is not None) for j in watch}; say("  noop 확인: %d초 동안 각 서보의 시작값 대비 최대 편차 " % (NOOP_WATCH_MS // 1000) + str(step["max_dev"]))
                bad = [j for j, d in step["max_dev"].items() if d > NOOP_TOL]
                if bad: raise Abort("noop 중 J%s가 시작값에서 %d° 넘게 움직였다(구간 최대 편차). write6 현재값 쓰기가 서보를 움직임." % (",J".join(bad), NOOP_TOL))
                continue
            for j in MOVING:
                reach = next((t_ for t_, v in ser[j] if v is not None and abs(v - tg[j - 1]) <= TOL), None); step.setdefault("reach_within_tol_ms", {})[str(j)] = reach
                if reach is not None and reach < FAST_RATIO * ms: raise Abort("J%d가 목표 ±%d°에 %d ms에 도달했다(명령 %d ms의 %.2f배, 하한 %.1f배). 시간 값이 적용되지 않았을 의심: 추가 이동 없음." % (j, TOL, reach, ms, reach / ms, FAST_RATIO))
                if fin[j] is None or abs(fin[j] - tg[j - 1]) > TOL: step["arrival_failed"] = j; raise Abort("J%d 도착 실패(마지막 유효 회독 %s, 목표 %d, 허용 %d°). 추가 이동 없음." % (j, fin[j], tg[j - 1], TOL))
                w = [v for t_, v in ser[j] if v is not None and t_ >= ser[j][-1][0] - 1000]
                if w and max(w) - min(w) >= DRIFT_LIMIT: raise Abort("J%d 정착 뒤 1초 회독 변화 %d° (한계 %d°, 헌팅·처짐 의심). 추가 이동 없음." % (j, max(w) - min(w), DRIFT_LIMIT))
        if mode != "dry_run":
            result["overall_ok"] = ask("마지막 확인 1회: " + ("움직임·소리·열 이상이 없었습니까?" if a.noop else "전체 이동에서 소리·떨림·냄새, 처짐·흔들림, 접촉·걸림, 케이블 당김이 모두 없었습니까?"))
            if not result["overall_ok"]: raise Abort("종합 확인에서 이상 보고(또는 명시적 y 없음). 추가 이동 없음.")
        result["ended"] = "completed"
    except Abort as e: result["ended"] = "aborted"; result["abort_reason"] = str(e); say("\n중단:", e)
    except Hangup as e: result["ended"] = "hangup"; result["abort_reason"] = str(e); say("\n중단:", e)
    except KeyboardInterrupt: result["interrupted"] = True; result["ended"] = "interrupted"; say("\n중단(Ctrl-C): 이후 명령 없음(이미 보낸 이동은 서보가 끝까지 간다)")
    except EOFError: result["ended"] = "input_closed"; say("\n중단: 입력이 끊겼다. 이후 명령 없음")
    except GripperTestError as e: result["ended"] = "adapter_refused"; result["error"] = str(e); say("\n어댑터 거부:", e)
    except Exception as e: result["ended"] = "error"; result["error"] = repr(e); say("\n오류:", repr(e))
    finally:
        if ad is not None and wrote and not a.noop and result["ended"] != "completed":
            try:
                if ask("팔이 시작 자세가 아닐 수 있습니다. 6축을 시작값으로 복귀(write6 1회, %d ms)할까요? (손·도구를 뗀 뒤에만 y)%s" % (RETURN_MS, " [프로브: 정지 자세를 사진으로 남기려면 **지금 답하지 말고** 에이전트에게 촬영을 요청하세요]" if a.probe else "")):
                    ad.confirm_hands_clear(); ad.return_to_start6(RETURN_MS); result["returned_to_start"] = True; say("복귀 명령 전송, 복귀 후 회독 확인 중...")
                    t0 = time.monotonic(); rs = []
                    while (time.monotonic() - t0) * 1000.0 < int(1.5 * RETURN_MS) + 1000: rs.append(read_all(ad, MOVING)); time.sleep(POLL_MS / 1000.0)
                    result["return_readback"] = {str(j): next((r[j] for r in reversed(rs) if r[j] is not None), None) for j in MOVING}; say("복귀 후 마지막 유효 회독:", result["return_readback"], "(시작 " + ", ".join("J%d=%d" % (j, start[j]) for j in MOVING) + ")")
                else: result["returned_to_start"] = False
            except BaseException as e: result["return_error"] = repr(e); say("복귀 처리 중단:", repr(e))
        result["adapter_log"] = [[("fake" if (a.fake and l[0] == "write6" and len(l) > 3 and l[3] == "real") else x) if k == 3 and l[0] == "write6" else x for k, x in enumerate(l)] for l in (ad.log if ad else [])]
        result["finished_at"] = datetime.datetime.now().isoformat(timespec="seconds")
        Path(a.out).write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"); say("기록 저장:", a.out)


if __name__ == "__main__": main()
