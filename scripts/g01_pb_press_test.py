#!/usr/bin/env python3
"""G07 P-B J6 연속 누름 시험(서보 6만, 팔 관절은 명령하지 않는다). 사람이 입회해 직접 실행하는 대화형 스크립트(무인 실행 금지).
승인 요청 문서: docs/gates/G07-PB-J6-연속-누름-시험-승인-요청-문서-초안.md (PB-1·PB-2·PB-3·PB-5 승인 2026-11-17, PB-4 블록 정체 미답, `--real`은 도구·가짜 시험·dry run 뒤 별도 승인)
사용: python3 g01_pb_press_test.py --fake [--fake-speed 40] [--fake-stall-at 146]   (가짜 서보, 하드웨어 없음)
      python3 g01_pb_press_test.py                              (실제 SDK 읽기, 쓰기는 dry_run: 쓰기 0회, 입력 1번)
      python3 g01_pb_press_test.py --real                       (실제 쓰기. 확인 문구를 직접 입력해야 한다)
구간: (열기, 시작이 오므림일 때만) 179→160→140→120→100 → (블록 없음 대조 1회, 10 s) → (블록 있음 사다리 10→20→30 s) → (마무리) 100→120→140→160→179 [블록 제거 뒤].
한 시도: 100→120→130→135→140→145→150으로 닫고, 명령 150에서 150 도달 뒤 2 s 정착한 다음 T초 읽기만 한다(추가 쓰기 없음). 그 뒤 100으로 열기(3000 ms).
누름 시간 코드 상한: PressBudget이 150 쓰기 시점부터 센다. 한 시도 총 누름 ≤ T + 2.5 s (30 s 단계는 32.5 s).
"""
import argparse, datetime, json, signal, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gripper_adapter import GripperTestError
from g01_5a_gripper_test import Abort, Hangup, _on_signal, ask, POLL_MS
import g01_p2h_gripper_test as H
from g01_p2h_gripper_test import poll, make_adapter, START_CLOSED, START_OPEN, ADAPTERS
from g01_p2g_gripper_test import return_time, QUIET_S
from g01_p2i_gripper_test import FastFakeBlock, tail5, OPEN_STEPS, FINISH_STEPS, WIN_OPEN, WIN_FINISH, HOLD_EXTRA_MS
from press_budget import PressBudget, BudgetExceeded

CLOSE_STEPS = [120, 130, 135, 140, 145, 150]     # Run 3·4와 같은 닫기 계단(시작 S=100)
WIN_PRESS = (95, 150)                            # 누르는 구간 각도 창(PB-1: P2i의 155는 쓰지 않는다)
LADDER_S = [10, 20, 30]                          # PB-1 승인. 40 s는 결과를 본 뒤 별도 승인
CONTROL_S = 10                                   # PB-2 블록 없음 대조(발열 비교)
REST_S = 90                                      # PB-1 시도 사이 휴식
CRUSH_RISE = 2                                   # PB-1: 누름 중 마지막 5회 읽기가 단조 증가하고 정체값보다 이만큼 오르면 눌림(문서 '2° 넘게'를 보수적으로 ≥2로 구현)
E3_MAX = 1                                       # PB-1: 누름 중 회독 최대−최소가 이를 넘으면 기록하고 사다리를 멈춘다(즉시 열기는 아님)
RETURN_MS = 3000                                 # 150→100 = 50° × 60 ms/° = 3000 ms
BUDGET_EXTRA_S = 2.5                             # 150 쓰기 뒤 정착 2.0 s + 여유 0.5 s
PHRASE = "APPROVE PB REAL"


def e3_range(vals):
    v = [x for x in vals if x is not None]
    return (max(v) - min(v)) if v else None


def hold_check(ref, vals):
    """누름 중 판정. vals는 누름 구간 회독 목록. 반환 (상태, 사유): 'ok' | 'read_none' | 'crush'."""
    if vals and vals[-1] is None: return "read_none", "회독이 None이다(읽기 실패)"
    t = [v for v in vals if v is not None][-5:]
    if ref is not None and len(t) == 5 and all(b >= a for a, b in zip(t[:-1], t[1:])) and t[-1] - ref >= CRUSH_RISE and t[-1] > t[0]:
        return "crush", "누름 중 회독이 정체값 %d에서 계속 올라 %s에 이르렀다(블록 눌림·미끄러짐 의심)" % (ref, t)
    return "ok", None


class PBFake(FastFakeBlock):
    """P-B 가짜 서보: wobble이면 누르는 동안 회독이 번갈아 wobble°씩 뛴다(E3 시험), none_after면 N번 읽은 뒤 None을 돌려준다."""
    def __init__(self, v, speed): super().__init__(v, speed); self.wobble = 0; self.none_after = None; self._n = 0
    def Arm_serial_servo_read(self, id):
        v = super().Arm_serial_servo_read(id)
        if self._held and v is not None:
            self._n += 1
            if self.none_after is not None and self._n > self.none_after: return None
            if self.wobble and self._n % 2: return v + self.wobble
        return v


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--fake", action="store_true"); ap.add_argument("--real", action="store_true")
    ap.add_argument("--fake-dry", action="store_true"); ap.add_argument("--fake-speed", type=int, default=1); ap.add_argument("--fake-start", type=int, default=179)
    ap.add_argument("--fake-stall-at", type=int, default=None); ap.add_argument("--fake-creep", action="store_true"); ap.add_argument("--fake-wobble", type=int, default=0)
    ap.add_argument("--fake-none-after", type=int, default=None); ap.add_argument("--fake-no-hold-end", action="store_true", help="--fake에서만: 누름 읽기 종료 조건을 끈다(PressBudget 상한 시험)")
    ap.add_argument("--no-control", action="store_true", help="블록 없음 대조 1회를 생략한다(PB-2는 넣는 것이 기본)")
    ap.add_argument("--run-id", default="local"); ap.add_argument("--out", default="/tmp/g01_pb_result.json"); a = ap.parse_args()
    if a.fake and a.real: sys.exit("--fake와 --real은 함께 쓸 수 없다")
    if (a.fake_dry or a.fake_speed != 1 or a.fake_stall_at or a.fake_creep or a.fake_wobble or a.fake_none_after is not None or a.fake_no_hold_end or a.fake_start != 179) and not a.fake: sys.exit("--fake-* 옵션은 --fake에서만 쓸 수 있다")
    mode = "dry_run" if (a.fake and a.fake_dry) else ("fake" if a.fake else ("real" if a.real else "dry_run"))
    dry = (mode == "dry_run"); sp = a.fake_speed if a.fake else 1
    result = {"run_id": a.run_id, "mode": mode, "started_at": datetime.datetime.now().isoformat(timespec="seconds"), "windows": {"open": list(WIN_OPEN), "press": list(WIN_PRESS), "finish": list(WIN_FINISH)}, "ladder_s": LADDER_S, "control_s": None if a.no_control else CONTROL_S, "rest_s": REST_S, "limits": {"crush_rise": CRUSH_RISE, "e3_max": E3_MAX, "budget_extra_s": BUDGET_EXTRA_S}, "phases": [], "trials": [], "interrupted": False, "ended": None, "ladder_stop": None}
    ad = None; cur = None; start_of_ad = None; writes_done = False
    for sg in (signal.SIGHUP, signal.SIGTERM): signal.signal(sg, _on_signal)

    def move_steps(label, adapter, targets, times):
        """열기·마무리 구간: 승인된 목록을 순서대로 이동하고 단계마다 정착 확인(도착 실패면 중단)."""
        nonlocal cur, writes_done
        adapter.set_targets(targets); steps = []
        for i, (tgt, t_ms) in enumerate(zip(targets[1:], times), 1):
            adapter.confirm_hands_clear(); time.sleep(QUIET_S); adapter.move(tgt, t_ms); writes_done = writes_done or not dry; cur = tgt
            if dry: steps.append({"n": i, "angle": tgt, "time_ms": t_ms, "dry": True}); continue
            stop = lambda sr, el, _t=tgt, _m=t_ms: el >= _m / sp and len(tail5(sr)) == 5 and max(tail5(sr)) - min(tail5(sr)) <= 1 and abs(_t - tail5(sr)[-1]) <= 3
            series = poll(adapter, int(2 * t_ms + 1000) // sp, POLL_MS / sp, stop=stop)
            last = next((v for _, v in reversed(series) if v is not None), None); steps.append({"n": i, "angle": tgt, "time_ms": t_ms, "final_readback": last, "series": series})
            print("  [%s] 이동 %d/%d: %d (%d ms) → 회독 %s" % (label, i, len(targets) - 1, tgt, t_ms, last))
            if last is None or abs(last - tgt) > 3: raise Abort("도착 실패(회독 %s, 목표 %d, 허용 3°). 추가 이동 없음." % (last, tgt))
        return steps

    def go_home(adapter, s):
        nonlocal cur
        adapter.confirm_hands_clear(); time.sleep(QUIET_S); adapter.return_to_start(RETURN_MS); cur = s
        stop = lambda sr, el, _s=s: el >= RETURN_MS / sp and len(tail5(sr)) == 5 and max(tail5(sr)) - min(tail5(sr)) <= 1 and abs(_s - tail5(sr)[-1]) <= 3
        series = poll(adapter, (2 * RETURN_MS + 1000) // sp, POLL_MS / sp, stop=stop); return next((v for _, v in reversed(series) if v is not None), None)

    def run_trial(label, T, block):
        """한 시도: 닫기 계단 → 150 도달·정착 → T초 읽기만 → 열기. 누름 중 이상(눌림·None·상한 초과)이면 즉시 열고 Abort. 반환 시도 기록(E3 초과는 기록만)."""
        nonlocal cur, start_of_ad, writes_done, ad
        ad = make_adapter(arm, WIN_PRESS, 8, dry); s = ad.read_start(); cur = s; start_of_ad = s
        if s not in START_OPEN: raise Abort("누르기 시작 회독 S=%d가 100(±1)이 아니다." % s)
        targets = [s] + CLOSE_STEPS; ad.set_targets(targets)
        tr = {"label": label, "T_s": T, "block": block, "steps": [], "hold": None, "e3": None, "e3_exceeded": False, "stall_step": None, "crush": None, "abort": None}
        result["trials"].append(tr)
        if dry: tr["dry"] = True; return tr
        budget = PressBudget(limit_s=(T + BUDGET_EXTRA_S) / sp)
        ref = None
        for i, tgt in enumerate(CLOSE_STEPS, 1):
            ad.confirm_hands_clear(); time.sleep(QUIET_S); ad.move(tgt, 1500); writes_done = True; cur = tgt
            last_step = (tgt == 150)
            if last_step: budget.start()                  # 누름 시간 계산은 150 쓰기 시점부터(이전 단계의 가벼운 접촉은 시도마다 이미 P2i·Run 3·4 범위)
            if last_step: stop = lambda sr, el, _h=int(1500 + HOLD_EXTRA_MS) / sp: el >= _h
            else: stop = lambda sr, el, _t=tgt: el >= 1500 / sp and len(tail5(sr)) == 5 and max(tail5(sr)) - min(tail5(sr)) <= 1 and (abs(_t - tail5(sr)[-1]) <= 3 or (block and _t - tail5(sr)[-1] >= 3))
            series = poll(ad, int(max(2 * 1500 + 1000, 0)) // sp, POLL_MS / sp, stop=stop)
            last = next((v for _, v in reversed(series) if v is not None), None); t5 = tail5(series)
            st = {"n": i, "angle": tgt, "final_readback": last, "tail5": t5, "series": series}; tr["steps"].append(st)
            print("  [%s] 이동 %d/%d: %d → 회독 %s" % (label, i, len(CLOSE_STEPS), tgt, last))
            if last is None: st["arrival_failed"] = True; tr["abort"] = "회독 None"; break
            stalled = len(t5) == 5 and max(t5) - min(t5) <= 1 and tgt - last >= 3
            if stalled and tr["stall_step"] is None: tr["stall_step"] = tgt
            if not block and (abs(tgt - last) > 3):
                st["free_contact_suspect"] = True; tr["abort"] = "블록 없음 구간인데 명령 %d에서 회독 %s(잔차 %d°). 손가락 사이에 무언가 있거나 서보 이상이 의심된다" % (tgt, last, tgt - last); break
            if block and not stalled and abs(tgt - last) > 3: st["arrival_failed"] = True; tr["abort"] = "도착 실패(회독 %s, 목표 %d)" % (last, tgt); break
            if last_step: ref = last
        if tr["abort"] is None:
            # 150 도달 뒤 T초 읽기만(추가 쓰기 없음). 블록 없음 대조는 눌림 규칙을 적용하지 않고 기록만 한다
            hold = []; t0 = time.monotonic(); tr["hold"] = {"ref": ref, "series": hold}
            while True:
                el = time.monotonic() - t0
                hold.append((round(el * 1000), ad.read_position()))
                vals = [v for _, v in hold]
                status, why = hold_check(ref, vals) if block else (("read_none", "회독이 None이다(읽기 실패)") if vals[-1] is None else ("ok", None))
                if status != "ok": tr["abort"] = why; tr["crush"] = why if status == "crush" else None; break
                try: budget.check()
                except BudgetExceeded as e: tr["abort"] = str(e); break
                if not a.fake_no_hold_end and el >= T / sp: break
                time.sleep(POLL_MS / 1000.0 / sp)
            e3 = e3_range([v for _, v in hold]); tr["e3"] = e3; tr["e3_exceeded"] = bool(e3 is not None and e3 > E3_MAX)
            tr["hold"]["used_s_scaled"] = round(budget.used_s() * sp, 2); tr["hold"]["n_reads"] = len(hold)
            print("  누름 %.1f s(표시는 실제 시간) 읽기 %d회, 회독 최대−최소(E3) %s%s" % (el * sp, len(hold), e3, "  ★ E3 초과" if tr["e3_exceeded"] else ""))
        tr["return_final"] = go_home(ad, s); writes_done = True; budget.stop()
        if tr["abort"]: print("★ %s. 열기를 마쳤다." % tr["abort"]); raise Abort("시도 %s: %s" % (label, tr["abort"]))
        return tr

    try:
        print("G07 P-B J6 연속 누름 시험(%s): 그리퍼 서보 6만 움직인다. 팔 관절은 명령하지 않는다." % mode)
        print("사전 확인:")
        for k, q in enumerate(("전원 차단 담당자가 현재 자리에 있다", "팔이 K1 기립(대기) 자세이고 낙하 받침이 준비돼 있다(전원 차단 시 직립 팔이 낙하할 수 있음)", "블록 없음 대조 구간 동안 손가락 사이에 아무것도 없다", "블록 홀더(스탠드·집게)와 블록이 준비돼 있다(블록은 대조가 끝난 뒤에 둔다. **등록 블록(마커 id 7)이 아닌 여분 블록 사용 권장, PB-4**)", "누름이 최대 약 30 s까지 간다(서보 발열 미측정). 눌림·소리·냄새·떨림이 보이면 즉시 Ctrl-C, 시도 사이 휴식 %d s를 지킨다" % REST_S), 1): print("  %d. %s" % (k, q))
        if not ask("위 5개를 모두 확인했습니까? (질문은 이 한 번으로 묶었습니다)"): raise Abort("사전 확인 미충족. 명령은 보내지 않았다.")
        if a.real and input("실제 쓰기를 시작하려면 정확히 '%s'를 입력: " % PHRASE).strip() != PHRASE: raise Abort("승인 문구 불일치. 명령은 보내지 않았다.")
        if a.fake:
            arm = PBFake(a.fake_start, sp); arm.creep = a.fake_creep; arm.wobble = a.fake_wobble; arm.none_after = a.fake_none_after
        else:
            from Arm_Lib import Arm_Device; arm = Arm_Device()
        ad = make_adapter(arm, WIN_OPEN, 6, dry, first_min=3000)
        s0 = ad.read_start(); cur = s0; start_of_ad = s0
        skip_open = s0 in START_OPEN
        if s0 not in START_CLOSED and not skip_open: raise Abort("시작 회독 S=%d가 오므림 범위 %s도 열림 범위 %s도 아니다. 명령은 보내지 않았다." % (s0, list(START_CLOSED), list(START_OPEN)))
        open_t = [s0] + OPEN_STEPS; open_ms = [3000] + [1500] * (len(OPEN_STEPS) - 1)
        plan = ([] if a.no_control else [("블록 없음 대조", CONTROL_S, False)]) + [("블록 %d s" % T, T, True) for T in LADDER_S]
        print("시작 회독값 S = %d. 계획: 열기 %s%s → %s → 마무리 %s" % (s0, open_t, ", 생략" if skip_open else "", [(l, T) for l, T, _ in plan], [100] + FINISH_STEPS))
        if dry:
            print("dry_run: 쓰기·이동 없음. 위 계획이 실제 시험의 목록이다(입력은 이 한 번의 확인뿐).")
            result["phases"].append({"phase": "dry_run_plan", "open": open_t, "close_steps": CLOSE_STEPS, "plan": [(l, T, b) for l, T, b in plan], "finish": [100] + FINISH_STEPS}); result["ended"] = "completed"
        else:
            if not ask("이 목록을 확인했고 이대로 진행합니까?"): raise Abort("목표 목록 미확인. 쓰기 없음.")
            if not skip_open:
                steps = move_steps("열기", ad, open_t, open_ms); result["phases"].append({"phase": "open", "targets": open_t, "steps": steps})
                if cur != 100: raise Abort("열기가 100에 도착하지 않았다(%s)." % cur)
            else: result["phases"].append({"phase": "open_skipped", "start": s0})
            first = True; block_placed = False
            for label, T, block in plan:
                if not first: print("시도 사이 휴식 %d초... (열린 상태입니다)" % REST_S); time.sleep(REST_S / sp)
                first = False
                if block and not block_placed:
                    print("\n=== **지금 블록을 홀더에 고정해 손가락 패드 사이(30 mm 폭 방향)에 두세요.** 다음 질문에서 확인합니다. ===")
                    if not ask("블록이 홀더에 고정돼 손가락 패드 사이에 있고, 손·도구가 손가락에서 멀리 있습니까?"): raise Abort("블록·손 확인 미충족. 추가 명령 없음.")
                    block_placed = True
                    if a.fake and a.fake_stall_at is not None: arm.stall_at = a.fake_stall_at
                tr = run_trial(label, T, block)
                if tr.get("e3_exceeded"): result["ladder_stop"] = "E3 초과(%s°) %s에서 사다리를 멈춘다" % (tr["e3"], label); print("★", result["ladder_stop"]); break
                if not ask("시도 %s 직후: J6 서보가 뜨겁거나 소음·냄새·떨림이 있었습니까? (**이상 없으면 y**, 이상이면 N로 사다리를 멈춘다)" % label): result["ladder_stop"] = "사용자 이상 보고(%s)" % label; print("★", result["ladder_stop"]); break
            if a.fake: arm.stall_at = None
            if not ask("블록과 홀더를 손가락 사이에서 치웠습니까? (마무리 닫기를 시작합니다)"): raise Abort("블록 제거 미확인. 추가 명령 없음.")
            ad = make_adapter(arm, WIN_FINISH, 6, dry); s = ad.read_start(); cur = s; start_of_ad = s
            fin_t = [s] + FINISH_STEPS; steps = move_steps("마무리", ad, fin_t, [1500] * len(FINISH_STEPS)); result["phases"].append({"phase": "finish", "targets": fin_t, "steps": steps})
            result["ended"] = "completed"
    except Abort as e:
        result["ended"] = "aborted"; result["abort_reason"] = str(e); print("\n중단:", e)
    except Hangup as e:
        result["ended"] = "hangup"; result["abort_reason"] = str(e); print("\n중단:", e)
    except KeyboardInterrupt:
        result["interrupted"] = True; result["ended"] = "interrupted"; print("\n중단(Ctrl-C): 이후 명령 없음")
    except EOFError:
        result["ended"] = "input_closed"; print("\n중단: 입력이 끊겼다. 이후 명령 없음")
    except GripperTestError as e:
        result["ended"] = "adapter_refused"; result["error"] = str(e); print("\n어댑터 거부:", e)
    except Exception as e:
        result["ended"] = "error"; result["error"] = repr(e); print("\n오류:", repr(e))
    finally:
        if ad is not None and writes_done and result["ended"] != "completed" and start_of_ad is not None:
            try:
                dist = abs((cur if cur is not None else start_of_ad) - start_of_ad); t_ret = return_time(dist)
                opening = start_of_ad <= 101
                if dist == 0 or getattr(ad, "_returned", False): result["returned_to_start"] = None
                elif t_ret is None: print("복귀 폭 %d°가 승인된 시간으로 갈 수 없다. 복귀하지 않는다." % dist); result["returned_to_start"] = False
                elif ask(("그리퍼가 블록을 누르고 있을 수 있습니다(마지막 명령 %s). 열린 쪽 S=%d로 복귀 쓰기를 1회 보낼까요? 이 복귀는 블록을 놓는 **여는 동작**이며 시간 %d ms입니다. 이상(소리·발열·도착 실패·눌림)이 있으면 y를 권장합니다" % (cur, start_of_ad, t_ret)) if opening else ("그리퍼가 이 구간의 시작값 S=%d가 아닐 수 있습니다(마지막 명령 %s). S로 복귀 쓰기를 1회 보낼까요? 이 복귀는 **닫는 동작**이며 시간 %d ms입니다. 끼임·소리·발열이면 N (손·도구가 멀 때만 y)" % (start_of_ad, cur, t_ret))):
                    ad.confirm_hands_clear(); time.sleep(QUIET_S); ad.return_to_start(t_ret); result["returned_to_start"] = True; result["return_time_ms"] = t_ret; print("복귀 명령 1회 전송")
                else: result["returned_to_start"] = False
            except BaseException as e: result["return_error"] = repr(e); print("복귀 처리 중단:", repr(e))
        result["adapter_logs"] = [[(["write", *l[1:3], "fake" if (a.fake and not a.fake_dry and l[0] == "write") else l[3], *l[4:]] if l[0] == "write" else list(l)) for l in x.log] for x in ADAPTERS]
        result["writes_total"] = sum(1 for x in ADAPTERS for l in x.log if l[0] == "write")
        result["finished_at"] = datetime.datetime.now().isoformat(timespec="seconds")
        Path(a.out).write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"); print("기록 저장:", a.out)


if __name__ == "__main__": main()
