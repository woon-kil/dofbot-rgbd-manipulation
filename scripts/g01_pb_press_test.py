#!/usr/bin/env python3
"""G07 P-B J6 연속 누름 시험(서보 6만, 팔 관절은 명령하지 않는다). 사람이 입회해 직접 실행하는 대화형 스크립트(무인 실행 금지).
승인 요청 문서: docs/gates/G07-PB-J6-연속-누름-시험-승인-요청-문서-초안.md (PB-1·PB-2·PB-3·PB-5 승인 2026-11-17, PB-4 블록 정체 미답, `--real`은 도구·가짜 시험·dry run 뒤 별도 승인)
사용: python3 g01_pb_press_test.py --fake [--fake-speed 40] [--fake-stall-at 146]   (가짜 서보, 하드웨어 없음)
      python3 g01_pb_press_test.py                              (실제 SDK 읽기, 쓰기는 dry_run: 쓰기 0회, 입력 1번)
      python3 g01_pb_press_test.py --real                       (실제 쓰기. 확인 문구를 직접 입력해야 한다)
구간: (열기, 시작이 오므림일 때만) 179→160→140→120→100 → (블록 없음 대조 1회, 10 s) → (블록 있음 사다리 10→20→30 s) → (마무리) 100→120→140→160→179 [블록 제거 뒤].
한 시도: 100→120→130→135→140→145→150으로 닫고, 블록 시도는 145 쓰기부터(블록 없음 대조는 150 쓰기부터) 총 T초 누른 뒤(150 정착 뒤 읽기만, 추가 쓰기 없음) 100으로 열기(3000 ms).
블록 있음 시도는 145 미만(140 이하) 계단에서 정체하면(조기 접촉) 거기서 중단하고 열며(145 정체는 정상 접촉으로 기록하고 150까지 계속), 150에서 정체하지 않으면(블록 미접촉) 누르지 않고 열고 중단한다.
누름 시간 코드 상한: PressBudget이 위 시작점부터 센다. 백스톱 = T + 1.0 s.
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
EARLY_BELOW = 145                                # 사용자 승인 2026-11-17(권장안 A): 조기 접촉 중단은 명령 145 미만(140 이하) 정체에만 적용한다. 145 정체(P2h 3/3: 145→142)는 정상 접촉으로 기록하고 150까지 계속한다
NONE_CONSEC_MAX = 3                              # 사용자 승인 2026-11-17(권장안 A): 누름 중 읽기 None이 이 횟수만큼 연속이면(100 ms 읽기에서 약 0.3 s) 중단하고 연다. 단발 None은 건너뛰고 개수만 기록한다
CRUSH_RISE = 2                                   # PB-1: 누름 중 마지막 5회 읽기가 단조 증가하고 정체값보다 이만큼 오르면 눌림(문서 '2° 넘게'를 보수적으로 ≥2로 구현)
E3_MAX = 1                                       # PB-1: 누름 중 회독 최대−최소가 이를 넘으면 기록하고 사다리를 멈춘다(즉시 열기는 아님)
RETURN_MS = 3000                                 # 150→100 = 50° × 60 ms/° = 3000 ms
SETTLE_S = 2.0                                   # 150 쓰기 뒤 정착 읽기 시간(이동 1.5 s + 0.5 s). 총 누름(150 쓰기부터)이 T가 되도록 이 뒤 읽기 시간은 T − SETTLE_S
BUDGET_EXTRA_S = 1.0                             # PressBudget 백스톱 = T + 1.0 s(정상 시도는 T 안에 끝난다. 읽기 지연이 큰 이상 상황에서만 작동)
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
    """P-B 가짜 서보: 누름 읽기 구간(hold_on)에서만 이상을 모사한다. creep이면 회독이 천천히 오르고(상한 149), wobble이면 번갈아 wobble°씩 뛰며, none_after면 N번 읽은 뒤 None을 돌려준다."""
    def __init__(self, v, speed): super().__init__(v, speed); self.wobble = 0; self.none_after = None; self.creep_hold = False; self.hold_on = False; self._n = 0; self.drop_open = 0; self.drop_close = 0; self.none_every = 0
    def Arm_serial_servo_write(self, id, angle, time_ms):
        if self.drop_open > 0 and angle == 100 and time_ms == 3000: self.drop_open -= 1; return      # 열기 쓰기 폐기 모사(읽기 직후 쓰기 폐기 실측 이력)
        if self.drop_close > 0 and angle == 179 and time_ms == 3000: self.drop_close -= 1; return      # 닫는 쪽 복귀 쓰기 폐기 모사
        super().Arm_serial_servo_write(id, angle, time_ms)
    def Arm_serial_servo_read(self, id):
        v = super().Arm_serial_servo_read(id)
        if self.hold_on and self._held and v is not None:
            self._n += 1
            if self.none_after is not None and self._n > self.none_after: return None
            if self.none_every and self._n % self.none_every == 0: return None
            if self.creep_hold: v = min(149, v + self._n // 2)
            if self.wobble and self._n % 2: v += self.wobble
        return v


class OpenUnconfirmed(Exception):
    pass


class ClosingUnconfirmed(Exception):
    pass


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--fake", action="store_true"); ap.add_argument("--real", action="store_true")
    ap.add_argument("--fake-dry", action="store_true"); ap.add_argument("--fake-speed", type=int, default=1); ap.add_argument("--fake-start", type=int, default=179)
    ap.add_argument("--fake-stall-at", type=int, default=None); ap.add_argument("--fake-creep", action="store_true"); ap.add_argument("--fake-wobble", type=int, default=0)
    ap.add_argument("--fake-none-after", type=int, default=None); ap.add_argument("--fake-no-hold-end", action="store_true", help="--fake에서만: 누름 읽기 종료 조건을 끈다(PressBudget 상한 시험)")
    ap.add_argument("--fake-drop-open", type=int, default=0, help="--fake에서만: 처음 N번의 열기(100, 3000 ms) 쓰기를 버린다")
    ap.add_argument("--fake-drop-close", type=int, default=0, help="--fake에서만: 처음 N번의 닫는 쪽 복귀(179, 3000 ms) 쓰기를 버린다")
    ap.add_argument("--fake-none-every", type=int, default=0, help="--fake에서만: 누름 읽기 N번마다 1번 None(단발 None 시험)")
    ap.add_argument("--no-control", action="store_true", help="블록 없음 대조 1회를 생략한다(PB-2는 넣는 것이 기본)")
    ap.add_argument("--run-id", default="local"); ap.add_argument("--out", default="/tmp/g01_pb_result.json"); a = ap.parse_args()
    if a.fake and a.real: sys.exit("--fake와 --real은 함께 쓸 수 없다")
    if (a.fake_dry or a.fake_speed != 1 or a.fake_stall_at or a.fake_creep or a.fake_wobble or a.fake_none_after is not None or a.fake_no_hold_end or a.fake_none_every or a.fake_drop_open or a.fake_drop_close or a.fake_start != 179) and not a.fake: sys.exit("--fake-* 옵션은 --fake에서만 쓸 수 있다")
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
            adapter.confirm_hands_clear(); time.sleep(QUIET_S / sp); adapter.move(tgt, t_ms); writes_done = writes_done or not dry; cur = tgt
            if dry: steps.append({"n": i, "angle": tgt, "time_ms": t_ms, "dry": True}); continue
            stop = lambda sr, el, _t=tgt, _m=t_ms: el >= _m / sp and len(tail5(sr)) == 5 and max(tail5(sr)) - min(tail5(sr)) <= 1 and abs(_t - tail5(sr)[-1]) <= 3
            series = poll(adapter, int(2 * t_ms + 1000) // sp, POLL_MS / sp, stop=stop)
            last = next((v for _, v in reversed(series) if v is not None), None); steps.append({"n": i, "angle": tgt, "time_ms": t_ms, "final_readback": last, "series": series})
            print("  [%s] 이동 %d/%d: %d (%d ms) → 회독 %s" % (label, i, len(targets) - 1, tgt, t_ms, last))
            if last is None or abs(last - tgt) > 3: raise Abort("도착 실패(회독 %s, 목표 %d, 허용 3°). 추가 이동 없음." % (last, tgt))
        return steps

    def settle_to(adapter, s, t_ms):
        stop = lambda sr, el, _s=s, _m=t_ms: el >= _m / sp and len(tail5(sr)) == 5 and max(tail5(sr)) - min(tail5(sr)) <= 1 and abs(_s - tail5(sr)[-1]) <= 3
        series = poll(adapter, int(max(2 * t_ms + 1000, 2.7 * t_ms + 600)) // sp, POLL_MS / sp, stop=stop); return next((v for _, v in reversed(series) if v is not None), None)

    def verify_return(adapter, s, t_ms):
        """복귀(열기) 쓰기 뒤 도착 확인. 확인 실패(쓰기 폐기 등)면 새 어댑터로 같은 쓰기를 1회 더 보내고, 그래도 안 되면 OpenUnconfirmed. repeat_last는 직전 '이동'(=150)을 다시 보내므로 쓰지 않는다. 정상 경로(go_home)와 중단 경로(finally)가 함께 쓴다."""
        nonlocal cur, ad
        last = settle_to(adapter, s, t_ms)
        if last is not None and abs(last - s) <= 3: cur = s; return last
        if s > 101: raise ClosingUnconfirmed("닫는 쪽 복귀가 확인되지 않았다(회독 %s, 목표 %d). 닫는 쓰기는 손 제거 답 없이 다시 보내지 않는다. 추가 명령 없음" % (last, s))
        print("★ 복귀가 확인되지 않았다(회독 %s, 목표 %d). 새 어댑터로 같은 쓰기를 1회 다시 보낸다." % (last, s)); result["open_retry"] = result.get("open_retry", 0) + 1
        try:
            rad = make_adapter(arm, WIN_OPEN, 2, dry); s2 = rad.read_start(); rad.set_targets([s2, s]); rad.confirm_hands_clear(); time.sleep(QUIET_S / sp); rad.move(s, t_ms); last2 = settle_to(rad, s, t_ms)
        except GripperTestError as e: raise OpenUnconfirmed("복귀 재전송 준비 실패: %s" % e)
        if last2 is not None and abs(last2 - s) <= 3: cur = s; ad = rad; return last2
        raise OpenUnconfirmed("복귀 쓰기를 두 번 보냈지만 회독 %s(목표 %d)로 도착하지 않았다. 열기 미확인" % (last2, s))

    def go_home(adapter, s):
        adapter.confirm_hands_clear(); time.sleep(QUIET_S / sp); adapter.return_to_start(RETURN_MS)
        return verify_return(adapter, s, RETURN_MS)

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
            ad.confirm_hands_clear(); time.sleep(QUIET_S / sp); ad.move(tgt, 1500); writes_done = True; cur = tgt
            last_step = (tgt == 150)
            if tgt == (EARLY_BELOW if block else 150): budget.start()   # 총 누름 시간은 블록 시도에서 145 쓰기부터 센다(P2h는 145에서 이미 정체하므로 승인된 30 s를 넘지 않게 보수적으로). 블록 없음 대조는 150부터
            if last_step: stop = lambda sr, el, _h=int(1500 + HOLD_EXTRA_MS) / sp: el >= _h
            else: stop = lambda sr, el, _t=tgt: el >= 1500 / sp and len(tail5(sr)) == 5 and max(tail5(sr)) - min(tail5(sr)) <= 1 and (abs(_t - tail5(sr)[-1]) <= 3 or (block and _t - tail5(sr)[-1] >= 3))
            series = poll(ad, int(max(2 * 1500 + 1000, 0)) // sp, POLL_MS / sp, stop=stop)
            last = next((v for _, v in reversed(series) if v is not None), None); t5 = tail5(series)
            st = {"n": i, "angle": tgt, "final_readback": last, "tail5": t5, "series": series}; tr["steps"].append(st)
            print("  [%s] 이동 %d/%d: %d → 회독 %s" % (label, i, len(CLOSE_STEPS), tgt, last))
            if last is None: st["arrival_failed"] = True; tr["abort"] = "회독 None"; break
            stalled = len(t5) == 5 and max(t5) - min(t5) <= 1 and tgt - last >= 3
            if stalled and tr["stall_step"] is None: tr["stall_step"] = tgt
            if block and stalled and tgt < EARLY_BELOW: st["early_contact"] = True; tr["abort"] = "조기 접촉: 명령 %d에서 회독 %s(잔차 %d°)로 정체(P2h는 140에서 138·139, 145에서 142로 정체). 홀더 위치가 P2h와 다를 수 있다" % (tgt, last, tgt - last); break
            if not block and (abs(tgt - last) > 3):
                st["free_contact_suspect"] = True; tr["abort"] = "블록 없음 구간인데 명령 %d에서 회독 %s(잔차 %d°). 손가락 사이에 무언가 있거나 서보 이상이 의심된다" % (tgt, last, tgt - last); break
            if block and not last_step and not stalled and abs(tgt - last) > 3: st["arrival_failed"] = True; tr["abort"] = "도착 실패(회독 %s, 목표 %d)" % (last, tgt); break
            if last_step:
                ref = min(t5) if t5 else last      # 정체값 기준: 마지막 5회의 최솟값(146↔147 흔들림이면 146, 눌림 감지를 늦추지 않는다)
                if tgt - last >= 3 and tr["stall_step"] is None: tr["stall_step"] = tgt   # 150에서는 읽기 흔들림(폭 > 1)이 있어도 잔차 3° 이상이면 블록에 물린 것으로 본다(흔들림은 E3가 판정)
                if block and tgt - last < 3: tr["abort"] = "블록 미접촉: 명령 150에서 회독 %s(잔차 %d°)로 정체하지 않았다. 블록이 빠졌거나 홀더가 밀렸을 수 있어 누르지 않는다" % (last, tgt - last)
        if tr["abort"] is None:
            # 150 도달 뒤 T초 읽기만(추가 쓰기 없음). 블록 없음 대조는 눌림 규칙을 적용하지 않고 기록만 한다
            if a.fake: arm.hold_on = True
            hold = []; consec = 0; n_none = 0; max_consec = 0; t0 = time.monotonic(); tr["hold"] = {"ref": ref, "series": hold}
            while True:
                el = time.monotonic() - t0
                v = ad.read_position(); hold.append((round(el * 1000), v))
                if v is None: consec += 1; n_none += 1; max_consec = max(max_consec, consec)
                else: consec = 0
                vals = [x for _, x in hold if x is not None]
                if consec >= NONE_CONSEC_MAX: status, why = "read_none", "누름 중 회독이 연속 %d회 None이다(읽기 실패)" % consec
                elif block and vals: status, why = hold_check(ref, vals)
                else: status, why = "ok", None
                if status != "ok": tr["abort"] = why; tr["crush"] = why if status == "crush" else None; break
                if not a.fake_no_hold_end and budget.used_s() >= (T - QUIET_S) / sp: break   # 총 누름(145 쓰기부터, 블록 없음은 150부터)이 T − 열기 전 중지 0.3 s에 이르면 읽기를 끝낸다
                try: budget.check()
                except BudgetExceeded as e: tr["abort"] = str(e); break
                time.sleep(POLL_MS / 1000.0 / sp)
            tr["n_none"] = n_none; tr["max_consecutive_none"] = max_consec
            if a.fake: arm.hold_on = False
            e3 = e3_range([v for _, v in hold]); tr["e3"] = e3; tr["e3_exceeded"] = bool(e3 is not None and e3 > E3_MAX)
            tr["hold"]["used_s_scaled"] = round(budget.used_s() * sp, 2); tr["hold"]["n_reads"] = len(hold)
            print("  누름 읽기 %.1f s(150 쓰기 뒤 정착 %.1f s 별도, 표시는 실제 시간 환산) 읽기 %d회, 회독 최대−최소(E3) %s%s" % (el * sp, SETTLE_S, len(hold), e3, "  ★ E3 초과" if tr["e3_exceeded"] else ""))
        tr["return_final"] = go_home(ad, s); writes_done = True; budget.stop()
        if tr["abort"]: print("★ %s. 열기를 마쳤다." % tr["abort"]); raise Abort("시도 %s: %s" % (label, tr["abort"]))
        return tr

    try:
        print("G07 P-B J6 연속 누름 시험(%s): 그리퍼 서보 6만 움직인다. 팔 관절은 명령하지 않는다." % mode)
        print("사전 확인:")
        for k, q in enumerate(("전원 차단 담당자가 현재 자리에 있다", "팔이 K1 기립(대기) 자세이고 낙하 받침이 준비돼 있다(전원 차단 시 직립 팔이 낙하할 수 있음)", "블록 없음 대조 구간 동안 손가락 사이에 아무것도 없다", "블록 홀더(스탠드·집게)와 **여분(다른 색) 블록**이 준비돼 있다(블록은 대조가 끝난 뒤에 둔다. **등록 블록(마커 id 7)은 종이 위에 그대로 둔다**, PB-4)", "누름이 최대 약 30 s까지 간다(서보 발열 미측정). 눌림·소리·냄새·떨림이 보이면 즉시 Ctrl-C, 시도 사이 휴식 %d s를 지킨다" % REST_S), 1): print("  %d. %s" % (k, q))
        if not ask("위 5개를 모두 확인했습니까? (질문은 이 한 번으로 묶었습니다)"): raise Abort("사전 확인 미충족. 명령은 보내지 않았다.")
        if a.real and input("실제 쓰기를 시작하려면 정확히 '%s'를 입력: " % PHRASE).strip() != PHRASE: raise Abort("승인 문구 불일치. 명령은 보내지 않았다.")
        if a.fake:
            arm = PBFake(a.fake_start, sp); arm.creep_hold = a.fake_creep; arm.wobble = a.fake_wobble; arm.none_after = a.fake_none_after; arm.none_every = a.fake_none_every; arm.drop_open = a.fake_drop_open; arm.drop_close = a.fake_drop_close
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
                if not ask("시도 %s 직후: J6 서보가 이상 없이 정상이었습니까? (뜨겁지 않고 소음·냄새·떨림이 없었으면 y, 하나라도 있었으면 N으로 사다리를 멈춘다)" % label): result["ladder_stop"] = "사용자 이상 보고(%s)" % label; print("★", result["ladder_stop"]); break
            if a.fake: arm.stall_at = None
            if not ask("블록과 홀더를 손가락 사이에서 치웠습니까? (마무리 닫기를 시작합니다)"): raise Abort("블록 제거 미확인. 추가 명령 없음.")
            ad = make_adapter(arm, WIN_FINISH, 6, dry); s = ad.read_start(); cur = s; start_of_ad = s
            fin_t = [s] + FINISH_STEPS; steps = move_steps("마무리", ad, fin_t, [1500] * len(FINISH_STEPS)); result["phases"].append({"phase": "finish", "targets": fin_t, "steps": steps})
            result["ended"] = "completed"
    except OpenUnconfirmed as e:
        result["ended"] = "open_unconfirmed"; result["open_unconfirmed"] = True; result["abort_reason"] = str(e); print("\n★★ 열기 미확인:", e); print("추가 명령을 보내지 않는다. 손을 넣지 마세요. 그리퍼가 블록을 계속 누르고 있을 수 있으므로 전원 차단 담당자가 판단하세요.")
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
        if ad is not None and writes_done and result["ended"] not in ("completed", "open_unconfirmed") and start_of_ad is not None:
            try:
                dist = abs((cur if cur is not None else start_of_ad) - start_of_ad); t_ret = return_time(dist)
                opening = start_of_ad <= 101
                if dist == 0 or getattr(ad, "_returned", False): result["returned_to_start"] = None
                elif t_ret is None: print("복귀 폭 %d°가 승인된 시간으로 갈 수 없다. 복귀하지 않는다." % dist); result["returned_to_start"] = False
                elif ask(("그리퍼가 블록을 누르고 있을 수 있습니다(마지막 명령 %s). 열린 쪽 S=%d로 복귀 쓰기를 1회 보낼까요? 이 복귀는 블록을 놓는 **여는 동작**이며 시간 %d ms입니다. 이상(소리·발열·도착 실패·눌림)이 있으면 y를 권장합니다" % (cur, start_of_ad, t_ret)) if opening else ("그리퍼가 이 구간의 시작값 S=%d가 아닐 수 있습니다(마지막 명령 %s). S로 복귀 쓰기를 1회 보낼까요? 이 복귀는 **닫는 동작**이며 시간 %d ms입니다. 끼임·소리·발열이면 N (손·도구가 멀 때만 y)" % (start_of_ad, cur, t_ret))):
                    ad.confirm_hands_clear(); time.sleep(QUIET_S / sp); ad.return_to_start(t_ret); result["return_sent"] = True; result["return_time_ms"] = t_ret; print("복귀 명령 1회 전송. 도착을 확인합니다.")
                    result["return_final"] = verify_return(ad, start_of_ad, t_ret); result["returned_to_start"] = True; print("복귀 확인: 회독 %s" % result["return_final"])
                else: result["returned_to_start"] = False
            except OpenUnconfirmed as e:
                result["open_unconfirmed"] = True; result["return_error"] = str(e); print("\n★★ 복귀 미확인:", e); print("추가 명령을 보내지 않는다. 손을 넣지 마세요. 그리퍼가 블록을 계속 누르고 있을 수 있으므로 전원 차단 담당자가 판단하세요.")
            except ClosingUnconfirmed as e:
                result["closing_unconfirmed"] = True; result["return_error"] = str(e); print("\n★ 닫는 쪽 복귀 미확인:", e)
            except BaseException as e:
                result["return_error"] = repr(e); print("복귀 처리 중단:", repr(e))
                if result.get("return_sent") and not result.get("returned_to_start"): result["return_unconfirmed"] = True; print("★★ 복귀 쓰기를 보냈지만 확인 전에 끊겼다. 복귀했다고 믿지 마세요. 손을 넣지 마세요. 전원 차단 담당자가 판단하세요.")
        result["adapter_logs"] = [[(["write", *l[1:3], "fake" if (a.fake and not a.fake_dry and l[0] == "write") else l[3], *l[4:]] if l[0] == "write" else list(l)) for l in x.log] for x in ADAPTERS]
        result["writes_total"] = sum(1 for x in ADAPTERS for l in x.log if l[0] == "write")
        result["finished_at"] = datetime.datetime.now().isoformat(timespec="seconds")
        Path(a.out).write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"); print("기록 저장:", a.out)


if __name__ == "__main__": main()
