#!/usr/bin/env python3
"""G07 P2i 큰 명령(150·155)에서 블록 있음/없음 회독 비교 시험(서보 6만). 사람이 입회해 직접 실행하는 대화형 스크립트(무인 실행 금지).
승인 요청 문서: docs/gates/G07-P2i-큰-명령-블록-유무-비교-승인-요청-문서.md (I1~I5 승인, R9 `--real` 승인은 도구·가짜 시험·dry run 뒤 별도)
사용: python3 g01_p2i_gripper_test.py --fake [--fake-speed 20] [--fake-stall-at 142]   (가짜 서보, 하드웨어 없음)
      python3 g01_p2i_gripper_test.py                              (실제 SDK 읽기, 쓰기는 dry_run: 쓰기 0회, 입력 1번)
      python3 g01_p2i_gripper_test.py --real                       (실제 쓰기. 확인 문구를 직접 입력해야 한다)
구간: (열기) 177~179→160→140→120→100 [쓰기 4] → (블록 없음 2회) 100→120→140→150→155, 100으로 복귀 → (블록 있음 3회, 홀더 a안) 같은 계단, 정체가 나와도 150·155까지 계속,
눌림 감지 시 즉시 복귀, 100으로 복귀 → (마무리) 100→120→140→160→179 [쓰기 4, 블록 제거]. 누르는 구간 각도 창 [95, 155]. 한 단계 총 유지 ≤ 2.0 s.
"""
import argparse, datetime, json, signal, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gripper_adapter import GripperTestError
from g01_5a_gripper_test import Abort, Hangup, _on_signal, ask, POLL_MS
import g01_p2h_gripper_test as H
from g01_p2h_gripper_test import poll, make_adapter, FastFakeStall, START_CLOSED, START_OPEN, ADAPTERS
from g01_p2g_gripper_test import return_time, QUIET_S

OPEN_STEPS = [160, 140, 120, 100]
PRESS_STEPS = [120, 140, 150, 155]               # 누르는 계단(시작 S=100). 150·155가 측정 대상
FINISH_STEPS = [120, 140, 160, 179]
WIN_OPEN, WIN_PRESS, WIN_FINISH = (95, 179), (95, 155), (95, 179)   # 누르는 구간 창 상한 155(I2 승인, 새 안전 한도)
HOLD_EXTRA_MS = 500                              # 이동이 끝난 뒤 이 시간만 읽고 곧바로 다음 단계·복귀(한 단계 총 유지 ≈ 이동 1.5 s + 0.5 s = 2.0 s)
RETURN_MS = 4800                                 # 155→100 = 55° → 60 ms/° × 55 = 3300 ms 이상 필요(승인된 4800 ms, 복귀 시작 시점부터 누르기를 멈춤)
FREE_TRIALS, BLOCK_TRIALS = 2, 3
REST_FREE_S, REST_BLOCK_S = 30, 60
CRUSH_RISE = 2                                   # 같은 단계의 마지막 5회 읽기가 단조 증가하며 이만큼 오르면 눌림(제안)
CRUSH_OVER_150 = 4                               # 155 회독이 150 정체 회독보다 이만큼 넘게 크면 눌림·미끄러짐(제안)
SUSPECT_X = 2.7                                  # 블록 없음 구간 접촉 의심 판정 시각 = 이동 시간 × 2.7 (1500 ms → 4.05 s; 실측 느린 이동 3.2·3.9 s를 넘김)
STALL_ZONE_MIN = 141                             # 상승을 눌림으로 세려면 5회 읽기의 최솟값이 이미 P2h 정체 영역(141 이상)에 있어야 한다(아직 다가가는 중인 상승을 눌림으로 오판하지 않기 위함, 검토 반영)
PHRASE = "APPROVE P2I REAL"
SEP_MIN = 4                                      # 판정 문턱: 명령 150에서 (블록 없음 평균 − 블록 있음 평균) ≥ 4°이면 회독으로 파지 판정 가능(I4)
BLOCK_RANGE_MAX = 2; FREE150_LO, FREE150_HI = 147, 150


def tail5(series):
    return [v for _, v in series if v is not None][-5:]


def crush_check(series, ref150=None, cmd=None):
    """눌림 감지(I3): 마지막 5회 읽기가 단조 증가하며 CRUSH_RISE 이상 오르거나, 155에서 회독이 150 정체 회독보다 CRUSH_OVER_150 넘게 크면 True."""
    t = tail5(series)
    if len(t) == 5 and min(t) >= STALL_ZONE_MIN and all(b >= a for a, b in zip(t[:-1], t[1:])) and t[-1] - t[0] >= CRUSH_RISE: return True, "정체 영역(%d 이상)에서 회독이 같은 명령에서 계속 오름 %s" % (STALL_ZONE_MIN, t)
    if cmd == 155 and ref150 is not None and t and max(t) > ref150 + CRUSH_OVER_150: return True, "155 회독 %d가 150 회독 %d보다 %d° 넘게 큼" % (max(t), ref150, CRUSH_OVER_150)
    return False, None


def decide(free, block):
    """I4 판정. free·block은 {150: [회독...], 155: [...]}. 명령 150에서 (블록 없음 평균 − 블록 있음 평균) ≥ SEP_MIN이면 가능."""
    out = {}
    for c in (150, 155):
        f, b = free.get(c) or [], block.get(c) or []
        if f and b: out[c] = {"free_mean": round(sum(f) / len(f), 2), "block_mean": round(sum(b) / len(b), 2), "diff": round(sum(f) / len(f) - sum(b) / len(b), 2), "free_range": max(f) - min(f), "block_range": max(b) - min(b)}
    r150 = out.get(150)
    out["a_block150_range_ok"] = bool(r150 and r150["block_range"] <= BLOCK_RANGE_MAX)
    out["b_free150_in_147_150"] = bool(free.get(150) and all(FREE150_LO <= x <= FREE150_HI for x in free[150]))
    out["c_separation_ok"] = bool(r150 and r150["diff"] >= SEP_MIN)
    out["n_free150"] = len(free.get(150) or []); out["n_block150"] = len(block.get(150) or [])
    out["n_ok"] = out["n_free150"] >= FREE_TRIALS and out["n_block150"] >= BLOCK_TRIALS   # 판정은 블록 없음 2회·블록 있음 3회가 모두 안정된 값일 때만
    out["verdict"] = ("회독으로 파지 판정 가능(명령 150에서 차이 %.1f° ≥ %d°)" % (r150["diff"], SEP_MIN)) if (r150 and out["n_ok"] and out["a_block150_range_ok"] and out["b_free150_in_147_150"] and out["c_separation_ok"]) else "회독으로 파지 판정 가능하지 않음 또는 판정 불가(영상 등 보조 수단 필요, 위 항목 확인)"
    return out


class FastFakeBlock(FastFakeStall):
    """가짜 서보: stall_at이 있으면 그 각도를 넘어 닫히지 못한다. squeeze k면 명령이 stall_at을 넘은 만큼의 k배만큼 더 닫힌다(눌림 모사). creep이면 읽을 때마다 조금씩 오른다(눌림 감지 시험)."""
    def __init__(self, v, speed): super().__init__(v, speed); self.squeeze = 0.0; self.creep = False; self._reads = 0; self._held = False; self.slow = 1.0
    def Arm_serial_servo_write(self, id, angle, time_ms):
        time_ms = int(time_ms * self.slow)   # 실측: 1500 ms 명령이 3.2~3.9 s 걸린 이동이 있었다(P2g 이동 8, P2h 마무리 160)
        self._held = False
        if self.stall_at is not None and angle > self.stall_at:
            self._held = True; angle = int(round(self.stall_at + self.squeeze * (angle - self.stall_at)))
            sa, self.stall_at = self.stall_at, None
            try: super().Arm_serial_servo_write(id, angle, time_ms)
            finally: self.stall_at = sa
            return
        super().Arm_serial_servo_write(id, angle, time_ms)
    def Arm_serial_servo_read(self, id):
        v = super().Arm_serial_servo_read(id)
        if self.creep and self._held and v is not None: self._reads += 1; return v + self._reads // 2
        return v


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--fake", action="store_true"); ap.add_argument("--real", action="store_true")
    ap.add_argument("--fake-dry", action="store_true"); ap.add_argument("--fake-speed", type=int, default=1)
    ap.add_argument("--fake-start", type=int, default=179); ap.add_argument("--fake-stall-at", type=int, default=None, help="--fake에서만: 블록 있음 구간에서 이 각도를 넘어 닫히지 못한다")
    ap.add_argument("--fake-squeeze", type=float, default=0.0); ap.add_argument("--fake-block-early", type=int, default=None, help="--fake에서만: 블록 없음 구간부터 이 각도에서 막힌다(블록을 일찍 놓은 실수 모사)"); ap.add_argument("--fake-slow", type=float, default=1.0, help="--fake에서만: 이동이 명령 시간의 이 배수가 걸리는 느린 서보"); ap.add_argument("--fake-creep", action="store_true")
    ap.add_argument("--run-id", default="local"); ap.add_argument("--out", default="/tmp/g01_p2i_result.json"); a = ap.parse_args()
    if a.fake and a.real: sys.exit("--fake와 --real은 함께 쓸 수 없다")
    if (a.fake_dry or a.fake_speed != 1 or a.fake_stall_at or a.fake_squeeze or a.fake_creep or a.fake_slow != 1.0 or a.fake_block_early) and not a.fake: sys.exit("--fake-* 옵션은 --fake에서만 쓸 수 있다")
    mode = "dry_run" if (a.fake and a.fake_dry) else ("fake" if a.fake else ("real" if a.real else "dry_run"))
    dry = (mode == "dry_run"); sp = a.fake_speed if a.fake else 1
    result = {"run_id": a.run_id, "mode": mode, "started_at": datetime.datetime.now().isoformat(timespec="seconds"), "windows": {"open": list(WIN_OPEN), "press": list(WIN_PRESS), "finish": list(WIN_FINISH)}, "allowed_times_ms": H.ALLOWED_MS, "phases": [], "free_trials": [], "block_trials": [], "interrupted": False, "ended": None}
    ad = None; cur = None; start_of_ad = None; writes_done = False
    for sg in (signal.SIGHUP, signal.SIGTERM): signal.signal(sg, _on_signal)

    def run_steps(label, adapter, targets, times, press=False, block=False, ref150=None):
        """승인된 목록을 순서대로 이동하고 단계마다 짧게 읽는다. press=True면 150 이상 단계의 회독을 기록하고 눌림을 감지한다. 반환: (steps, 눌림 사유 또는 None)."""
        nonlocal cur, writes_done
        adapter.set_targets(targets); steps = []; crush = None
        result.setdefault("partial_steps", []).append({"label": label, "steps": steps})   # 중단돼도 이 구간에서 읽은 단계는 결과 JSON에 남긴다(P2i 1차 실행에서 블록 없음 2 단계 기록이 사라졌다)
        for i, (tgt, t_ms) in enumerate(zip(targets[1:], times), 1):
            adapter.confirm_hands_clear(); time.sleep(QUIET_S); adapter.move(tgt, t_ms); writes_done = writes_done or not dry; cur = tgt
            if dry: steps.append({"n": i, "angle": tgt, "time_ms": t_ms, "dry": True}); continue
            pressing = press and block and tgt >= 150       # I3의 한 단계 총 유지 2.0 s 제한은 블록을 누르는 단계에만 적용한다(검토 반영: 느린 이동 때문에 다른 단계가 도착 전에 끊기는 것을 막는다)
            hold = int(t_ms + HOLD_EXTRA_MS) / sp
            free_zone = press and not block               # 블록 없음 시도: 접촉이 의심되면(블록을 일찍 둔 경우 등) 질문 없이 바로 열린 쪽으로 복귀한다(검토 반영: 1차 실행은 146에서 정체한 채 전체 창을 읽고 복귀 질문을 기다렸다)
            early_zone = press and block and tgt < 150     # 블록 시도의 150 미만 단계: 홀더 위치가 P2h와 달라 조기 접촉(정체)이 날 수 있다. 정착하면 잔차와 무관하게 읽기를 끝내 누르는 시간을 줄인다(검토 반영)
            if pressing: stop = lambda sr, el, _h=hold: el >= _h
            elif free_zone: stop = lambda sr, el, _m=t_ms, _t=tgt: len(tail5(sr)) == 5 and max(tail5(sr)) - min(tail5(sr)) <= 1 and ((el >= _m / sp and abs(_t - tail5(sr)[-1]) <= 3) or el >= SUSPECT_X * _m / sp)   # 블록 없음: 정상 정착이면 일찍, 접촉 의심(잔차 > 3°)은 이동 시간의 2.7배(약 4 s, 실측 최장 3.9 s)가 지나 느린 이동·멈칫이 아님을 확인한 뒤에만 판정(누르는 위험이 없는 구간)
            elif early_zone: stop = lambda sr, el, _m=t_ms: el >= 1.5 * _m / sp and len(tail5(sr)) == 5 and max(tail5(sr)) - min(tail5(sr)) <= 1   # 블록 있음 150 미만: 접촉 의심 판정을 이동 시간의 1.5배(약 2.25 s) 뒤로, 느린 이동의 오판 방지(누르는 시간은 이 한 단계에 한정)
            else: stop = lambda sr, el, _t=tgt, _m=t_ms: el >= _m / sp and len(tail5(sr)) == 5 and max(tail5(sr)) - min(tail5(sr)) <= 1 and abs(_t - tail5(sr)[-1]) <= 3   # 정착(5회 읽기 폭 ≤ 1°, 잔차 ≤ 3°)하면 일찍 끝냄
            series = poll(adapter, int(max(2 * t_ms + 1000, SUSPECT_X * t_ms + 600)) // sp, POLL_MS / sp, stop=stop)
            last = next((v for _, v in reversed(series) if v is not None), None); t5 = tail5(series)
            step = {"n": i, "angle": tgt, "time_ms": t_ms, "final_readback": last, "tail5": t5, "series": series}; steps.append(step)
            print("  [%s] 이동 %d/%d: %d (%d ms) → 회독 %s" % (label, i, len(targets) - 1, tgt, t_ms, last))
            if press and tgt >= 150 and block:
                c, why = crush_check(series, ref150 if tgt == 155 else None, tgt)
                if c: step["crush"] = why; crush = why; print("  ★ 눌림 감지: %s. 즉시 복귀합니다." % why); return steps, crush
                if len(t5) < 5 or max(t5) - min(t5) > 1: step["unsettled"] = True; print("  ! 경고: 이 단계의 마지막 5회 읽기가 안정되지 않았다(이동이 느렸을 수 있음). 이 값은 판정에서 제외한다.")
                if tgt == 150 and not step.get("unsettled"): ref150 = last
            elif free_zone and last is not None and len(t5) == 5 and max(t5) - min(t5) <= 1 and tgt - last >= 3:
                why = "블록 없음 구간 접촉 의심: 명령 %d에서 회독 %s(잔차 %d°)로 정체. 손가락 사이에 블록·물체가 있거나 서보 이상이 의심된다" % (tgt, last, tgt - last)
                step["free_contact_suspect"] = why; print("  ★ %s. 질문 없이 바로 열린 쪽으로 복귀합니다." % why); return steps, why
            elif early_zone and last is not None and len(t5) == 5 and max(t5) - min(t5) <= 1 and tgt - last >= 3:
                why = "조기 접촉: 명령 %d에서 회독 %s(잔차 %d°)로 정체(P2h는 140에서 138·139). 홀더 위치가 P2h와 다를 수 있다" % (tgt, last, tgt - last)
                step["early_contact"] = why; print("  ★ %s. 질문 없이 바로 열린 쪽으로 복귀합니다." % why); return steps, why
            elif last is None or abs(last - tgt) > 3:
                if press and tgt >= 150 and not block: step["free_contact_suspect"] = True; raise Abort("블록 없음 구간인데 명령 %d에서 회독 %s(잔차 %s). 손가락 사이에 무언가 있거나 서보 이상이 의심된다. 추가 이동 없음." % (tgt, last, None if last is None else tgt - last))
                step["arrival_failed"] = True; raise Abort("도착 실패(회독 %s, 목표 %d, 허용 3°). 추가 이동 없음." % (last, tgt))
        return steps, crush

    def go_home(adapter, s):
        nonlocal cur
        adapter.confirm_hands_clear(); time.sleep(QUIET_S); adapter.return_to_start(RETURN_MS); cur = s
        stop = lambda sr, el, _s=s: el >= RETURN_MS / sp and len(tail5(sr)) == 5 and max(tail5(sr)) - min(tail5(sr)) <= 1 and abs(_s - tail5(sr)[-1]) <= 3   # 복귀가 느려도(실측 2배) 정착할 때까지 기다린다(검토 반영)
        series = poll(adapter, (2 * RETURN_MS + 1000) // sp, POLL_MS / sp, stop=stop); return next((v for _, v in reversed(series) if v is not None), None)

    try:
        print("G07 P2i 큰 명령 블록 있음/없음 회독 비교(%s): 그리퍼 서보 6만 움직인다. 팔 관절은 명령하지 않는다." % mode)
        print("사전 확인:")
        for k, q in enumerate(("전원 차단 담당자가 현재 자리에 있다", "팔이 K1 기립(대기) 자세이고 낙하 받침이 준비돼 있다(전원 차단 시 직립 팔이 낙하할 수 있음)", "열기·블록 없음 구간 동안 손가락 사이에 아무것도 없다", "블록 홀더(스탠드·집게)와 블록이 준비돼 있다(블록은 블록 없음 구간이 끝난 뒤에 둔다)", "이 시험은 명령 150·155까지 누른다(서보 발열·블록 눌림 미측정, 눌림이 보이면 즉시 Ctrl-C, 시도 사이 휴식을 지킨다)"), 1): print("  %d. %s" % (k, q))
        if not ask("위 5개를 모두 확인했습니까? (질문은 이 한 번으로 묶었습니다)"): raise Abort("사전 확인 미충족. 명령은 보내지 않았다.")
        if a.real and input("실제 쓰기를 시작하려면 정확히 '%s'를 입력: " % PHRASE).strip() != PHRASE: raise Abort("승인 문구 불일치. 명령은 보내지 않았다.")
        if a.fake:
            arm = FastFakeBlock(a.fake_start, sp); arm.squeeze = a.fake_squeeze; arm.creep = a.fake_creep; arm.slow = a.fake_slow
        else:
            from Arm_Lib import Arm_Device; arm = Arm_Device()   # 생성자는 버스를 열기만 한다(소스 확인)
        ad = make_adapter(arm, WIN_OPEN, 6, dry, first_min=3000)
        s0 = ad.read_start(); cur = s0; start_of_ad = s0
        skip_open = s0 in START_OPEN                      # 이미 열려 있으면(예: 이전 시험이 중단돼 100에 남음) 열기 구간을 생략한다
        if s0 not in START_CLOSED and not skip_open: raise Abort("시작 회독 S=%d가 오므림 범위 %s도 열림 범위 %s도 아니다. 명령은 보내지 않았다." % (s0, list(START_CLOSED), list(START_OPEN)))
        open_t = [s0] + OPEN_STEPS; open_ms = [3000] + [1500] * (len(OPEN_STEPS) - 1)
        if skip_open: print("시작이 열린 상태(S=%d)라 열기 구간을 생략합니다." % s0)
        print("시작 회독값 S = %d. 계획: 열기 %s (쓰기 %d%s) → 블록 없음 %d회 %s → 블록 있음 %d회 %s(누르는 창 [95,155], 눌림 감지 시 복귀) → 마무리 %s" % (s0, open_t, len(OPEN_STEPS), "" if not skip_open else ", 생략", FREE_TRIALS, PRESS_STEPS, BLOCK_TRIALS, PRESS_STEPS, FINISH_STEPS))
        if dry:
            print("dry_run: 쓰기·이동 없음. 위 계획이 실제 시험의 목록이다(입력은 이 한 번의 확인뿐).")
            result["phases"].append({"phase": "dry_run_plan", "open": open_t, "press_steps": PRESS_STEPS, "finish": [100] + FINISH_STEPS}); result["ended"] = "completed"
        else:
            if not ask("이 목록을 확인했고 이대로 진행합니까?"): raise Abort("목표 목록 미확인. 쓰기 없음.")
            if not skip_open:
                steps, _ = run_steps("열기", ad, open_t, open_ms); result["phases"].append({"phase": "open", "targets": open_t, "steps": steps})
                if cur != 100: raise Abort("열기가 100에 도착하지 않았다(%s)." % cur)
            else: result["phases"].append({"phase": "open_skipped", "start": s0})
            if a.fake and a.fake_block_early: arm.stall_at = a.fake_block_early   # 열기가 끝난 뒤부터 막힘(블록을 일찍 놓은 실수 모사)
            free = {150: [], 155: []}; block = {150: [], 155: []}
            for tr in range(1, FREE_TRIALS + 1):
                if tr > 1: print("시도 사이 휴식 %d초... (블록 없음 구간입니다. **블록을 아직 두지 마세요.** 블록은 블록 없음 %d회가 모두 끝난 뒤에 둡니다)" % (REST_FREE_S, FREE_TRIALS)); time.sleep(REST_FREE_S / sp)
                ad = make_adapter(arm, WIN_PRESS, 8, dry); s = ad.read_start(); cur = s; start_of_ad = s
                if s not in START_OPEN: raise Abort("누르기 시작 회독 S=%d가 100(±1)이 아니다." % s)
                targets = [s] + PRESS_STEPS; steps, suspect = run_steps("블록 없음 %d" % tr, ad, targets, [1500] * len(PRESS_STEPS), press=True, block=False)
                if not suspect:
                    for st in steps:
                        if st["angle"] in free: free[st["angle"]].append(st["final_readback"])
                result["free_trials"].append({"trial": tr, "steps": steps, "suspect": suspect, "return_final": go_home(ad, s)}); writes_done = True
                if suspect: raise Abort("시도 %d(블록 없음): %s. 열린 쪽으로 자동 복귀했다. 블록·홀더를 손가락 사이에서 치우고 다시 시작하세요." % (tr, suspect))
            print("\n=== 블록 없음 %d회 끝. **지금 블록을 홀더에 고정해 손가락 패드 사이(30 mm 폭 방향)에 두세요.** 다음 질문에서 확인합니다. ===" % FREE_TRIALS)
            if a.fake and a.fake_stall_at: arm.stall_at = a.fake_stall_at
            for tr in range(1, BLOCK_TRIALS + 1):
                if tr > 1: print("시도 사이 휴식 %d초(서보 식히기)..." % REST_BLOCK_S); time.sleep(REST_BLOCK_S / sp)
                if not ask("블록 시도 %d/%d: 블록이 홀더에 고정돼 손가락 패드 사이(30 mm 폭 방향)에 있고, 손·도구가 손가락에서 멀리 있습니까?" % (tr, BLOCK_TRIALS)): raise Abort("블록·손 확인 미충족. 추가 명령 없음.")
                ad = make_adapter(arm, WIN_PRESS, 8, dry); s = ad.read_start(); cur = s; start_of_ad = s
                if s not in START_OPEN: raise Abort("누르기 시작 회독 S=%d가 100(±1)이 아니다." % s)
                targets = [s] + PRESS_STEPS; steps, crush = run_steps("블록 있음 %d" % tr, ad, targets, [1500] * len(PRESS_STEPS), press=True, block=True)
                for st in steps:
                    if st["angle"] in block and st.get("final_readback") is not None and not st.get("crush") and not st.get("unsettled"): block[st["angle"]].append(st["final_readback"])
                result["block_trials"].append({"trial": tr, "steps": steps, "crush": crush, "early_contact": any(st.get("early_contact") for st in steps), "return_final": go_home(ad, s)}); writes_done = True
                if crush: raise Abort("시도 %d: 눌림·조기 접촉 감지(%s). 시험을 중단한다(블록·홀더 위치를 확인하세요)." % (tr, crush))
            if a.fake: arm.stall_at = None
            if not ask("블록과 홀더를 손가락 사이에서 치웠습니까? (마무리 닫기를 시작합니다)"): raise Abort("블록 제거 미확인. 추가 명령 없음.")
            ad = make_adapter(arm, WIN_FINISH, 6, dry); s = ad.read_start(); cur = s; start_of_ad = s
            fin_t = [s] + FINISH_STEPS; steps, _ = run_steps("마무리", ad, fin_t, [1500] * len(FINISH_STEPS)); result["phases"].append({"phase": "finish", "targets": fin_t, "steps": steps})
            result["free_readings"] = free; result["block_readings"] = block; result["summary"] = decide(free, block)
            print("\n명령 150·155의 회독: 블록 없음 %s | 블록 있음 %s" % (free, block)); print("판정:", result["summary"]["verdict"])
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
    except Exception as e:   # I2C 오류 등
        result["ended"] = "error"; result["error"] = repr(e); print("\n오류:", repr(e))
    finally:
        if ad is not None and writes_done and result["ended"] != "completed" and start_of_ad is not None:
            try:
                dist = abs((cur if cur is not None else start_of_ad) - start_of_ad); t_ret = return_time(dist)
                opening = start_of_ad <= 101
                if dist == 0 or getattr(ad, "_returned", False): result["returned_to_start"] = None
                elif t_ret is None: print("복귀 폭 %d°가 승인된 시간으로 갈 수 없다. 복귀하지 않는다." % dist); result["returned_to_start"] = False
                elif ask(("그리퍼가 블록을 누르고 있을 수 있습니다(마지막 명령 %s). 열린 쪽 S=%d로 복귀 쓰기를 1회 보낼까요? 이 복귀는 블록을 놓는 **여는 동작**이며 시간 %d ms입니다. 이상(소리·발열·도착 실패·눌림)이 있으면 y를 권장합니다(여는 쪽은 손가락을 벌려 끼임에 안전)" % (cur, start_of_ad, t_ret)) if opening else ("그리퍼가 이 구간의 시작값 S=%d가 아닐 수 있습니다(마지막 명령 %s). S로 복귀 쓰기를 1회 보낼까요? 이 복귀는 **닫는 동작**(179 쪽)이며 시간 %d ms입니다. 끼임·소리·발열이면 N (손·도구가 멀 때만 y)" % (start_of_ad, cur, t_ret))):
                    ad.confirm_hands_clear(); time.sleep(QUIET_S); ad.return_to_start(t_ret); result["returned_to_start"] = True; result["return_time_ms"] = t_ret; print("복귀 명령 1회 전송")
                else: result["returned_to_start"] = False
            except BaseException as e: result["return_error"] = repr(e); print("복귀 처리 중단:", repr(e))
        result["adapter_logs"] = [[(["write", *l[1:3], "fake" if (a.fake and not a.fake_dry and l[0] == "write") else l[3], *l[4:]] if l[0] == "write" else list(l)) for l in x.log] for x in ADAPTERS]
        result["writes_total"] = sum(1 for x in ADAPTERS for l in x.log if l[0] == "write")
        result["finished_at"] = datetime.datetime.now().isoformat(timespec="seconds")
        Path(a.out).write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"); print("기록 저장:", a.out)


if __name__ == "__main__": main()
