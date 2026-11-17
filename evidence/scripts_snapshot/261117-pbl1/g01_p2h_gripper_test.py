#!/usr/bin/env python3
"""G07 P2h 블록을 지지한 채 닫기(정체 회독 읽기) 시험(서보 6만). 사람이 입회해 직접 실행하는 대화형 스크립트(무인 실행 금지).
승인 요청 문서: docs/gates/G07-P2h-블록-지지-닫기-승인-요청-문서.md (H1 a안 홀더 고정, H2~H6 승인, R8 `--real` 승인은 도구·가짜 시험·dry run 뒤 별도)
사용: python3 g01_p2h_gripper_test.py --fake [--fake-speed 20] [--fake-stall-at 141]   (가짜 서보, 하드웨어 없음)
      python3 g01_p2h_gripper_test.py                              (실제 SDK 읽기, 쓰기는 dry_run: 쓰기 0회, 입력 1번)
      python3 g01_p2h_gripper_test.py --real                       (실제 쓰기. 확인 문구를 직접 입력해야 한다)
구간: (열기) 179→160→140→120→100 [쓰기 4, 블록 없음] → (닫기 계단 3회) 100→120→130→135→140→145→150, 정체가 감지되면 거기서 중단하고 S(100)로 복귀 [시도당 최대 7] → (마무리) 100→120→140→160→179 [쓰기 4, 블록 제거].
정체 판정: 이동이 끝나고 정착한 뒤 마지막 5회 읽기의 최대−최소가 1° 이하이고 명령 − 최대값 ≥ 3이면 정체(검토 반영: 같은 값만 인정하면 1° 오르내림을 놓쳐 도착 실패로 중단된다). 정체되면 더 높은 목표로 가지 않고 S로 복귀한다(과도한 누름·발열 방지).
"""
import argparse, datetime, json, signal, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gripper_adapter import GripperTestAdapter, GripperTestError
from g01_5a_gripper_test import Abort, Hangup, _on_signal, ask, settle_ms, POLL_MS
from g01_p2g_gripper_test import FastFake, return_time, QUIET_S

ADAPTERS = []                                    # 구간마다 새 어댑터를 만든다. 결과 JSON에는 전부의 기록을 남긴다(검토 반영: 이전에는 마지막 어댑터 기록만 남았다)
ALLOWED_MS = [1500, 3000, 4800]                  # 4800 ms는 중단 뒤 복귀 전용(60 ms/° 상한, G5 승인과 같은 방식)
OPEN_STEPS = [160, 140, 120, 100]                # 열기: 179에서 열림 쪽(각도 감소)
CLOSE_STEPS = [120, 130, 135, 140, 145, 150]     # 닫기 계단(시작 S=100)
FINISH_STEPS = [120, 140, 160, 179]              # 마무리 닫기(블록 제거 뒤)
WIN_OPEN, WIN_BLOCK, WIN_FINISH = (95, 179), (95, 150), (95, 179)   # 각도 창(어댑터가 코드로 강제). 블록 닫기는 150 초과 금지
STALL_DEG = 3                                    # 명령 − (마지막 5회 읽기 최대값) ≥ 3이고 5회 읽기 폭 ≤ 1°이면 정체(P2g 자유 이동 잔차는 1~2°)
TRIALS = 3; REST_S = 30
RETURN_MS = 3000                                 # 정체 뒤 복귀(최대 50° × 60 ms/° = 3000 ms)
START_CLOSED = (177, 178, 179)                  # P2g 마지막 이동(179 명령)의 정지값이 177이었다(간격 약 6.7 mm): 오므림 시작 허용 범위를 177~179로 넓힘(검토 반영, 문서 §8)
START_OPEN = (99, 100, 101)                      # 정체 뒤 복귀가 ±1로 읽힐 수 있어 닫기 시작을 99~101로 허용(창 [95,150] 안)
PHRASE = "APPROVE P2H REAL"
PASS_RANGE = 2; PASS_LO, PASS_HI = 130, 150      # 합격 제안(문서 §3): 3회 정체 회독 범위 ≤ 2°, 모두 130~150 안


def poll(ad, total_ms, interval_ms=POLL_MS, stop=None):
    """도착 확인용 회독 시계열(서보 6만 읽음). 실제 시험은 100 ms 간격, 가짜 시험은 --fake-speed로 간격도 줄인다.
    stop(시계열, 경과 ms)이 True면 바로 끝낸다(정체 감지 뒤 서보가 블록을 계속 누르는 시간을 줄이기 위해, H4 '3초 안 복귀')."""
    series = []; t0 = time.monotonic()
    while (time.monotonic() - t0) * 1000.0 < total_ms:
        el = (time.monotonic() - t0) * 1000.0; series.append((round(el), ad.read_position()))
        if stop is not None and stop(series, el): break
        time.sleep(interval_ms / 1000.0)
    return series


def is_stall(cmd, series):
    """정착 뒤 마지막 5회 유효 읽기가 모두 같고 명령보다 STALL_DEG 이상 작으면 정체."""
    tail = [v for _, v in series if v is not None][-5:]
    ok = len(tail) == 5 and max(tail) - min(tail) <= 1 and cmd - max(tail) >= STALL_DEG   # 블록을 누르는 서보 회독이 1° 안에서 오르내려도 정체(최대값 기준 잔차 ≥ 3°)
    return ok, (max(tail) if tail else None)


def make_adapter(arm, window, max_writes, dry, first_min=None):
    today = datetime.date.today()
    appr = {"approved": True, "expires": today.isoformat(), "allowed_times_ms": ALLOWED_MS, "max_writes": max_writes, "delta_deg": 5, "joint_id": 6, "direction": -1, "angle_window": list(window), "pattern": "list"}
    if first_min: appr["first_move_min_ms"] = first_min
    ad_ = GripperTestAdapter(arm, appr, today=today, dry_run=dry); ADAPTERS.append(ad_); return ad_


class FastFakeStall(FastFake):
    """가짜 서보: stall_at이 있으면 그 각도를 넘어 닫히지 못한다(블록에 물린 상태 모사)."""
    def __init__(self, v, speed): super().__init__(v, speed); self.stall_at = None
    def Arm_serial_servo_write(self, id, angle, time_ms):
        if self.stall_at is not None and angle > self.stall_at: angle = self.stall_at
        super().Arm_serial_servo_write(id, angle, time_ms)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--fake", action="store_true"); ap.add_argument("--real", action="store_true")
    ap.add_argument("--fake-dry", action="store_true"); ap.add_argument("--fake-speed", type=int, default=1)
    ap.add_argument("--fake-start", type=int, default=179); ap.add_argument("--fake-stall-at", type=int, default=None, help="--fake에서만: 블록 닫기 구간에서 이 각도를 넘지 못한다(없으면 접촉 없음)")
    ap.add_argument("--run-id", default="local"); ap.add_argument("--out", default="/tmp/g01_p2h_result.json"); a = ap.parse_args()
    if a.fake and a.real: sys.exit("--fake와 --real은 함께 쓸 수 없다")
    if (a.fake_dry or a.fake_speed != 1 or a.fake_stall_at) and not a.fake: sys.exit("--fake-* 옵션은 --fake에서만 쓸 수 있다")
    mode = "dry_run" if (a.fake and a.fake_dry) else ("fake" if a.fake else ("real" if a.real else "dry_run"))
    dry = (mode == "dry_run"); sp = a.fake_speed if a.fake else 1
    result = {"run_id": a.run_id, "mode": mode, "started_at": datetime.datetime.now().isoformat(timespec="seconds"), "windows": {"open": list(WIN_OPEN), "block": list(WIN_BLOCK), "finish": list(WIN_FINISH)}, "allowed_times_ms": ALLOWED_MS, "stall_deg": STALL_DEG, "phases": [], "trials": [], "interrupted": False, "ended": None}
    ad = None; cur = None; start_of_ad = None; writes_done = False; adapters = []
    for sg in (signal.SIGHUP, signal.SIGTERM): signal.signal(sg, _on_signal)

    def run_list(label, adapter, targets, times, ch_stall=False):
        """승인된 목록을 순서대로 이동한다. ch_stall=True면 정체를 판정해 정체하면 (정체 회독, 단계)를 반환한다."""
        nonlocal cur, writes_done
        adapter.set_targets(targets); steps = []
        for i, (tgt, t_ms) in enumerate(zip(targets[1:], times), 1):
            adapter.confirm_hands_clear(); time.sleep(QUIET_S); adapter.move(tgt, t_ms); writes_done = writes_done or not dry; cur = tgt
            if dry: steps.append({"n": i, "angle": tgt, "time_ms": t_ms, "dry": True}); continue
            series = poll(adapter, (2 * t_ms + 1000) // sp, POLL_MS / sp, stop=((lambda sr, el, _t=tgt, _m=t_ms: el >= _m / sp and is_stall(_t, sr)[0]) if ch_stall else None)); st = settle_ms(series); last = next((v for _, v in reversed(series) if v is not None), None)
            step = {"n": i, "angle": tgt, "time_ms": t_ms, "settle_ms": st, "final_readback": last, "series": series}; steps.append(step)
            print("  [%s] 이동 %d/%d: %d (%d ms) → 최종 회독 %s" % (label, i, len(targets) - 1, tgt, t_ms, last))
            stalled, val = is_stall(tgt, series)
            if ch_stall and stalled:
                step["stall"] = val; print("  ★ 정체 감지: 명령 %d, 회독 %s (잔차 %d°). 더 닫지 않고 복귀합니다." % (tgt, val, tgt - val)); return steps, val
            if last is None or abs(last - tgt) > 3: step["arrival_failed"] = True; raise Abort("도착 실패(회독 %s, 목표 %d, 허용 3°). 추가 이동 없음." % (last, tgt))
        return steps, None

    try:
        print("G07 P2h 블록을 지지한 채 닫기(%s): 그리퍼 서보 6만 움직인다. 팔 관절은 명령하지 않는다." % mode)
        print("사전 확인:")
        for k, q in enumerate(("전원 차단 담당자가 현재 자리에 있다", "팔이 K1 기립(대기) 자세이고 낙하 받침이 준비돼 있다(전원 차단 시 직립 팔이 낙하할 수 있음)", "열기 구간(179→100) 동안 손가락 사이에 아무것도 없다", "블록 홀더(스탠드·집게)와 블록이 준비돼 있다(블록은 아직 손가락 사이에 두지 않음)", "블록은 손가락 패드에 30 mm 폭이 닿는 방향으로 놓을 것이며, 시도마다 손·도구가 손가락에서 멀다"), 1): print("  %d. %s" % (k, q))
        if not ask("위 5개를 모두 확인했습니까? (질문은 이 한 번으로 묶었습니다)"): raise Abort("사전 확인 미충족. 명령은 보내지 않았다.")
        if a.real and input("실제 쓰기를 시작하려면 정확히 '%s'를 입력: " % PHRASE).strip() != PHRASE: raise Abort("승인 문구 불일치. 명령은 보내지 않았다.")
        if a.fake:
            arm = FastFakeStall(a.fake_start, sp)
        else:
            from Arm_Lib import Arm_Device; arm = Arm_Device()   # 생성자는 버스를 열기만 한다(소스 확인)
        ad = make_adapter(arm, WIN_OPEN, 6, dry, first_min=3000)
        s0 = ad.read_start(); cur = s0; start_of_ad = s0
        if s0 not in START_CLOSED: raise Abort("시작 회독 S=%d가 오므림 범위 %s가 아니다. 열기 구간은 오므림에서 시작한다. 명령은 보내지 않았다." % (s0, list(START_CLOSED)))
        open_t = [s0] + OPEN_STEPS; open_ms = [3000] + [1500] * (len(OPEN_STEPS) - 1)
        print("시작 회독값 S = %d. 계획: 열기 %s (쓰기 %d) → 닫기 계단 %s ×%d회(정체 감지 시 중단·복귀, S=100) → 마무리 %s (쓰기 %d)" % (s0, open_t, len(OPEN_STEPS), CLOSE_STEPS, TRIALS, FINISH_STEPS, len(FINISH_STEPS)))
        if dry:
            print("dry_run: 쓰기·이동 없음. 위 계획이 실제 시험의 목록이다(입력은 이 한 번의 확인뿐).")
            result["phases"].append({"phase": "dry_run_plan", "open": open_t, "close_steps": CLOSE_STEPS, "finish": [100] + FINISH_STEPS}); result["ended"] = "completed"
        else:
            if not ask("이 목록을 확인했고 이대로 진행합니까?"): raise Abort("목표 목록 미확인. 쓰기 없음.")
            steps, _ = run_list("열기", ad, open_t, open_ms); result["phases"].append({"phase": "open", "targets": open_t, "steps": steps})
            if cur != 100: raise Abort("열기가 100에 도착하지 않았다(%s)." % cur)
            if a.fake and a.fake_stall_at: arm.stall_at = a.fake_stall_at
            stalls = []
            for tr in range(1, TRIALS + 1):
                if tr > 1:
                    print("시도 사이 휴식 %d초(서보 식히기)..." % REST_S); time.sleep(REST_S / sp)
                if not ask("시도 %d/%d: 블록이 홀더에 고정돼 손가락 패드 사이(30 mm 폭 방향)에 있고, 손·도구가 손가락에서 멀리 있습니까?" % (tr, TRIALS)): raise Abort("블록·손 확인 미충족. 추가 명령 없음.")
                ad = make_adapter(arm, WIN_BLOCK, 8, dry); s = ad.read_start(); cur = s; start_of_ad = s
                if s not in START_OPEN: raise Abort("닫기 시작 회독 S=%d가 100(±1)이 아니다." % s)
                targets = [s] + CLOSE_STEPS; steps, stall = run_list("닫기 %d" % tr, ad, targets, [1500] * len(CLOSE_STEPS), ch_stall=True)
                trial = {"trial": tr, "targets": targets, "steps": steps, "stall_readback": stall}
                ad.confirm_hands_clear(); time.sleep(QUIET_S); ad.return_to_start(RETURN_MS); cur = s; writes_done = True
                series = poll(ad, (2 * RETURN_MS + 1000) // sp, POLL_MS / sp); trial["return_final"] = next((v for _, v in reversed(series) if v is not None), None)
                print("  복귀: 회독 %s" % trial["return_final"]); result["trials"].append(trial)
                if stall is None: raise Abort("시도 %d: 150까지 정체 없음(블록 접촉 없음 또는 위치 어긋남). 예측(약 141)이 틀렸을 수 있다. 중단." % tr)
                stalls.append(stall)
            if a.fake: arm.stall_at = None
            if not ask("블록과 홀더를 손가락 사이에서 치웠습니까? (마무리 닫기를 시작합니다)"): raise Abort("블록 제거 미확인. 추가 명령 없음.")
            ad = make_adapter(arm, WIN_FINISH, 6, dry); s = ad.read_start(); cur = s; start_of_ad = s
            fin_t = [s] + FINISH_STEPS; steps, _ = run_list("마무리", ad, fin_t, [1500] * len(FINISH_STEPS)); result["phases"].append({"phase": "finish", "targets": fin_t, "steps": steps})
            rng = max(stalls) - min(stalls)
            result["summary"] = {"stall_readbacks": stalls, "range": rng, "mean": round(sum(stalls) / len(stalls), 2), "pass_range": rng <= PASS_RANGE, "pass_window": all(PASS_LO <= x <= PASS_HI for x in stalls)}
            print("\n정체 회독 3회:", stalls, "범위", rng, "평균", result["summary"]["mean"], "| 합격 제안(범위 ≤ %d°, 130~150): %s" % (PASS_RANGE, "충족" if result["summary"]["pass_range"] and result["summary"]["pass_window"] else "미충족"))
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
                if dist == 0: result["returned_to_start"] = None
                elif t_ret is None: print("복귀 폭 %d°가 승인된 시간으로 갈 수 없다. 복귀하지 않는다." % dist); result["returned_to_start"] = False
                elif ask(("그리퍼가 블록을 누르고 있을 수 있습니다(마지막 명령 %s). 열린 쪽 S=%d로 복귀 쓰기를 1회 보낼까요? 이 복귀는 블록을 놓는 **여는 동작**이며 시간 %d ms입니다. 이상(소리·발열·도착 실패)이 있으면 y를 권장합니다(여는 쪽은 손가락을 벌려 끼임에 안전)" % (cur, start_of_ad, t_ret)) if opening else ("그리퍼가 이 구간의 시작값 S=%d가 아닐 수 있습니다(마지막 명령 %s). S로 복귀 쓰기를 1회 보낼까요? 이 복귀는 **닫는 동작**(179 쪽)이며 시간 %d ms입니다. 끼임·소리·발열이면 N (손·도구가 멀 때만 y)" % (start_of_ad, cur, t_ret))):
                    ad.confirm_hands_clear(); time.sleep(QUIET_S); ad.return_to_start(t_ret); result["returned_to_start"] = True; result["return_time_ms"] = t_ret; print("복귀 명령 1회 전송")
                else: result["returned_to_start"] = False
            except BaseException as e: result["return_error"] = repr(e); print("복귀 처리 중단:", repr(e))
        result["adapter_logs"] = [[(["write", *l[1:3], "fake" if (a.fake and not a.fake_dry and l[0] == "write") else l[3], *l[4:]] if l[0] == "write" else list(l)) for l in x.log] for x in ADAPTERS]
        result["writes_total"] = sum(1 for x in ADAPTERS for l in x.log if l[0] == "write")
        result["adapter_log_last"] = [(["write", *l[1:3], "fake" if (a.fake and not a.fake_dry and l[0] == "write") else l[3], *l[4:]] if l[0] == "write" else list(l)) for l in (ad.log if ad else [])]
        result["finished_at"] = datetime.datetime.now().isoformat(timespec="seconds")
        Path(a.out).write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"); print("기록 저장:", a.out)


if __name__ == "__main__": main()
