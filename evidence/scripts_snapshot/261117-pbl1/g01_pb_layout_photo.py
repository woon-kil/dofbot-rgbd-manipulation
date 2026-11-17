#!/usr/bin/env python3
"""G07 P-B 배치 사진용 J6 열기·대기·닫기 시험(서보 6만, 팔 관절은 명령하지 않는다). 사람이 입회해 직접 실행하는 대화형 스크립트(무인 실행 금지).
승인 요청 문서: docs/gates/G07-PB-J6-연속-누름-시험-승인-요청-문서-초안.md §8-4 (사용자 승인 2026-11-17: J6만·쓰기 8·누름 없음, 구현 + --check·dry run 통과 시 --real 1회까지)
사용: python3 g01_pb_layout_photo.py --fake [--fake-speed 40] [--fake-start 176] [--fake-dry]   (가짜 서보, 하드웨어 없음)
      python3 g01_pb_layout_photo.py                    (실제 SDK 읽기, 쓰기는 dry_run: 쓰기 0회)
      python3 g01_pb_layout_photo.py --real             (실제 쓰기. 확인 문구를 직접 입력해야 한다)
구간: (열기, 시작이 오므림 176~179일 때만) S→160→140→120→100 → 대기(손가락 사이 빈 상태로 사진) → 게이트 y일 때만 닫기 100→120→140→160→179.
게이트는 기본 N이다. N·Ctrl-C·입력 끊김·신호면 닫지 않고 열린 채로 '추가 명령 없음'으로 끝난다(복귀·재전송 경로 없음, 열린 쪽이 끼임에 안전).
누름·정체 구간이 없고 새 안전 값이 없다(구간·시간·창은 P2i·P-B 상수를 import한다).
"""
import argparse, datetime, json, signal, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gripper_adapter import GripperTestError
from g01_5a_gripper_test import Abort, Hangup, _on_signal, ask, POLL_MS
from g01_p2h_gripper_test import poll, make_adapter, START_OPEN, ADAPTERS
from g01_p2g_gripper_test import QUIET_S
from g01_p2i_gripper_test import FastFakeBlock, tail5, OPEN_STEPS, FINISH_STEPS, WIN_OPEN, WIN_FINISH
from g01_pb_press_test import START_CLOSED

PHRASE = "APPROVE PB LAYOUT REAL"


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--fake", action="store_true"); ap.add_argument("--real", action="store_true")
    ap.add_argument("--fake-dry", action="store_true"); ap.add_argument("--fake-speed", type=int, default=1); ap.add_argument("--fake-start", type=int, default=179)
    ap.add_argument("--run-id", default="local"); ap.add_argument("--out", default="/tmp/g01_pb_layout_result.json"); a = ap.parse_args()
    if a.fake and a.real: sys.exit("--fake와 --real은 함께 쓸 수 없다")
    if (a.fake_dry or a.fake_speed != 1 or a.fake_start != 179) and not a.fake: sys.exit("--fake-* 옵션은 --fake에서만 쓸 수 있다")
    mode = "dry_run" if (a.fake and a.fake_dry) else ("fake" if a.fake else ("real" if a.real else "dry_run"))
    dry = (mode == "dry_run"); sp = a.fake_speed if a.fake else 1
    result = {"run_id": a.run_id, "mode": mode, "started_at": datetime.datetime.now().isoformat(timespec="seconds"), "windows": {"open": list(WIN_OPEN), "finish": list(WIN_FINISH)}, "phases": [], "gate_close": None, "ended": None}
    for sg in (signal.SIGHUP, signal.SIGTERM): signal.signal(sg, _on_signal)
    writes_done = False

    def move_steps(label, adapter, targets, times):
        """승인된 목록을 순서대로 이동하고 단계마다 정착 확인(도착 실패면 중단, 재전송·복귀 없음)."""
        nonlocal writes_done
        adapter.set_targets(targets); steps = []; cur = targets[0]
        for i, (tgt, t_ms) in enumerate(zip(targets[1:], times), 1):
            adapter.confirm_hands_clear(); time.sleep(QUIET_S / sp); adapter.move(tgt, t_ms); writes_done = writes_done or not dry; cur = tgt
            if dry: steps.append({"n": i, "angle": tgt, "time_ms": t_ms, "dry": True}); continue
            stop = lambda sr, el, _t=tgt, _m=t_ms: el >= _m / sp and len(tail5(sr)) == 5 and max(tail5(sr)) - min(tail5(sr)) <= 1 and abs(_t - tail5(sr)[-1]) <= 3
            series = poll(adapter, int(2 * t_ms + 1000) // sp, POLL_MS / sp, stop=stop)
            last = next((v for _, v in reversed(series) if v is not None), None); steps.append({"n": i, "angle": tgt, "time_ms": t_ms, "final_readback": last, "series": series})
            print("  [%s] 이동 %d/%d: %d (%d ms) → 회독 %s" % (label, i, len(targets) - 1, tgt, t_ms, last))
            if last is None or abs(last - tgt) > 3: raise Abort("도착 실패(회독 %s, 목표 %d, 허용 3°). 추가 명령 없음(재전송·복귀 없음)." % (last, tgt))
        return steps, cur

    try:
        print("G07 P-B 배치 사진용 J6 열기·대기·닫기(%s): 그리퍼 서보 6만 움직인다. 팔 관절은 명령하지 않는다." % mode)
        print("사전 확인:")
        for k, q in enumerate(("전원 차단 담당자가 현재 자리에 있다", "팔이 K1 기립(대기) 자세이고 낙하 받침이 준비돼 있다(전원 차단 시 직립 팔이 낙하할 수 있음)", "손가락 사이에 아무것도 없다(사진은 **빈 상태**로 찍는다. 홀더 집게·블록은 손가락 사이에 넣지 않는다)", "사진 2장을 찍을 준비(정면=사용자 쪽, 옆=방향을 말해 둔다). 눈금자는 손가락에 닿지 않게 둔다. 열린 뒤 부하는 없다. 소리·발열·떨림이 보이면 즉시 Ctrl-C(열린 채로 끝난다)"), 1): print("  %d. %s" % (k, q))
        if not ask("위 4개를 모두 확인했습니까? (질문은 이 한 번으로 묶었습니다)"): raise Abort("사전 확인 미충족. 명령은 보내지 않았다.")
        if a.real and input("실제 쓰기를 시작하려면 정확히 '%s'를 입력: " % PHRASE).strip() != PHRASE: raise Abort("승인 문구 불일치. 명령은 보내지 않았다.")
        if a.fake: arm = FastFakeBlock(a.fake_start, sp)
        else:
            from Arm_Lib import Arm_Device; arm = Arm_Device()
        ad = make_adapter(arm, WIN_OPEN, 6, dry, first_min=3000)
        s0 = ad.read_start(); skip_open = s0 in START_OPEN
        if s0 not in START_CLOSED and not skip_open: raise Abort("시작 회독 S=%d가 오므림 범위 %s도 열림 범위 %s도 아니다. 명령은 보내지 않았다." % (s0, list(START_CLOSED), list(START_OPEN)))
        open_t = [s0] + OPEN_STEPS; open_ms = [3000] + [1500] * (len(OPEN_STEPS) - 1)
        print("시작 회독값 S = %d. 계획: 열기 %s%s → 대기(사진 2장, 빈 손가락) → 게이트 y일 때만 닫기 %s" % (s0, open_t, ", 생략" if skip_open else "", [100] + FINISH_STEPS))
        if dry:
            print("dry_run: 쓰기·이동 없음. 위 계획이 실제 시험의 목록이다(입력은 이 한 번의 확인뿐).")
            result["phases"].append({"phase": "dry_run_plan", "open": open_t, "skip_open": skip_open, "finish": [100] + FINISH_STEPS}); result["ended"] = "completed"
        else:
            if not ask("이 목록을 확인했고 이대로 진행합니까?"): raise Abort("목표 목록 미확인. 쓰기 없음.")
            if not skip_open:
                steps, cur = move_steps("열기", ad, open_t, open_ms); result["phases"].append({"phase": "open", "targets": open_t, "steps": steps})
                if cur != 100: raise Abort("열기가 100에 도착하지 않았다(%s)." % cur)
            else: result["phases"].append({"phase": "open_skipped", "start": s0})
            print("\n=== 손가락이 100(열림)입니다. **지금 사진 2장(정면·옆)을 찍으세요.** 천천히 하셔도 됩니다. 이 동안 부하·명령은 없습니다. ===")
            go = ask("사진을 마쳤고 홀더·자·손을 손가락 근처에서 **모두 치웠습니까?** y면 닫기를 시작하고, N(기본)·Ctrl-C면 열린 채로 끝납니다")
            result["gate_close"] = bool(go)
            if not go: result["ended"] = "left_open"; print("닫지 않고 열린 채로 끝냅니다. 추가 명령 없음.")
            else:
                ad2 = make_adapter(arm, WIN_FINISH, 6, dry); s = ad2.read_start()
                if s not in START_OPEN: raise Abort("닫기 시작 회독 S=%d가 100(±1)이 아니다. 추가 명령 없음." % s)
                fin_t = [s] + FINISH_STEPS; steps, cur = move_steps("닫기", ad2, fin_t, [1500] * len(FINISH_STEPS)); result["phases"].append({"phase": "finish", "targets": fin_t, "steps": steps})
                result["ended"] = "completed"
    except Abort as e: result["ended"] = "aborted"; result["abort_reason"] = str(e); print("\n중단:", e)
    except Hangup as e: result["ended"] = "hangup"; result["abort_reason"] = str(e); print("\n중단:", e, "이후 명령 없음(닫지 않는다)")
    except KeyboardInterrupt: result["interrupted"] = True; result["ended"] = "interrupted"; print("\n중단(Ctrl-C): 이후 명령 없음(닫지 않는다)")
    except EOFError: result["ended"] = "input_closed"; print("\n중단: 입력이 끊겼다. 이후 명령 없음(닫지 않는다)")
    except GripperTestError as e: result["ended"] = "adapter_refused"; result["error"] = str(e); print("\n어댑터 거부:", e)
    except Exception as e: result["ended"] = "error"; result["error"] = repr(e); print("\n오류:", repr(e))
    finally:
        result["writes_done"] = writes_done
        result["adapter_logs"] = [[(["write", *l[1:3], "fake" if (a.fake and not a.fake_dry and l[0] == "write") else l[3], *l[4:]] if l[0] == "write" else list(l)) for l in x.log] for x in ADAPTERS]
        result["writes_total"] = sum(1 for x in ADAPTERS for l in x.log if l[0] == "write")
        result["finished_at"] = datetime.datetime.now().isoformat(timespec="seconds")
        Path(a.out).write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"); print("기록 저장:", a.out)


if __name__ == "__main__": main()
