#!/usr/bin/env python3
"""G01.5b-4 쓰기 신뢰도 측정(진단): 한 관절을 소폭(기본 4°)으로 N번 왕복시키며 쓰기마다 회독이 반응하는지 센다(재전송 없이 누락률 측정). 사람이 입회해 직접 실행.
사용: python3 g01_5b4_wiggle_test.py --joint 4 --amp 4 --n 20 --poll 1|4 [--real]   (기본 dry_run: 쓰기 0회)
쓰기는 서보 id 1개에 단일 쓰기(첫 이동 8000 ms, 이후 1500 ms). 누락(회독 불변)이 나도 계획된 다음 목표를 그대로 보낸다(절대 목표라 누적 없음). 끝에 시작값 복귀를 확인한다."""
import argparse, builtins, datetime, json, signal, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gripper_adapter import GripperTestAdapter, GripperTestError
from g01_5b4_joint_test import FakeArm, Hangup, _on_signal, read_valid, last_valid

def say(*a, **k): builtins.print(*a, **k, flush=True)
def inp(p): return builtins.input("\a" + p)
def ask(msg): return inp(msg + " [y/N] ").strip().lower() in ("y", "yes")
class Abort(Exception): pass


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--joint", type=int, choices=[1, 2, 3, 4], default=4); ap.add_argument("--amp", type=int, default=4); ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--poll", type=int, choices=[1, 4], default=1); ap.add_argument("--real", action="store_true"); ap.add_argument("--fake", action="store_true"); ap.add_argument("--fake-drop-every", type=int, default=0)
    ap.add_argument("--pre-delay", type=float, default=0.0, help="쓰기 직전 읽기 뒤 대기(초). 읽기 직후 쓰기 누락 가설 시험용"); ap.add_argument("--pre-delays", default="", help="쉼표 목록(초). 쓰기마다 이 값들을 번갈아 쓰고(교차), --n은 조건당 쓰기 수로 바뀐다"); ap.add_argument("--note", default=""); ap.add_argument("--run-id", default="local"); ap.add_argument("--out", default="/tmp/g01_5b4_wiggle_result.json"); a = ap.parse_args()
    DL = [float(x) for x in a.pre_delays.split(",")] if a.pre_delays else []
    if DL: a.n = a.n * len(DL)
    mode = "fake" if a.fake else ("real" if a.real else "dry_run"); j = a.joint
    result = {"run_id": a.run_id, "mode": mode, "joint": j, "amp": a.amp, "n": a.n, "poll": a.poll, "pre_delay": a.pre_delay, "pre_delays": DL, "note": a.note, "started_at": datetime.datetime.now().isoformat(timespec="seconds"), "trials": [], "ended": None}
    ad = None
    for sg in (signal.SIGHUP, signal.SIGTERM): signal.signal(sg, _on_signal)
    try:
        class F(FakeArm):
            n = 0
            def Arm_serial_servo_write(self, id, angle, time_ms):
                F.n += 1
                if a.fake_drop_every and F.n % a.fake_drop_every == 0: return
                super().Arm_serial_servo_write(id, angle, time_ms)
        arm = F({1: 90, 2: 91, 3: 91, 4: 91, 5: 90, 6: 98}) if a.fake else __import__("Arm_Lib").Arm_Device()
        state = {i: read_valid(arm, i) for i in range(1, 7)}; result["state0"] = state
        say("쓰기 신뢰도 측정(J%d ±%d° × %d회, 회독 %d개 관절, 모드 %s). 현재 회독 %s" % (j, a.amp, a.n, a.poll, mode, state))
        if any(state[i] is None for i in range(1, 7)): raise Abort("회독 None이 있다. 쓰기 없음.")
        if abs(state[1] - 90) > 3 or abs(state[5] - 90) > 3 or abs(state[2] - 91) > 3 or abs(state[3] - 91) > 4 or abs(state[4] - 91) > 3: raise Abort("직립 근처(J1·J5 90, J2~J4 91±3)가 아니다. 쓰기 없음.")
        s = state[j]; today = datetime.date.today()
        tg = [s]; 
        for k in range(a.n): tg.append(s - a.amp if k % 2 == 0 else s)
        if tg[-1] != s: tg.append(s)
        ad = GripperTestAdapter(arm, {"approved": True, "expires": today.isoformat(), "allowed_times_ms": [1500, 8000], "first_move_min_ms": 8000, "max_writes": len(tg) + 2, "delta_deg": 1, "joint_id": j, "direction": -1, "angle_window": [s - a.amp - 3, s + 3], "pattern": "list"}, today=today, dry_run=(mode == "dry_run"))
        ad.read_start(); ad.set_targets(tg); result["targets"] = tg
        others = [k for k in (1, 2, 3, 4) if k != j][:a.poll - 1]
        say("목표 %s (총 쓰기 %d회). 전원 차단 담당자·낙하 받침·팔 주변 비움을 확인하세요. 팔이 ±%d° 작게 왔다 갔다 합니다." % (tg[1:], len(tg) - 1, a.amp))
        if not ask("확인했습니까? (y면 소리 뒤 3초 뒤 시작, 이동 중간에 질문은 없습니다)"): raise Abort("사전 확인 미충족. 쓰기 없음.")
        say("\n[이동 1/%d] J%d 소폭 왕복 시작: 3초 뒤 움직입니다. ▶ J%d를 보세요. 이상하면 Ctrl-C." % (len(tg) - 1, j, j)); time.sleep(3)
        for i, t in enumerate(tg[1:], 1):
            ms = 8000 if i == 1 else 1500
            if mode == "dry_run": result["trials"].append({"i": i, "target": t}); continue
            before = ad.read_position()
            pd = DL[(i - 1) % len(DL)] if DL else a.pre_delay
            if pd > 0: time.sleep(pd)   # 읽기와 쓰기 사이 간격
            t0 = time.monotonic(); ad.confirm_hands_clear(); ad.move(t, ms)
            first = None; series = []
            while (time.monotonic() - t0) * 1000.0 < (ms + 700):
                tt = round((time.monotonic() - t0) * 1000.0); v = ad.read_position(); series.append((tt, v))
                for o in others: arm.Arm_serial_servo_read(o)
                if first is None and v is not None and before is not None and v != before: first = tt
                time.sleep(0.1)
            fin = last_valid(series); tr = {"i": i, "pre_delay": pd, "target": t, "ms": ms, "before": before, "final": fin, "first_change_ms": first, "dropped": first is None and before is not None and abs(t - before) >= 2}
            result["trials"].append(tr); say("  쓰기 %2d/%d 목표 %d: 이전 %s → 최종 %s, 첫 변화 %s ms%s" % (i, len(tg) - 1, t, before, fin, first, "  ← 누락(무반응)" if tr["dropped"] else ""), end="")
            say("  [지연 %.2fs]" % pd)
        dropped = sum(1 for tr in result["trials"] if tr.get("dropped")); result["dropped"] = dropped; result["writes"] = len([tr for tr in result["trials"] if "ms" in tr])
        say("\n결과: 쓰기 %d회 중 누락 %d회 (%.1f%%)" % (result["writes"], dropped, 100.0 * dropped / max(result["writes"], 1)))
        if DL:
            summ = {}
            for d in DL:
                ev = [tr for tr in result["trials"] if tr.get("pre_delay") == d and tr.get("before") is not None and abs(tr["target"] - tr["before"]) >= 2]
                summ[str(d)] = {"evaluable": len(ev), "dropped": sum(1 for tr in ev if tr["dropped"])}
                say("  지연 %.2fs: 판정 가능 %d회 중 누락 %d회" % (d, len(ev), summ[str(d)]["dropped"]))
            result["by_delay"] = summ
        fin = read_valid(arm, j); result["final_readback"] = fin; say("최종 회독 J%d=%s (시작 %d)" % (j, fin, s))
        result["ended"] = "completed"
    except Abort as e: result["ended"] = "aborted"; result["abort_reason"] = str(e); say("\n중단:", e)
    except Hangup as e: result["ended"] = "hangup"; say("\n중단:", e)
    except KeyboardInterrupt: result["ended"] = "interrupted"; say("\n중단(Ctrl-C)")
    except EOFError: result["ended"] = "input_closed"
    except GripperTestError as e: result["ended"] = "adapter_refused"; result["error"] = str(e); say("\n어댑터 거부:", e)
    except Exception as e: result["ended"] = "error"; result["error"] = repr(e); say("\n오류:", repr(e))
    finally:
        result["adapter_log"] = [list(l) for l in (ad.log if ad else [])]; result["finished_at"] = datetime.datetime.now().isoformat(timespec="seconds")
        Path(a.out).write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"); say("기록 저장:", a.out)

if __name__ == "__main__": main()
