#!/usr/bin/env python3
"""G01.5a 그리퍼(서보 6) 소각도 시험. 사람이 입회해 직접 실행하는 대화형 스크립트(무인 실행 금지).
사용: python3 g01_5a_gripper_test.py --fake            (가짜 서보, 하드웨어 없음)
      python3 g01_5a_gripper_test.py                   (실제 SDK 읽기, 쓰기는 dry_run: 쓰기 0회, 도착 검사 생략)
      python3 g01_5a_gripper_test.py --real            (실제 쓰기. 확인 문구를 직접 입력해야 한다)
중단·오류·입력 끊김·Ctrl-C가 나도 기록을 저장한다. 중단 후에는 운영자가 명시적으로 y로 답할 때만 시작값 S로 복귀 쓰기 1회를 보낸다(기본 N).
"""
import argparse, datetime, json, signal, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gripper_adapter import GripperTestAdapter, GripperTestError

DELTA = 5
TIMES = [1500, 3000, 1500]      # 이동 3회(S-Δ, S-2Δ, 복귀). 같은 Δ를 1500/3000 ms로 움직여 time 단위를 비교한다
POLL_MS = 100


class FakeServo:
    """가짜 서보: write 후 time ms 동안 선형 이동. limit이 있으면 그 값을 넘어서 가지 못한다(기계 한계 모사). Arm_Lib.py 시그니처를 따른다."""
    def __init__(self, v=100, limit=None): self.v0 = v; self.v1 = v; self.t0 = 0.0; self.dur = 0.0; self.limit = limit; self.writes = []
    def _pos(self):
        e = (time.monotonic() - self.t0) * 1000.0
        return self.v1 if e >= self.dur else self.v0 + (self.v1 - self.v0) * e / self.dur
    def Arm_serial_servo_read(self, id): return int(round(self._pos()))
    def Arm_serial_servo_write(self, id, angle, time_ms):
        self.v0 = self._pos(); self.v1 = angle if (self.limit is None or angle >= self.limit) else self.limit
        self.t0 = time.monotonic(); self.dur = time_ms; self.writes.append((id, angle, time_ms))
    def __getattr__(self, n): raise AssertionError("허용되지 않은 SDK API: " + n)


class Abort(Exception):
    pass


class Hangup(Exception):
    pass


def _on_signal(signum, frame):
    raise Hangup("신호 %d 수신(연결 끊김·종료)" % signum)


PHRASE = "APPROVE G01.5A REAL"   # ASCII (한글 입력 환경 차이 방지)


def ask(msg, default=False):
    ans = input(msg + (" [Y/n] " if default else " [y/N] ")).strip().lower()
    return default if ans == "" else ans in ("y", "yes")


def poll(ad, total_ms):
    series = []; t0 = time.monotonic()
    while (time.monotonic() - t0) * 1000.0 < total_ms:
        series.append((round((time.monotonic() - t0) * 1000.0), ad.read_position())); time.sleep(POLL_MS / 1000.0)
    return series


def settle_ms(series):
    """None 읽기(쓰기 직후·이동 중 일시 실패, G01.5a 실측 108회 중 3회)는 변화로 취급하지 않고 무시한다."""
    valid = [(t, v) for t, v in series if v is not None]
    if len(valid) < 3: return None
    final = valid[-1][1]; settled = None
    for t, v in valid:
        if v == final and settled is None: settled = t
        if v != final: settled = None
    return settled


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--fake", action="store_true"); ap.add_argument("--real", action="store_true")
    ap.add_argument("--fake-limit", type=int, default=None); ap.add_argument("--run-id", default="local"); ap.add_argument("--out", default="/tmp/g01_5a_result.json"); a = ap.parse_args()
    if a.fake and a.real: sys.exit("--fake와 --real은 함께 쓸 수 없다")
    mode = "fake" if a.fake else ("real" if a.real else "dry_run")
    result = {"run_id": a.run_id, "mode": mode, "started_at": datetime.datetime.now().isoformat(timespec="seconds"), "delta_deg": DELTA, "times_ms": TIMES, "steps": [], "interrupted": False, "ended": None}
    ad = None; writes_done = False
    for sg in (signal.SIGHUP, signal.SIGTERM): signal.signal(sg, _on_signal)
    try:
        print("G01.5a 소각도 시험(%s): 그리퍼 서보 6만 움직인다. 팔 관절은 움직이지 않는다. 물체 없음." % mode)
        print("사전 확인(모두 y여야 진행):")
        for q in ("전원 차단 담당자가 현재 자리에 있습니까?", "낙하 받침을 준비했습니까?", "손가락 사이에 물체·손·케이블이 없습니까?", "캘리퍼는 지금 손가락에서 떼어져 있습니까?"):
            if not ask(q): raise Abort("사전 확인 미충족. 명령은 보내지 않았다.")
        if a.real:
            if input("실제 쓰기를 시작하려면 정확히 '%s'를 입력: " % PHRASE).strip() != PHRASE: raise Abort("승인 문구 불일치. 명령은 보내지 않았다.")
        elif mode == "dry_run":
            input("(연습) 실제 시험에서는 '%s'를 입력해야 합니다. 지금 입력해 보세요(틀려도 진행): " % PHRASE)
        if a.fake: arm = FakeServo(100, a.fake_limit)
        else:
            from Arm_Lib import Arm_Device; arm = Arm_Device()   # 생성자는 버스를 열기만 한다(소스 확인)
        today = datetime.date.today()
        ad = GripperTestAdapter(arm, {"approved": True, "expires": today.isoformat(), "allowed_times_ms": sorted(set(TIMES)), "max_writes": 6, "delta_deg": DELTA}, today=today, dry_run=(mode == "dry_run"))
        s = ad.read_start(); targets = [s, s - DELTA, s - 2 * DELTA, s]; result["targets"] = targets
        print("시작 회독값 S = %d. 절대 목표 목록 = %s (첫 값은 현재 위치, 쓰기는 두 번째부터)" % (s, targets))
        if not ask("이 목록을 확인했고 이대로 진행합니까?"): raise Abort("목표 목록 미확인. 쓰기 없음.")
        ad.set_targets(targets)
        result["steps"].append({"point": "start", "angle": s, "caliper_mm": input("현재 위치(S=%d)에서 캘리퍼 간격(mm, 안쪽 평판)을 재고 입력(엔터=건너뜀): " % s).strip() or None})
        for i, (tgt, t_ms) in enumerate(zip(targets[1:], TIMES)):
            if not ask("손과 캘리퍼를 손가락에서 뗐습니까? 다음 이동: %d (time %d ms)" % (tgt, t_ms)): raise Abort("손 제거 미확인. 추가 명령 없음.")
            ad.confirm_hands_clear(); ad.move(tgt, t_ms); writes_done = writes_done or mode != "dry_run"
            if mode == "dry_run":
                print("  dry_run: 이동 없음(쓰기 0회, 도착 검사 생략)"); result["steps"].append({"point": "dry_run_move", "angle": tgt, "time_ms": t_ms}); continue
            series = poll(ad, 2 * t_ms + 1000); st = settle_ms(series); last = next((v for _, v in reversed(series) if v is not None), None)  # 마지막 유효 회독(일시 None 무시)
            step = {"point": "move", "angle": tgt, "time_ms": t_ms, "settle_ms": st, "final_readback": last, "series": series}; result["steps"].append(step)
            print("  -> 이동 %d, time %d ms: 정착 %s ms, 최종 회독 %s" % (tgt, t_ms, st, last))
            if last is None or abs(last - tgt) > 3:
                step["arrival_failed"] = True; raise Abort("도착 실패(회독 %s, 목표 %d, 허용 3°). 서보는 마지막 목표를 유지 중일 수 있다. 추가 이동 없음." % (last, tgt))
            if abs(last - tgt) >= 2: print("  ! 경고: 회독 잔차가 %d°입니다. 한계에 밀리고 있을 수 있으니 소리·떨림·발열을 확인하세요." % abs(last - tgt))
            step["caliper_mm"] = input("  손이 닿지 않는 상태에서 캘리퍼 간격(mm)을 재고 입력(엔터=건너뜀): ").strip() or None
            if i >= 1:
                step["gap_changed"] = ask("  직전 측정보다 손가락 간격이 바뀌었습니까?")
                if not step["gap_changed"]: raise Abort("간격이 바뀌지 않음: 한계 도달 또는 명령 불일치 의심. 추가 이동 없음.")
            if i == 0:
                d = input("  첫 이동 후 손가락이 벌어졌습니까, 좁혀졌습니까, 변화 없습니까? [벌어짐/좁혀짐/없음]: ").strip()
                result["direction_after_first_move"] = d; step["direction"] = d
                if d in ("없음", ""): raise Abort("손가락 변화 없음(또는 입력 없음): 한계 도달 또는 명령 불일치 의심. 추가 이동 없음.")
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
        # 중단 후 복귀: 쓰기가 있었고 운영자가 명시적으로 y일 때만 시작값 S로 1회 쓴다(기본 N)
        if ad is not None and writes_done and result["ended"] != "completed":
            try:
                if ask("그리퍼가 시작값 S=%d가 아닐 수 있습니다. S로 복귀 쓰기를 1회 보낼까요? (손·캘리퍼를 뗀 뒤에만 y)" % result["targets"][0]):
                    ad.confirm_hands_clear(); ad.return_to_start(TIMES[0]); result["returned_to_start"] = True; print("복귀 명령 1회 전송")
                else: result["returned_to_start"] = False
            except BaseException as e: result["return_error"] = repr(e); print("복귀 처리 중단:", repr(e))
        moves = [x for x in result["steps"] if x["point"] == "move" and x.get("settle_ms")]
        if len(moves) >= 2: result["ratio_settle_3000_over_1500"] = round(moves[1]["settle_ms"] / moves[0]["settle_ms"], 2); print("정착 시간 비(3000/1500) =", result["ratio_settle_3000_over_1500"])
        result["adapter_log"] = [(["write", *l[1:3], "fake" if (a.fake and l[0] == "write") else l[3], *l[4:]] if l[0] == "write" else list(l)) for l in (ad.log if ad else [])]
        result["finished_at"] = datetime.datetime.now().isoformat(timespec="seconds")
        Path(a.out).write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"); print("기록 저장:", a.out)

if __name__ == "__main__": main()
