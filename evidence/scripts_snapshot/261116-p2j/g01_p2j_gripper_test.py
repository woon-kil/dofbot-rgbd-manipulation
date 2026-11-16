#!/usr/bin/env python3
"""G07 P2j K1 직립 J6별 캡 끝 위치 측정(서보 6만; P2g 사본, 측정 입력만 사진 번호로 교체). [원본 P2g: G07 P2g 그리퍼 단독 개폐·간격 대응표 시험(서보 6만).] 사람이 입회해 직접 실행하는 대화형 스크립트(무인 실행 금지).
승인 요청 문서: docs/gates/G07-P2j-J6별-캡끝-위치-승인-요청-문서.md (P2j-1~3 승인 2026-11-16, R-P2j `--real` 승인은 도구·점검·dry run 뒤 별도)
사용: python3 g01_p2j_gripper_test.py --fake [--fake-speed 20]   (가짜 서보, 하드웨어 없음)
      python3 g01_p2j_gripper_test.py                              (실제 SDK 읽기, 쓰기는 dry_run: 쓰기 0회)
      python3 g01_p2j_gripper_test.py --real                       (실제 쓰기. 확인 문구를 직접 입력해야 한다)
목표 목록(시작값 S 확정 뒤 고정): S, 160, 140, 120, 100, 120, 140, 160, S (쓰기 8회). 첫 이동 3000 ms, 이후 1500 ms.
각 점에서 정지 뒤 폰 사진 1장(사진 번호 입력, 캡 끝면 가운데·둥근 끝 맨 위 점·기준 표시·쇠자). 이동 중에는 측정하지 않는다. 중단 시 운영자가 y일 때만 S로 복귀 1회(폭에 맞는 승인된 시간).
"""
import argparse, datetime, json, signal, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gripper_adapter import GripperTestAdapter, GripperTestError
from g01_5a_gripper_test import FakeServo, Abort, Hangup, _on_signal, poll, settle_ms
import builtins


def flush_stdin():
    """tty에 남은 입력(정지 중 친 두 번째 입력 등)을 버린다: 사진 번호·y/N 질문이 한 칸씩 밀리지 않게(R16 사고 수정과 같은 방식)."""
    try:
        import termios; termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
    except Exception: pass


def input(p=""): flush_stdin(); return builtins.input(p)   # 이 모듈의 모든 질문 직전에 입력 버퍼를 비운다(P2g 대비 변경)


def ask(msg, default=False):
    ans = input(msg + (" [Y/n] " if default else " [y/N] ")).strip().lower()
    return default if ans == "" else ans in ("y", "yes")

MID = [160, 140, 120, 100]                      # S에서 열림 쪽(각도 감소) 계단, 그 뒤 같은 계단으로 올라온다
WINDOW = (95, 179)                                   # 승인 제안(문서 §2): 창을 지정하지 않으면 어댑터 기본값이 0~180 전체다
MOVE_MS_FIRST, MOVE_MS = 3000, 1500                  # 첫 이동 3000 ms, 이후 1500 ms
ALLOWED_MS = [1500, 3000, 4800]                      # 4800 ms는 중단 뒤 복귀 전용(최대 폭 79° × 60 ms/° = 4740 ms 이상 필요)
MAX_WRITES = 12                                      # 이동 10 + 재전송 한도 2(복귀가 필요하면 이동 수가 줄어 한도 안에 든다)
PHRASE = "APPROVE P2J REAL"
QUIET_S = 0.3                                       # 쓰기 직전 읽기 중지 시간(프로젝트 규칙: 읽은 직후의 쓰기는 버려진다). 사람 확인 질문 뒤에도 코드로 보장한다
GAP_SPREAD_OK = 1.0                                  # 각 점 3회 판독 최대−최소 합격 기준(제안, 문서 §6)
REVERSE_TOL = 1.0                                    # 예상 방향과 반대로 이만큼 넘게 변하면 중단(문서 §4)


def build_targets(s):
    return [s] + MID + MID[-2::-1] + [s]


def times_for(targets):
    return [MOVE_MS_FIRST] + [MOVE_MS] * (len(targets) - 2)


def parse_caliper(text):
    """'49.5, 49.6, 49.4' 같은 3개 숫자 → (값 목록, 평균, 범위). 빈 입력이면 None."""
    text = text.strip()
    if not text: return None
    vals = [float(x) for x in text.replace(";", ",").replace(" ", ",").split(",") if x.strip()]
    if len(vals) != 3: raise ValueError("3개 값(쉼표로 구분)이 필요하다")
    return vals, round(sum(vals) / 3, 2), round(max(vals) - min(vals), 2)


def return_time(dist_deg):
    """중단 뒤 복귀: 60 ms/° 이상을 만족하는 승인된 시간 중 가장 짧은 값. 없으면 None."""
    need = 60 * dist_deg
    ok = [t for t in ALLOWED_MS if t >= need]
    return min(ok) if ok else None


class FastFake(FakeServo):
    """시험 단축용 가짜 서보: 이동 시간을 speed로 나눠 빠르게 움직인다(하드웨어 시험에는 쓰지 않는다)."""
    def __init__(self, v, speed): super().__init__(v); self.speed = speed
    def Arm_serial_servo_write(self, id, angle, time_ms): super().Arm_serial_servo_write(id, angle, max(1, time_ms // self.speed))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--fake", action="store_true"); ap.add_argument("--real", action="store_true")
    ap.add_argument("--fake-dry", action="store_true", help="--fake에서만: dry_run 흐름(쓰기 0회)을 가짜 서보로 시험한다")
    ap.add_argument("--fake-speed", type=int, default=1, help="--fake에서만: 이동·정착 대기를 이 배수로 줄인다(시험 단축)")
    ap.add_argument("--fake-start", type=int, default=179); ap.add_argument("--fake-nomove-at", type=int, default=None, help="--fake에서만: 이 번호의 이동(1부터)을 무시하는 서보(쓰기 누락 모사)")
    ap.add_argument("--run-id", default="local"); ap.add_argument("--out", default="/tmp/g01_p2j_result.json"); a = ap.parse_args()
    if a.fake and a.real: sys.exit("--fake와 --real은 함께 쓸 수 없다")
    if a.fake_speed != 1 and not a.fake: sys.exit("--fake-speed는 --fake에서만 쓸 수 있다")
    if a.fake_dry and not a.fake: sys.exit("--fake-dry는 --fake에서만 쓸 수 있다")
    mode = "dry_run" if (a.fake and a.fake_dry) else ("fake" if a.fake else ("real" if a.real else "dry_run"))
    result = {"run_id": a.run_id, "mode": mode, "started_at": datetime.datetime.now().isoformat(timespec="seconds"), "window": list(WINDOW), "allowed_times_ms": ALLOWED_MS, "max_writes": MAX_WRITES, "steps": [], "interrupted": False, "ended": None, "layout_note": None}
    ad = None; writes_done = False; cur_angle = None
    for sg in (signal.SIGHUP, signal.SIGTERM): signal.signal(sg, _on_signal)
    try:
        print("G07 P2j K1 직립 J6별 캡 끝 위치 측정(%s): 그리퍼 서보 6만 움직인다. 팔 관절은 명령하지 않는다. 물체 없음." % mode)
        print("사전 확인:")
        for k, q in enumerate(("전원 차단 담당자가 현재 자리에 있다", "팔이 K1 기립(대기) 자세이고 낙하 받침이 준비돼 있다(전원 차단 시 직립 팔이 낙하할 수 있음)", "손가락 사이에 물체·손·케이블이 없다", "폰·쇠자·스티커가 SVG 안내대로 놓였다: 스티커는 Arm5 서보 상자 앞면(J6와 무관한 기준점), 쇠자는 오른쪽 열림 범위 밖 +80 mm 고정이며 손가락과 같은 깊이, 폰은 arm5 z축 방향에서 본다(손은 점 사이 이동 전에 뗀다)", "영상 모드 시험 녹화로 눈금·손끝·초점이 보임을 확인했다"), 1): print("  %d. %s" % (k, q))
        if not ask("위 5개를 모두 확인했습니까? (질문은 이 한 번으로 묶었습니다)"): raise Abort("사전 확인 미충족. 명령은 보내지 않았다.")
        if mode != "dry_run":   # dry run은 사진·쓰기가 없어 아래 입력을 받지 않는다(입력 최소화)
            result["layout_note"] = input("측정 배치 한 줄 설명(예: 폰 위치·쇠자·기준 스티커 위치, 엔터=건너뜀): ").strip() or None
        if a.real:
            if input("실제 쓰기를 시작하려면 정확히 '%s'를 입력: " % PHRASE).strip() != PHRASE: raise Abort("승인 문구 불일치. 명령은 보내지 않았다.")
        if a.fake:
            arm = FastFake(a.fake_start, a.fake_speed)
            if a.fake_nomove_at:
                orig = arm.Arm_serial_servo_write; cnt = {"n": 0}
                def flaky(id, angle, time_ms, _o=orig, _c=cnt):
                    _c["n"] += 1
                    if _c["n"] == a.fake_nomove_at: arm.writes.append((id, angle, time_ms)); return   # 쓰기 누락 모사
                    return _o(id, angle, time_ms)
                arm.Arm_serial_servo_write = flaky
        else:
            from Arm_Lib import Arm_Device; arm = Arm_Device()   # 생성자는 버스를 열기만 한다(소스 확인)
        today = datetime.date.today()
        ad = GripperTestAdapter(arm, {"approved": True, "expires": today.isoformat(), "allowed_times_ms": ALLOWED_MS, "first_move_min_ms": MOVE_MS_FIRST, "max_writes": MAX_WRITES, "delta_deg": 5, "joint_id": 6, "direction": -1, "angle_window": list(WINDOW), "pattern": "list"}, today=today, dry_run=(mode == "dry_run"))
        s = ad.read_start(); cur_angle = s
        if s not in (178, 179): raise Abort("시작 회독 S=%d가 179(±1)이 아니다. 다른 상태에서는 시작하지 않는다. 명령은 보내지 않았다." % s)
        targets = build_targets(s); tms = times_for(targets); result["targets"] = targets; result["times_ms"] = tms
        print("시작 회독값 S = %d. 절대 목표 목록 = %s (첫 값은 현재 위치, 쓰기는 두 번째부터, 쓰기 %d회)" % (s, targets, len(targets) - 1))
        if mode != "dry_run" and not ask("이 목록을 확인했고 이대로 진행합니까?"): raise Abort("목표 목록 미확인. 쓰기 없음.")
        ad.set_targets(targets)
        def measure(label, angle):
            """점마다 폰 사진 1장을 찍은 뒤 사진 번호를 입력(엔터=건너뜀). 사진은 사전 등록 판독 규칙으로만 읽는다."""
            if mode == "dry_run": return None
            ph = input("  %s(J6 %s): 폰 사진을 찍고 사진 번호/파일명 입력(엔터=건너뜀): " % (label, angle)).strip()
            return {"photo": ph, "epoch": round(time.time(), 1)} if ph else None   # epoch: 폰 사진 EXIF 시각과 J6 점의 대응을 독립 확인하기 위한 기록
        result["steps"].append({"point": "start", "angle": s, "photo": measure("시작", s)}); prev_angle = s
        for i, (tgt, t_ms) in enumerate(zip(targets[1:], tms), 1):
            if mode != "dry_run" and not ask("손을 뗐고 쇠자는 열림 범위 밖 고정 자리, 스티커는 상자 앞면에 그대로 있습니까? 다음 이동 %d/%d: %d (time %d ms)" % (i, len(targets) - 1, tgt, t_ms)): raise Abort("손 제거 미확인. 추가 명령 없음.")
            ad.confirm_hands_clear(); time.sleep(QUIET_S); ad.move(tgt, t_ms); writes_done = writes_done or mode != "dry_run"; before = cur_angle; cur_angle = tgt
            if mode == "dry_run":
                print("  dry_run: 이동 없음(쓰기 0회, 도착 검사 생략)"); result["steps"].append({"point": "dry_run_move", "angle": tgt, "time_ms": t_ms}); continue
            wait = (2 * t_ms + 1000) // (a.fake_speed if a.fake else 1)
            series = poll(ad, wait); st = settle_ms(series); last = next((v for _, v in reversed(series) if v is not None), None)
            step = {"point": "move", "n": i, "angle": tgt, "time_ms": t_ms, "settle_ms": st, "final_readback": last, "series": series}; result["steps"].append(step)
            print("  -> 이동 %d, time %d ms: 정착 %s ms, 최종 회독 %s" % (tgt, t_ms, st, last))
            if last is None or abs(last - tgt) > 3:
                if last is not None and abs(last - before) <= 1 and abs(tgt - before) >= 10 and ask("  서보가 움직이지 않았습니다(쓰기 누락 의심). 손이 없고 쇠자가 열림 범위 밖임을 확인했고 같은 목표를 한 번만 다시 보낼까요?"):
                    ad.confirm_hands_clear(); time.sleep(QUIET_S); ad.repeat_last(t_ms); step["repeated"] = True
                    series = poll(ad, wait); st = settle_ms(series); last = next((v for _, v in reversed(series) if v is not None), None); step.update({"settle_ms": st, "final_readback": last, "series": series})
                if last is None or abs(last - tgt) > 3:
                    step["arrival_failed"] = True; raise Abort("도착 실패(회독 %s, 목표 %d, 허용 3°). 서보는 마지막 목표를 유지 중일 수 있다. 추가 이동 없음." % (last, tgt))
            if abs(last - tgt) >= 2: print("  ! 경고: 회독 잔차가 %d°입니다. 소리·떨림·발열을 확인하세요." % abs(last - tgt))
            step["photo"] = measure("이동 %d/%d 뒤" % (i, len(targets) - 1), tgt)
            prev_angle = tgt
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
        if ad is not None and writes_done and result["ended"] != "completed":
            try:
                s0 = result["targets"][0]; dist = abs((cur_angle if cur_angle is not None else s0) - s0); t_ret = return_time(dist)
                if t_ret is None: print("복귀 폭 %d°가 승인된 시간으로 갈 수 없다. 복귀 명령을 보내지 않는다." % dist); result["returned_to_start"] = False
                elif ask("그리퍼가 시작값 S=%d가 아닐 수 있습니다(마지막 명령 %s). S로 복귀 쓰기를 1회 보낼까요? 시간 %d ms (손을 뗀 뒤에만 y, 쇠자는 열림 범위 밖에 둔다)" % (s0, cur_angle, t_ret)):
                    ad.confirm_hands_clear(); time.sleep(QUIET_S); ad.return_to_start(t_ret); result["returned_to_start"] = True; result["return_time_ms"] = t_ret; print("복귀 명령 1회 전송")
                else: result["returned_to_start"] = False
            except BaseException as e: result["return_error"] = repr(e); print("복귀 처리 중단:", repr(e))
        result["adapter_log"] = [(["write", *l[1:3], "fake" if (a.fake and not a.fake_dry and l[0] == "write") else l[3], *l[4:]] if l[0] == "write" else list(l)) for l in (ad.log if ad else [])]
        result["finished_at"] = datetime.datetime.now().isoformat(timespec="seconds")
        Path(a.out).write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"); print("기록 저장:", a.out)


if __name__ == "__main__": main()
