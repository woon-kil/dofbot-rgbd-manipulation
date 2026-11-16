#!/usr/bin/env python3
"""G01.5b-4 S4-2: 계획 파일(`g01_s42_plan.py`가 Mac에서 만든 이동 목록)에 따라 한 번에 한 관절씩 단일 쓰기로 접기 계단 → 파일럿 자세들 → 복귀 경로를 실행하고, 자세마다 DaBai 노출 스윕을 촬영한다.
쓰기는 서보 id 1~4에 개별로만 보낸다(write6·동시 이동 없음). 사람이 입회해 직접 실행(무인 실행 금지). 계획의 FK 안전 게이트는 Mac 계획 단계에서 이미 통과했다(이 도구는 FK를 모른다).
사용: python3 g01_5b4_s42_test.py --plan plan.json [--real] [--fake ...]   (기본 dry_run: 읽기만, 쓰기 0회)
자동 중단: 쓰기 후 3초 무반응(+육안 질문), 도착 실패 ±3°, 도달 0.5배 하한, 정착 뒤 1초 변화 2°, None 3연속, 명령하지 않은 관절·J5·J6가 현재 명령값에서 3° 이탈. 중단 후 복귀는 사용자 y일 때만(계획의 상태별 복귀 경로).
"""
import argparse, builtins, datetime, json, signal, subprocess, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gripper_adapter import GripperTestAdapter, GripperTestError
from g01_5b4_joint_test import FakeArm, Hangup, _on_signal, read_valid, last_valid

POLL_MS = 100; MAX_CONSEC_NONE = 3; TOL = 3; DRIFT_LIMIT = 2; HOLD_TOL = 3; REACT_WAIT_MS = 3000; FAST_RATIO = 0.5; START_TOL = 1
WINDOWS = {1: [70, 110], 2: [70, 110], 3: [10, 105], 4: [10, 105]}
JOINT_DESC = {1: "J1(밑동 회전: 팔 전체가 좌우로 돈다)", 2: "J2(어깨 피치: 가장 밑의 앞뒤 기울임 관절, 하중이 가장 큼)", 3: "J3(팔꿈치: 위팔과 아래팔 사이 피치 관절)", 4: "J4(손목 피치: 그리퍼·DaBai가 달린 마지막 링크를 앞뒤로 꺾는 관절)"}
CAP_CMD = 'source /opt/ros/humble/setup.bash; source ~/dofbot_pro_ws/install/setup.bash; mkdir -p /tmp/g01_s42_view; rm -rf /tmp/g01_s42_view/p{pid}; python3 ~/dofbot-g01_5b4/g01_dabai_expsweep.py /tmp/g01_s42_view/p{pid} {vals} dabai > /tmp/g01_s42_view/p{pid}.log 2>&1; echo capture_exit=$?'


def say(*a, **k): builtins.print(*a, **k, flush=True)
def inp(p): return builtins.input("\a" + p)
def ask(msg): return inp(msg + " [y/N] ").strip().lower() in ("y", "yes")
class Abort(Exception): pass

QUIET_S = 0.3   # 쓰기 직전 조용한 시간(초). 2026-11-12 스윕: 0초 8/8 누락, 0.03·0.1·0.3초 각 0/8 → 사전 고정 규칙으로 0.3
RETRIES = {"n": 0}   # 쓰기 재전송 누적(한 실행에서 2회 넘으면 중단)
MAX_RETRIES = 2
PROG = {"sent": 0, "return_started": False}   # sent: 계획 이동 중 쓰기를 실제로 보낸 마지막 이동 번호(복귀 상태 결정에 사용)
FAKE = {}   # 가짜 시험용 훅


class Reader:   # 쓰기 없는 관절용 읽기 전용 래퍼
    def __init__(self, arm, j): self.arm = arm; self.j = j
    def read_position(self): return self.arm.Arm_serial_servo_read(self.j)


class FakeArmS(FakeArm):
    def __init__(self, pos, ignore_ids=(), floors=None, drop_nth=None, read_quiet=0.0, hyst=None):
        super().__init__(pos); self.hyst = hyst; self.ignore = set(ignore_ids); self.floors = dict(floors or {}); self.nw = 0; self.drop_nth = drop_nth
        self.read_quiet = read_quiet; self.last_read = -1e9; self.quiet_drops = 0   # 실측 기제 모사: 마지막 읽기 뒤 read_quiet초 안에 온 쓰기는 버려진다
    def Arm_serial_servo_read(self, id):
        self.last_read = time.monotonic(); return super().Arm_serial_servo_read(id)
    def raw_servo_read(self, id):   # 가짜: Arm_Lib 환산의 역(원시 900~3100 = 0~180°, J2~J4는 반전)을 연속 위치에 적용
        self.last_read = time.monotonic(); a = 180.0 - self._p(id) if id in (2, 3, 4) else self._p(id); return int(round(900 + a * 2200.0 / 180.0))
    def Arm_serial_servo_write(self, id, angle, time_ms):
        self.nw += 1
        if self.read_quiet and time.monotonic() - self.last_read < self.read_quiet: self.quiet_drops += 1; self.writes.append((id, angle, time_ms)); return
        if self.drop_nth is not None and self.nw == self.drop_nth: self.writes.append((id, angle, time_ms)); return   # 가짜: 이 쓰기 한 번만 누락
        if id in self.ignore: self.writes.append((id, angle, time_ms)); return
        tgt = max(angle, self.floors[id]) if id in self.floors and angle < self.floors[id] else angle
        if self.hyst and id in (2, 3, 4):   # 가짜: 방향별 정지 오차(관찰 모사). 증가 방향은 d_up만큼 모자라게 멈추고(요청이 d_up 이하이면 움직이지 않음), 감소 방향은 o_down만큼 넘어간다
            cur = float(self._p(id)); h = list(self.hyst) + [0.0] * (6 - len(self.hyst)); nz = (h[5] if (h[5] and abs(tgt - cur) < 13) else h[3]); rng = self.__dict__.setdefault("_rng", __import__("random").Random(12345)); last = self.__dict__.setdefault("_lastdir", {})   # h = d_up, o_down, 크기 기울기 k_up, 잡음 SD, 직전이 아래였을 때 d_up 추가량(0이면 직전 방향 무관)
            if tgt > cur: tgt = max(cur, tgt - (h[0] + h[2] * (tgt - cur) + (h[4] if last.get(id) == -1 else 0.0))) + (rng.gauss(0.0, nz) if nz else 0.0); last[id] = 1
            elif tgt < cur: tgt = tgt - h[1] + (rng.gauss(0.0, nz) if nz else 0.0); last[id] = -1
        super().Arm_serial_servo_write(id, tgt, time_ms); self.writes[-1] = (id, angle, time_ms)


def watch(ads, arm, j, total_ms, before, need_react=True, retry_fn=None, tgt=None, ms=None):
    """쓰기 대상 관절 j와 J1~J4 전부를 total_ms 동안 읽는다. 반환: (시계열, 반응 여부, 경고 여부)"""
    ids = (1, 2, 3, 4); ser = {k: [] for k in ids}; cons = {k: 0 for k in ids}; t0 = time.monotonic(); react = not need_react; warned = False; retried = False
    while (time.monotonic() - t0) * 1000.0 < total_ms:
        t = round((time.monotonic() - t0) * 1000.0)
        for k in ids:
            v = ads[k].read_position(); ser[k].append((t, v)); cons[k] = cons[k] + 1 if v is None else 0
            if cons[k] >= MAX_CONSEC_NONE: raise Abort("J%d 회독 None이 연속 %d회. 추가 이동 없음." % (k, cons[k]))
            if k == j and v is not None and before is not None and v != before: react = True
        if not react and retry_fn is not None and not retried and t >= REACT_WAIT_MS:
            RETRIES["n"] += 1
            if RETRIES["n"] > MAX_RETRIES: raise Abort("쓰기 재전송이 %d회째다. 쓰기 누락이 반복되어 중단합니다(하드웨어·배선·전원 점검 필요). 추가 쓰기 없음." % RETRIES["n"])
            say("\n! 쓰기 후 %d ms 동안 J%d 회독이 변하지 않았다(쓰기 누락 의심). **같은 목표를 한 번만 다시 보냅니다**(재전송 %d/%d). 계속 읽습니다..." % (REACT_WAIT_MS, j, RETRIES["n"], MAX_RETRIES))
            retry_fn(); retried = True
        if not react and not warned and t >= (2 * REACT_WAIT_MS if retried else REACT_WAIT_MS):
            warned = True; say("\n! 쓰기 후 %d ms 동안 J%d 회독이 변하지 않았다(회독 기준 미응답). 추가 쓰기는 없다. **명령은 이미 보냈으므로 약 10초 동안 팔에 다가가지 마세요.** 계속 읽습니다..." % (REACT_WAIT_MS, j))
        if tgt is not None and react and t >= 1.1 * ms + 1000:   # 조기 종료: 명령 시간의 1.1배가 지나고 최근 1초 회독이 모두 목표 ±TOL 안에서 안정이면 더 기다리지 않는다(아니면 기존 상한 1.5배+1초까지)
            w_ = [v for t_, v in ser[j] if t_ >= t - 1000]
            if w_ and all(v is not None and abs(v - tgt) <= TOL for v in w_) and max(w_) - min(w_) < DRIFT_LIMIT: break
        time.sleep(POLL_MS / 1000.0)
    return ser, react, warned, retried


def run_move(ads, arm, mv, cmd, start_all, rec, n, total, mode, countdown, label_extra=""):
    """이동 1개 실행·검사. cmd는 현재 명령값(J1~J4)을 갱신한다. 반환: 기록 dict"""
    j = mv["joint"]; tgt = mv["target"]; ms = mv["time_ms"]
    desc = JOINT_DESC[j]
    say("\n[이동 %d/%d] J%d -> %d (time %d ms)%s: %d초 뒤 이동합니다. ▶ **J%d를 지켜보세요.** 이상하면 Ctrl-C.%s" % (n, total, j, tgt, ms, (" [자세 %s]" % mv["pose_id"]) if mv.get("pose_id") is not None else "", countdown, j, (" ※ " + mv["note"]) if mv.get("note") else ""))
    if FAKE.get("countdown_int") == n and mv.get("phase") in ("A", "B", "T"): raise KeyboardInterrupt()   # 가짜: 카운트다운 중 Ctrl-C
    time.sleep(countdown)
    step = {"n": n, "joint": j, "target": tgt, "time_ms": ms, "phase": mv.get("phase"), "role": mv.get("role"), "pose_id": mv.get("pose_id"), "cmd_before": dict(cmd)}
    if mode == "dry_run": step["point"] = "dry_run_move"; say("  dry_run: 쓰기 없음"); cmd[j] = tgt; return step
    before = ads[j].read_position()
    time.sleep(QUIET_S)   # 읽기와 쓰기 사이 조용한 시간(이 동안 어떤 읽기도 하지 않는다)
    ads[j].confirm_hands_clear(); ads[j].move(tgt, ms)
    if mv.get("phase") in ("A", "B", "T"): PROG["sent"] = n   # 탐침(T) 이동도 쓰기 직전에 기록한다(세트 18 중단 복귀 가짜 시험에서 발견: 이전에는 마지막 A/B 이동 번호가 남아 낡은 복귀 상태를 골랐다)
    width0 = abs(tgt - (before if before is not None else tgt))
    def _retry(): time.sleep(QUIET_S); ads[j].confirm_hands_clear(); ads[j].repeat_last(ms)   # 재전송도 읽기를 멈추고 조용한 시간 뒤에 보낸다
    ser, react, warned, retried = watch(ads, arm, j, int(1.5 * ms) + 1000, before, need_react=(width0 >= 5), retry_fn=_retry, tgt=tgt, ms=ms)   # 1~4° 소폭 이동은 데드밴드로 회독이 안 변할 수 있어 반응 필수 검사에서 제외
    step["point"] = "move"; step["series"] = {str(k): v for k, v in ser.items()}; step["readback_before"] = before; step["retried"] = retried
    if warned:
        step["visual_after_unresponsive"] = inp("진단용 질문 1개: 지금까지 J%d가 눈으로 보기에 움직였습니까? [움직임/안 움직임/모름]: " % j).strip()
        if not react: cmd[j] = tgt; raise Abort("J%d 쓰기 후 무반응(회독 기준). 육안 답: %s. 추가 쓰기 없음." % (j, step["visual_after_unresponsive"] or "없음"))
    fin = {k: last_valid(v) for k, v in ser.items()}; step["final"] = {str(k): v for k, v in fin.items()}
    cmd_old = dict(cmd); cmd[j] = tgt
    others = {k: read_valid(arm, k) for k in (5, 6)}; step["others"] = {str(k): v for k, v in others.items()}
    say("  -> 최종 회독 " + ", ".join("J%d=%s" % (k, fin[k]) for k in (1, 2, 3, 4)) + " (이번 J%d 목표 %d), J5=%s, J6=%s" % (j, tgt, others[5], others[6]))
    width = abs(tgt - (before if before is not None else tgt))
    if width > 1 and before is not None and fin[j] == before: step["no_readback_change"] = True
    reach = next((t_ for t_, v in ser[j] if v is not None and abs(v - tgt) <= TOL), None); step["reach_within_tol_ms"] = reach
    if reach is not None and width >= 10 and reach < FAST_RATIO * ms:   # 소폭 이동은 ±3° 도달이 원래 일찍 끝나므로 제외(폭 10° 이상에서 정상 도달 비율이 0.7 이상)
         raise Abort("J%d가 목표 ±%d°에 %d ms에 도달했다(명령 %d ms의 %.2f배, 하한 %.1f배). 시간 값이 적용되지 않았을 의심: 추가 이동 없음." % (j, TOL, reach, ms, reach / ms, FAST_RATIO))
    if fin[j] is None or abs(fin[j] - tgt) > TOL: step["arrival_failed"] = j; raise Abort("J%d 도착 실패(마지막 유효 회독 %s, 목표 %d, 허용 %d°). 추가 이동 없음." % (j, fin[j], tgt, TOL))
    w = [v for t_, v in ser[j] if v is not None and t_ >= ser[j][-1][0] - 1000]
    if w and max(w) - min(w) >= DRIFT_LIMIT: raise Abort("J%d 정착 뒤 1초 회독 변화 %d° (한계 %d°, 헌팅·처짐 의심). 추가 이동 없음." % (j, max(w) - min(w), DRIFT_LIMIT))
    for k in (1, 2, 3, 4):
        if k == j: continue
        if fin[k] is None or abs(fin[k] - cmd[k]) > HOLD_TOL: raise Abort("명령하지 않은 J%d 회독 %s가 현재 명령값 %d에서 %d° 넘게 벗어났다. 추가 이동 없음." % (k, fin[k], cmd[k], HOLD_TOL))
    for k in (5, 6):
        if others[k] is None or abs(others[k] - start_all[k]) > HOLD_TOL: raise Abort("J%d 회독 %s가 시작값 %d에서 %d° 넘게 벗어났다. 추가 이동 없음." % (k, others[k], start_all[k], HOLD_TOL))
    return step


def hold_for_measure(plan, pid, a, rec, arm=None):
    """사용자 실측용 대기: 계획의 hold_poses 자세에서 서보에 쓰기를 보내지 않고 멈춰 Enter(또는 최대 시간)까지 기다린다. fake에서는 건너뛴다.
    ① 대기 직전 tty에 남은 입력을 비운다(이동 중 눌린 Enter가 대기를 즉시 끝내지 않게) ② 1초마다 J2~J4 회독만 읽고 2° 이상 변하면 경고하며 대기를 끝낸다 ③ 남은 시간을 30초마다 표시한다."""
    if a.fake or str(pid) not in [str(x) for x in plan.get("hold_poses", [])]: return
    import select
    try:
        import termios; termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
    except Exception: pass
    secs = int(plan.get("hold_max_s", 180)); t0 = time.monotonic(); base = {k: read_valid(arm, k) for k in (2, 3, 4)} if arm is not None else {}
    if plan.get("set") == "20":   # P1-b2 (b)·#11 D1(A2): 사람이 할 일은 측면 폰 촬영과 종이 위 손끝 높이 mm 한 값 입력
        say("\n[측정 대기] 자세 C(모델 손끝 높이 44.1 mm)에서 멈춥니다(서보에 쓰기를 보내지 않고 J2~J4 회독만 읽어 처짐을 감시합니다). **팔 이동 범위 안에 손을 넣지 마세요.** 지금 측면 폰으로 손끝(캡 끝면 가운데)과 눈금자가 함께 찍히게 사진을 찍고, 종이 위에서 손끝까지의 높이를 눈금자로 읽어 mm 숫자 한 개를 쓴 뒤(예: 42) Enter를 누르세요(최대 %d초 뒤 자동으로 계속, 값을 못 읽으면 그냥 Enter)." % secs)
    else:
        say("\n[측정 대기] 자세 %s에서 멈춥니다(서보에 쓰기를 보내지 않고 J2~J4 회독만 읽어 처짐을 감시합니다). **팔 이동 범위 안에 손을 넣지 마세요.** 읽은 값을 한 줄에 쓰고(예: 74 75) Enter를 누르세요(최대 %d초 뒤 자동으로 계속)." % (pid, secs))
    ended = "timeout"; last_say = 0
    while time.monotonic() - t0 < secs:
        if select.select([sys.stdin], [], [], 1.0)[0]: rec["user_entry"] = sys.stdin.readline().strip(); ended = "enter"; break   # 사용자가 Enter 전에 쓴 측정값(예: "74 75")을 기록에 남긴다
        if arm is not None:
            cur = {k: read_valid(arm, k) for k in (2, 3, 4)}
            if any(cur[k] is None or base[k] is None or abs(cur[k] - base[k]) >= 2 for k in cur):
                ended = "drift"; rec["hold_drift"] = {"start": base, "now": cur}; say("\n! 대기 중 회독이 %s에서 %s로 변했습니다(2° 이상). 처짐 의심: 대기를 끝내고 계속합니다. 팔을 확인하세요." % (base, cur)); break
        el = time.monotonic() - t0
        if el - last_say >= 30: last_say = el; say("  측정 대기 %d초 경과(남은 %d초)." % (el, secs - el))
    rec["hold_s"] = round(time.monotonic() - t0, 1); rec["hold_ended_by"] = ended
    say("  측정 대기 종료(%s, %.0f초)." % (ended, rec["hold_s"]))


def read_raw(arm, jid):
    """서보 원시 위치 카운트(Arm_Lib `Arm_serial_servo_read`가 환산·절삭하기 전의 값)를 읽는다. 읽기 전용: 같은 I2C 읽기 호출(요청 바이트 쓰기 후 read_word_data)이며 환산·절삭만 하지 않는다. 실패하면 None(보조 기록이라 중단하지 않는다)."""
    try:
        f = getattr(arm, "raw_servo_read", None)   # 가짜 팔 전용(실제 Arm_Device에는 없다)
        if f is not None: return f(jid)
        arm.bus.write_byte_data(arm.addr, jid + 0x30, 0); time.sleep(0.003)
        w = arm.bus.read_word_data(arm.addr, jid + 0x30)
        return None if w == 0 else ((w >> 8 & 0xff) | (w << 8 & 0xff00))
    except Exception: return None


def raw_to_deg(raw, jid):
    """원시 카운트 → SDK 각도(실수, 절삭 없음). Arm_Lib 1.0.3 환산: 원시 900~3100 = 0~180°, J2~J4는 180 − 값."""
    if raw is None: return None
    a = 180.0 * (raw - 900) / 2200.0
    return round(180.0 - a if jid in (2, 3, 4) else a, 3)


def raw_set(arm):
    out = {}
    for k in (1, 2, 3, 4):
        r = read_raw(arm, k); out[str(k)] = {"raw": r, "deg": raw_to_deg(r, k)}
    return out


def raw_after_pair(arm):
    """소각 응답 시험(세트 16): 이동이 끝난 직후와 2초 뒤, J1~J4 원시 값을 각각 5회 평균(50 ms 간격)으로 읽는다. 읽기 전용. 다음 쓰기 전 조용한 시간 0.3초 규칙이 이어서 적용된다."""
    def mean5():
        acc = {k: [] for k in (1, 2, 3, 4)}
        for _ in range(5):
            for k in (1, 2, 3, 4):
                r = read_raw(arm, k)
                if r is not None: acc[k].append(r)
            time.sleep(0.05)
        return {str(k): {"raw_mean": (sum(v) / len(v) if v else None), "deg": (raw_to_deg(sum(v) / len(v), k) if v else None), "n": len(v)} for k, v in acc.items()}
    a_ = mean5(); time.sleep(2.0); b_ = mean5()
    return {"a": a_, "b": b_}


VISITS = {}   # 같은 자세를 여러 번 방문하는 계획(집합 7): 두 번째 방문부터 촬영 폴더 p<id>v<k>
def capture(plan, pid, arm, ads, a):
    k_ = VISITS.get(pid, 0); VISITS[pid] = k_ + 1; tag = str(pid) if k_ == 0 else "%sv%d" % (pid, k_)
    say("\n[촬영] 자세 %s 도착. %d초 정지 뒤 DaBai 노출 스윕(%s)을 촬영합니다(약 %d초). **팔·보드·케이블을 건드리지 마세요.**" % (pid, plan["settle_s"], ",".join(map(str, plan["exposures"])), 4 + 2 * len(plan["exposures"]))); time.sleep(plan["settle_s"])
    pre = {k: ads[k].read_position() for k in (1, 2, 3, 4)}; raw_pre = raw_set(arm)   # 정수 회독과 같은 순간의 원시 값(양자화 없는 FK 예측용)
    if a.fake:
        if FAKE.get("capture_fail"): raise Abort("DaBai 촬영 실패(가짜). 추가 이동 없음.")
        say("  (가짜) 촬영 생략"); return {"fake": True, "pre": pre, "raw_pre": raw_pre}
    t0 = time.monotonic(); rr = subprocess.run(["bash", "-lc", CAP_CMD.format(pid=tag, vals=",".join(map(str, plan["exposures"])))], capture_output=True, text=True, timeout=120)
    post = {k: ads[k].read_position() for k in (1, 2, 3, 4)}; raw_post = raw_set(arm)
    if rr.returncode != 0 or "capture_exit=0" not in rr.stdout: raise Abort("DaBai 촬영 실패(종료 코드 %s, 출력 %s). 추가 이동 없음." % (rr.returncode, rr.stdout.strip()[-80:]))
    res = {"view_tag": tag, "returncode": rr.returncode, "stdout_tail": rr.stdout[-120:], "seconds": round(time.monotonic() - t0, 1), "pre": pre, "post": post, "raw_pre": raw_pre, "raw_post": raw_post}
    say("  촬영 종료(%s, %.1f초). 자세 촬영 전 %s, 후 %s" % (rr.stdout.strip().splitlines()[-1] if rr.stdout.strip() else "출력 없음", res["seconds"], pre, post)); return res


def make_adapters(arm, lists, times, mode, first_min):
    today = datetime.date.today(); ads = {}
    for j in (1, 2, 3, 4):
        if len(lists[j]) < 2: ads[j] = Reader(arm, j); continue
        ads[j] = GripperTestAdapter(arm, {"approved": True, "expires": today.isoformat(), "allowed_times_ms": sorted(set(times)), "first_move_min_ms": first_min, "max_writes": len(lists[j]) + 2, "delta_deg": 1, "joint_id": j, "direction": 1, "angle_window": WINDOWS[j], "pattern": "list"}, today=today, dry_run=(mode == "dry_run"))
        s = ads[j].read_start(); lists[j][0] = s; ads[j].set_targets(lists[j])
    return ads


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--plan", required=True); ap.add_argument("--fake", action="store_true"); ap.add_argument("--real", action="store_true")
    ap.add_argument("--fake-ignore", type=int, nargs="*", default=[]); ap.add_argument("--fake-floor", type=str, default="", help="예: 2:95 (서보:바닥값)")
    ap.add_argument("--fake-drop-nth", type=int, default=None); ap.add_argument("--fake-hyst", type=str, default="", help="예: 1.5,0.3 (증가 방향 모자람°, 감소 방향 넘어감°)"); ap.add_argument("--fake-countdown-int", type=int, default=None); ap.add_argument("--fake-return-int", type=int, default=None); ap.add_argument("--fake-capture-fail", action="store_true"); ap.add_argument("--fake-read-quiet", type=float, default=0.0, help="가짜: 마지막 읽기 뒤 이 초 안의 쓰기를 버린다(실측 기제 모사)"); ap.add_argument("--run-id", default="local"); ap.add_argument("--out", default="/tmp/g01_5b4_s42_result.json"); ap.add_argument("--stop-after", type=int, default=None, help="가짜 시험: 이동 N번 뒤 사용자 중단을 흉내")
    a = ap.parse_args()
    if a.fake and a.real: sys.exit("--fake와 --real은 함께 쓸 수 없다")
    FAKE.update({"countdown_int": a.fake_countdown_int, "return_int": a.fake_return_int, "capture_fail": a.fake_capture_fail})
    plan = json.load(open(a.plan, encoding="utf-8"))
    if plan["set"] in ("18", "19"): WINDOWS[2] = [70, 140]   # T1-B(큰 이동 J2 90→135): 어댑터 목표 창을 이 세트에서만 J2 70~140으로 넓힌다(새 안전 한도, 사용자 승인 필요: docs/gates/G07-소각응답T1B-큰이동-승인-요청-문서.md D6). 다른 세트는 그대로 70~110
    mode = "fake" if a.fake else ("real" if a.real else "dry_run")
    moves = plan["moves"]; N = len(moves)
    result = {"run_id": a.run_id, "mode": mode, "kind": "s42_" + plan["set"], "plan_set": plan["set"], "plan_gate": plan["gate"], "started_at": datetime.datetime.now().isoformat(timespec="seconds"), "state0": None, "steps": [], "poses": [], "ended": None, "return_steps": []}
    ads = {}; arm = None; wrote = False; cmd = {}; done_moves = 0; start_all = {}
    for sg in (signal.SIGHUP, signal.SIGTERM): signal.signal(sg, _on_signal)
    try:
        say("G01.5b-4 S4-2%s 자세 취득(모드 %s): 한 번에 한 관절씩 단일 쓰기. 접기 계단 8 + 자세 이동 %d회, 파일럿 자세 %d개. 쓰기 서보 1~4, J5·J6는 읽기만. 물체 없음." % (plan["set"], mode, N - 8, len(plan["poses"])))
        floors = {}
        for tok in a.fake_floor.split(","):
            if ":" in tok: k, v = tok.split(":"); floors[int(k)] = int(v)
        ps = plan["start"]
        arm = FakeArmS({int(k): v for k, v in ps.items()}, a.fake_ignore, floors, a.fake_drop_nth, a.fake_read_quiet, tuple(float(x) for x in a.fake_hyst.split(",")) if a.fake_hyst else None) if a.fake else __import__("Arm_Lib").Arm_Device()
        state = {i: read_valid(arm, i) for i in range(1, 7)}; result["state0"] = state; start_all = dict(state)
        say("b-4-0 현재 회독(SDK 각도): " + ", ".join("J%d=%s" % (i, v) for i, v in state.items()))
        if any(state[i] is None for i in range(1, 7)): raise Abort("회독 None이 있다. 쓰기 없음.")
        for k in range(1, 7):
            if abs(state[k] - ps[str(k)]) > START_TOL: raise Abort("J%d 현재 회독 %s가 계획 시작값 %s와 %d° 넘게 다르다. 계획을 다시 만든 뒤 실행하세요. 쓰기 없음." % (k, state[k], ps[str(k)], START_TOL))
        # 이동 목록 → 관절별 목표 목록(첫 항목은 실제 시작값)
        lists = {j: [state[j]] + [m["target"] for m in moves if m["joint"] == j] for j in (1, 2, 3, 4)}
        times = [m["time_ms"] for m in moves] + [3000, 1500, 8000]
        ads = make_adapters(arm, lists, times, mode, 8000)
        cmd = {j: state[j] for j in (1, 2, 3, 4)}
        say("\n계획 요약: 접기 계단 후 기준 깊은 자세 J1~J4 = %s (모델 높이 %.1f mm), 자세 순서 %s" % (plan["base"], plan["gate"]["base_z"], list(plan["poses"].keys())))
        say("자세별 모델 높이(mm) %s, 전이 최저 %s, 복귀(첫 구간) 최저 %.1f mm" % (plan["gate"]["pose_z"], plan["gate"]["transition_min"], plan["gate"]["return_j12_min"]))
        say("\n== 안전 확인(시작 시 1회) ==")
        qs = ["전원 차단 담당자가 현재 자리에 있습니까?", "낙하 받침을 준비했고, 접힌 팔(앞쪽, 보드 위)의 낙하를 받치되 팔 경로(좌우 약 ±40 mm, TCP y 약 -15~220 mm, 높이 약 70~460 mm) 안에 들어오지 않습니까?", "팔이 접히고 좌우로 도는 동안 닿을 물체·손·사람·케이블이 팔 주변과 작업대·보드 위에 없습니까? 보드 종이가 움직이지 않았습니까?", "팔이 직립(대기) 자세입니까? J4(손목)가 아래팔과 일직선입니까?", "DaBai·그리퍼 케이블이 당겨지지 않고 여유가 있어 보입니까?", "J2·J3·J4를 만져서 **뜨겁지 않습니까?** (온도 읽기 API가 없어 손으로 확인)", "위 회독값과 이동 계획이 맞다."]
        if plan["set"] in ("12", "14", "15", "16", "17", "18", "19", "20"):   # P1·큰/근거리 M3: 블록·마커가 지정 자리에 놓여 있고, 그리퍼는 오므린 채 J6를 유지한다(S4용 "작업대 위에 물체 없음" 질문은 맞지 않는다)
            qs = ["전원 차단 담당자가 현재 자리에 있습니까?", "노란 블록(30×60×30 mm, 윗면 마커 id 7)이 지정 자리(중심 x +1.4, y 약 189 mm, 앞 모서리 y 약 219 mm)에 있고 움직이지 않았습니까? 블록과 베이스 판 사이·팔 주변에 그 밖의 물체·손·사람·케이블이 없습니까?",
                  "낙하 받침을 준비했고 블록(x ±15, y 160~220 mm)과 팔 경로에 겹치지 않습니까? (받침은 사용자가 직접 놓은 자리이며 Claude가 낙하 영역을 계산해 제안한 것이 아니다)", "팔이 K1 기립 자세이고 그리퍼는 오므려져 있습니까(J6 회독 179)? 이번 실행은 J6를 건드리지 않고 179를 유지합니다.",
                  "DaBai·그리퍼 케이블이 당겨지지 않고 여유가 있어 보입니까?", "J2·J3·J4를 만져서 **뜨겁지 않습니까?** (온도 읽기 API가 없어 손으로 확인)", "위 회독값과 이동 계획이 맞다."]
        if plan["set"] in ("14", "15", "16", "17", "18", "19", "20"): qs = [q.replace("J2·J3·J4를 만져서", "J2·J3·J4와 오므린 그리퍼 서보 J6를 만져서") for q in qs]   # 큰 이동 M3: J6 발열 확인 포함(P1에서 비어 있던 항목)
        if plan["set"] in ("18", "19"): qs.insert(-1, "이번 실행은 J2(어깨)와 J3(팔꿈치)가 뒤쪽(SDK 증가 방향)으로 각각 최대 45° 기울어집니다(J2 최대 135, J3 최대 67). 충돌 계산에는 팔 뒤쪽·위쪽의 물체가 없으므로 사람이 확인합니다: 팔 뒤쪽(약 300 mm)과 위쪽에 Jetson 보드·케이블·카메라 거치대·벽·손이 없고, 케이블이 뒤로 젖혀져도 당겨지지 않습니까?")   # D6: 새 영역 J2 110 초과(이 프로젝트에서 처음), 충돌 모델 밖의 뒤쪽 물체 확인
        if plan["set"] == "20": qs.insert(-1, "이번 실행은 P1-b2 (b): 자세 C(J2 82, J3 10, J4 11, 모델 손끝 높이 44.1 mm)까지만 가고 C2는 가지 않습니다. 이 높이(지금까지 최저 57.1 mm보다 약 13 mm 아래)는 처음 가는 영역입니다. 자세 C에서 멈추는 동안(최대 180초) 측면 폰과 눈금자로 **종이 위 손끝 높이** 사진을 찍습니다. 폰과 눈금자는 팔 이동 범위와 낙하 받침 밖에 설치돼 있고, 사진을 찍을 때 손이 팔 이동 범위 안에 들어가지 않습니까? (측정값은 Enter 전에 한 줄로 쓰면 기록에 남습니다)")   # 결정 1 (b)·#11 D1(A2) 추가 항목, 사용자 승인 2026-11-16
        for k, q in enumerate(qs, 1): say("  %d. %s" % (k, q))
        if not ask("위 %d개를 모두 확인했습니까? (y면 이동 시작, 이동마다 소리 뒤 대기 후 움직입니다. 자세 도착마다 촬영 중 가만히 계세요)" % len(qs)): raise Abort("사전 확인 미충족. 쓰기 없음.")
        say("\n▶ **지켜볼 관절은 이동 안내마다 바뀝니다**(J1 좌우 회전, J2 어깨, J3 팔꿈치, J4 손목). 소리가 나면 안내된 관절을 보세요. 접힌 자세에서 J1·J2를 처음 움직일 때는 처짐·케이블을 특히 봐 주세요.")
        for n, mv in enumerate(moves, 1):
            countdown = 2 if (n <= 2 or mv["time_ms"] >= 8000) else 1   # 속도 상향(사용자 2026-11-13): 첫 이동·8000 ms 이동만 2초, 나머지 1초(소리 신호는 유지)
            if a.stop_after is not None and n > a.stop_after: raise KeyboardInterrupt()
            step = run_move(ads, arm, mv, cmd, start_all, result, n, N, mode, countdown); result["steps"].append(step); done_moves = n; wrote = wrote or mode != "dry_run"
            if plan.get("raw_after") and mode != "dry_run": step["raw_after"] = raw_after_pair(arm)   # 세트 16: 이동마다 원시 값 두 시점(읽기 전용)
            if mv.get("capture") and mode != "dry_run":
                pid = mv["pose_id"]; cap = capture(plan, pid, arm, ads, a)
                result["poses"].append({"pose_id": pid, "commanded": dict(cmd), "readback": {str(k): last_valid(step["series"][str(k)]) if k in (1, 2, 3, 4) else None for k in (1, 2, 3, 4)}, "target_pose": plan["poses"][str(pid)], "capture": cap})
                hold_for_measure(plan, pid, a, result["poses"][-1], arm)
        # 복귀(정상 종료 시에도 같은 경로로 직립까지)
        if mode != "dry_run":
            do_return(plan, {"ads": ads}, arm, cmd, start_all, state, result, PROG["sent"], a, mode, normal=True)
            if not result.get("returned_to_start"): raise Abort("정상 종료 뒤 직립 복귀가 완료되지 않았다(%s). 팔 상태를 확인하고 에이전트에게 알리세요." % (result.get("return_abort") or result.get("return_state_mismatch") or "원인 미상"))
        if mode != "dry_run":
            result["overall_ok"] = ask("마지막 확인 1회: 전체 이동에서 소리·떨림·냄새, 처짐·흔들림, 접촉·걸림, 케이블 당김이 없었고 **J2·J3·J4와 오므린 그리퍼 서보 J6가 뜨겁지 않았습니까?**")
            if not result["overall_ok"]: raise Abort("종합 확인에서 이상 보고(또는 명시적 y 없음). 추가 이동 없음.")
        result["ended"] = "completed"
    except Abort as e: result["ended"] = "aborted"; result["abort_reason"] = str(e); say("\n중단:", e)
    except Hangup as e: result["ended"] = "hangup"; result["abort_reason"] = str(e); say("\n중단:", e)
    except KeyboardInterrupt: result["interrupted"] = True; result["ended"] = "interrupted"; say("\n중단(Ctrl-C): 이후 명령 없음(이미 보낸 이동은 서보가 끝까지 간다)")
    except EOFError: result["ended"] = "input_closed"; say("\n중단: 입력이 끊겼다. 이후 명령 없음")
    except GripperTestError as e: result["ended"] = "adapter_refused"; result["error"] = str(e); say("\n어댑터 거부:", e)
    except Exception as e: result["ended"] = "error"; result["error"] = repr(e); say("\n오류:", repr(e))
    finally:
        if ads and wrote and result["ended"] != "completed" and not PROG["return_started"]:
            try:
                if ask("팔이 시작 자세가 아닐 수 있습니다. 계획된 상태별 복귀 경로(J1·J2 → 펴기 계단, 한 번에 한 관절, 각 3000 ms 이하)로 직립까지 복귀할까요? (손·도구를 뗀 뒤에만 y)"):
                    do_return(plan, {"ads": ads}, arm, cmd, start_all, state, result, PROG["sent"], a, mode, normal=False)
                else: result["returned_to_start"] = False
            except BaseException as e: result["return_error"] = repr(e); say("복귀 처리 중단:", repr(e))
        result["adapter_log"] = {str(j): [(["write", *l[1:3], "fake" if (a.fake and l[0] == "write") else l[3], *l[4:]] if l[0] == "write" else list(l)) for l in ads[j].log] for j in ads if hasattr(ads[j], "log")}
        result["finished_at"] = datetime.datetime.now().isoformat(timespec="seconds")
        Path(a.out).write_text(json.dumps(result, ensure_ascii=False, indent=2, default=lambda o: float(o)) + "\n", encoding="utf-8"); say("기록 저장:", a.out)


def do_return(plan, box, arm, cmd, start_all, state, result, k, a, mode, normal):
    """상태 k(완료한 이동 수, 중단 시 마지막으로 쓰기를 보낸 이동 포함)의 복귀 경로를 새 어댑터로 실행한다."""
    route = plan["return_routes"][k]["route"]; planned = plan["return_routes"][k]["state"]
    cur_now = {j: read_valid(arm, j) for j in (1, 2, 3, 4)}
    if any(cur_now[j] is None or abs(cur_now[j] - planned[j - 1]) > 3 for j in (1, 2, 3, 4)):
        result["return_state_mismatch"] = "현재 J1~J4 회독 %s가 계획 상태 %s와 3° 넘게 다르다" % ([cur_now[j] for j in (1, 2, 3, 4)], planned); result["returned_to_start"] = False
        say("\n[복귀 중단] 현재 회독 %s가 계획된 복귀 상태 %s와 달라 **쓰기 없이 멈춘다**(계산하지 않은 경로로 움직이지 않기 위해). 팔 상태를 확인하고 에이전트에게 알려 주세요(복구 경로를 새로 계산합니다)." % ([cur_now[j] for j in (1, 2, 3, 4)], planned)); return
    PROG["return_started"] = True
    say("\n[복귀] 상태 %d의 복귀 경로 %d이동 실행(%s)." % (k, len(route), "정상 종료 뒤 직립 복귀" if normal else "중단 복귀"))
    lists = {j: [0] + [m["target"] for m in route if m["joint"] == j] for j in (1, 2, 3, 4)}
    if all(len(v) == 1 for v in lists.values()): say("  이미 시작 자세다."); result["returned_to_start"] = True; return
    # 목록이 비는 관절은 제외하고 어댑터 생성
    today = datetime.date.today(); ads = {}
    for j in (1, 2, 3, 4):
        if len(lists[j]) == 1: continue
        ads[j] = GripperTestAdapter(arm, {"approved": True, "expires": today.isoformat(), "allowed_times_ms": [1500, 3000, 8000], "first_move_min_ms": None, "max_writes": len(lists[j]) + 1, "delta_deg": 1, "joint_id": j, "direction": 1, "angle_window": WINDOWS[j], "pattern": "list"}, today=today, dry_run=(mode == "dry_run"))
        s = ads[j].read_start(); lists[j][0] = s; ads[j].set_targets(lists[j])
    for j in (1, 2, 3, 4):
        if j not in ads: ads[j] = None
    full = {j: (ads[j] if ads[j] is not None else Reader(arm, j)) for j in (1, 2, 3, 4)}
    for i, m in enumerate(route, 1):
        mv = {"joint": m["joint"], "target": m["target"], "time_ms": m["time_ms"], "phase": "R", "pose_id": None}
        try:
            st = run_move(full, arm, mv, cmd, start_all, result, i, len(route), mode, 2)
        except Abort as e:
            result["return_abort"] = str(e); result["returned_to_start"] = False; say("복귀 중 중단:", e); return
        result["return_steps"].append(st)
        if FAKE.get("return_int") == i: raise KeyboardInterrupt()   # 가짜: 복귀 중 Ctrl-C
    result["returned_to_start"] = True


if __name__ == "__main__": main()
