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
from g01_5a_gripper_test import poll
from g01_5b4_joint_test import FakeArm, Hangup, _on_signal, read_valid, last_valid

POLL_MS = 100; MAX_CONSEC_NONE = 3; TOL = 3; DRIFT_LIMIT = 2; HOLD_TOL = 3; REACT_WAIT_MS = 3000; FAST_RATIO = 0.5; START_TOL = 1
FAKE_ENTRIES = []   # D0 가짜 시험 입력 큐
WINDOWS = {1: [70, 110], 2: [70, 110], 3: [10, 105], 4: [10, 105]}
JOINT_DESC = {1: "J1(밑동 회전: 팔 전체가 좌우로 돈다)", 2: "J2(어깨 피치: 가장 밑의 앞뒤 기울임 관절, 하중이 가장 큼)", 3: "J3(팔꿈치: 위팔과 아래팔 사이 피치 관절)", 4: "J4(손목 피치: 그리퍼·DaBai가 달린 마지막 링크를 앞뒤로 꺾는 관절)"}
CAP_CMD = 'source /opt/ros/humble/setup.bash; source ~/dofbot_pro_ws/install/setup.bash; mkdir -p /tmp/g01_s42_view; rm -rf /tmp/g01_s42_view/p{pid}; python3 ~/dofbot-g01_5b4/g01_dabai_expsweep.py /tmp/g01_s42_view/p{pid} {vals} dabai > /tmp/g01_s42_view/p{pid}.log 2>&1; echo capture_exit=$?'


def say(*a, **k): builtins.print(*a, **k, flush=True)
def flush_stdin():
    """tty에 남은 입력(정지 중 친 두 번째 입력 등)을 버린다: 질문이 잘못된 입력을 소비하지 않게(R16에서 종합 확인 y가 버퍼에 먹혔을 가능성)."""
    try:
        import termios; termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
    except Exception: pass
def inp(p): flush_stdin(); return builtins.input("\a" + p)   # 모든 질문 직전에 입력 버퍼를 비운다
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
        self.last_read = time.monotonic(); v = super().Arm_serial_servo_read(id)
        if v is None or getattr(self, "none_ids", None) and id in self.none_ids: return v
        import math as _m; p_ = self._p(id); return int(_m.ceil(p_ - 1e-9)) if id in (2, 3, 4) else int(_m.floor(p_ + 1e-9))   # 실제 SDK: J2~J4는 180−절삭이라 올림, J1은 내림(D0~D2 기록으로 확인)
    def raw_servo_read(self, id):   # 가짜: Arm_Lib 환산의 역(원시 900~3100 = 0~180°, J2~J4는 반전)을 연속 위치에 적용
        self.last_read = time.monotonic(); a = 180.0 - self._p(id) if id in (2, 3, 4) else self._p(id); return int(round(900 + a * 2200.0 / 180.0))
    def Arm_serial_servo_write(self, id, angle, time_ms):
        self.nw += 1
        if self.read_quiet and time.monotonic() - self.last_read < self.read_quiet: self.quiet_drops += 1; self.writes.append((id, angle, time_ms)); return
        if self.drop_nth is not None and self.nw == self.drop_nth: self.writes.append((id, angle, time_ms)); return   # 가짜: 이 쓰기 한 번만 누락
        if id in self.ignore: self.writes.append((id, angle, time_ms)); return
        tgt = max(angle, self.floors[id]) if id in self.floors and angle < self.floors[id] else angle
        if id in FAKE.get('ceils', {}) and tgt <= 150 and tgt > FAKE['ceils'][id]: tgt = FAKE['ceils'][id]   # 가짜: 닫기 계단(≤150)에서 이 값 위로는 못 간다(블록에 막혀 정체 모사, 블록이 없는 178 닫기는 통과)
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
    if plan.get("set") in ("21", "22", "23", "24"):   # D0·D1-A·D2·D3: 단계마다 눈 읽기 한 값 입력, 하한 미만이면 중단·복귀
        say("\n[측정 대기] D0 자세 %s에서 멈춥니다(서보에 쓰기를 보내지 않고 J2~J4 회독만 읽어 처짐을 감시합니다). **팔 이동 범위 안에 손과 머리를 넣지 마세요.** 해야 할 일: ① 눈금자 쪽에서 눈높이를 손끝 높이에 맞춰 손끝(캡 아래끝) 가장 낮은 곳의 **종이 위 높이(mm)**를 읽습니다(폰 사진은 기록용). ② **읽은 뒤 몸을 팔에서 충분히 떼고** mm 정수 한 개를 쓴 다음 Enter를 누르세요(최대 %d초). 값이 숫자가 아니거나 이 단계 하한(%s mm) 미만이면 **더 내려가지 않고 복귀 질문**이 나옵니다. 값이 충분하면 다음 단계(또는 마지막이면 복귀)로 자동 이동합니다. **빈 Enter나 시간 초과는 값이 없는 것이라 중단·복귀 질문이 나옵니다.**" % (pid, secs, (plan.get("hold_min_mm") or {}).get(str(pid), "?")))
        if plan.get("set") in ("22", "23", "24"): say("  [세트 %s] 눈금자는 두 번째 소리(Ping) 뒤에만 정지한 손끝 옆에 직접 대어 읽으세요(팔·손끝에 닿지 않게 20 mm 이상 띄움). 읽은 뒤 눈금자와 손·머리를 팔 범위 밖으로 완전히 치우고 값을 쓴 다음 Enter를 누르세요." % plan.get("set"))
    elif plan.get("set") == "20":   # P1-b2 (b)·#11 D1(A2): 사람이 할 일은 손끝 높이를 눈으로 읽어 mm 한 값 입력(폰은 기록용), 그 뒤 팔이 자동 복귀한다
        say("\n[측정 대기] 자세 C(모델 손끝 높이 44.1 mm)에서 멈춥니다(서보에 쓰기를 보내지 않고 J2~J4 회독만 읽어 처짐을 감시합니다). **팔 이동 범위 안에 손과 머리를 넣지 마세요.** 해야 할 일: ① 눈금자 쪽에서 눈높이를 손끝 높이에 맞춰 손끝(캡 끝면) 가장 낮은 곳의 눈금을 읽습니다(폰 사진은 기록용, 읽는 값은 눈으로). ② **읽은 뒤 몸을 팔에서 충분히 떼고** mm 숫자 한 개를 쓴 다음(예: 42, 못 읽으면 숫자 없이) Enter를 누르세요. **Enter를 누르거나 %d초가 지나면 팔이 자동으로 복귀 이동을 시작합니다.**" % secs)
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
    if plan.get("gemini_photo") and time.monotonic() - t0 < 55.0:   # D1-A: Gemini 사진(약 40초)이 끝나기 전에 팔이 움직이지 않게 최소 55초까지 정지
        say("  (입력 %r 기록됨. Gemini 사진 촬영이 끝나도록 %d초 더 팔은 그대로 있습니다. **다시 입력하지 마세요.**)" % (rec.get("user_entry"), int(55.0 - (time.monotonic() - t0)) + 1)); time.sleep(max(0.0, 55.0 - (time.monotonic() - t0))); flush_stdin()   # 정지 중 추가 입력은 버린다
    rec["hold_s"] = round(time.monotonic() - t0, 1); rec["hold_ended_by"] = ended; rec["hold_start_epoch"] = round(time.time() - (time.monotonic() - t0), 1); rec["hold_end_epoch"] = round(time.time(), 1)
    say("  측정 대기 종료(%s, %.0f초)." % (ended, rec["hold_s"]))


P2K = {"ad6": None, "start6": None, "cur6": None, "low": False, "contact": False, "logs": {}}   # logs: 어댑터별 쓰기 로그(open·contact·final)를 결과에 따로 남긴다   # P2k: 자세 정지 중 J6 이동 상태(중단 시 J6 복귀에 사용)


def p2k_hold(plan, label, arm, rec, a):
    """점마다 고정 정지(사진용): 입력을 받지 않는다. 1초마다 J2~J4 회독만 읽어 2° 이상 변하면 기록하고 정지를 끝낸다. 시작 줄의 '자세 N에서'를 Mac 래퍼가 읽어 Gemini 사진을 시작한다."""
    secs = float(plan["p2k"]["hold_s"]); t0 = time.monotonic(); base = {k: read_valid(arm, k) for k in (2, 3, 4)}
    say("\n[측정 대기] J6 점 %s: 자세 %s%s에서 %d초 정지합니다(서보에 쓰기를 보내지 않고 J2~J4 회독만 읽습니다). **팔 이동 범위 안에 손을 넣지 말고 입력도 하지 마세요.**" % (label, rec["pose_id"], label, secs))   # 사진 이름이 점마다 달라지도록 자세 번호 뒤에 점 번호를 붙인다(래퍼가 '자세 N에서'의 N으로 사진 이름을 정한다)
    fast = 20 if a.fake else 1; ended = "timeout"
    while (time.monotonic() - t0) * fast < secs:
        time.sleep(1.0 / fast); cur = {k: read_valid(arm, k) for k in (2, 3, 4)}
        if any(cur[k] is None or base[k] is None or abs(cur[k] - base[k]) >= 2 for k in cur): ended = "drift"; rec.setdefault("drift", []).append({"point": label, "start": base, "now": cur}); say("\n! 정지 중 회독이 %s에서 %s로 변했습니다(2° 이상). 처짐 의심: 이 점을 판정 불가로 표시합니다." % (base, cur)); break
    flush_stdin(); return {"hold_s": round((time.monotonic() - t0) * fast, 1), "hold_ended_by": ended, "hold_start_epoch": round(time.time() - (time.monotonic() - t0), 1), "hold_end_epoch": round(time.time(), 1)}


def p2k_j6_sequence(plan, arm, pose_rec, a, mode):
    """자세 정지 중 J6만 178→140→100→140→178(P2k, 승인 P2k-1~4). 점마다 정지 사진. 새 한도 없음: 창 [95,179]·승인 시간 3000/4800 ms·P2g 어댑터 재사용."""
    cfg = plan["p2k"]; today = datetime.date.today()
    ad6 = GripperTestAdapter(arm, {"approved": True, "expires": today.isoformat(), "allowed_times_ms": [3000, 4800], "first_move_min_ms": 3000, "max_writes": 6, "delta_deg": 5, "joint_id": 6, "direction": -1, "angle_window": list(cfg["window"]), "pattern": "list"}, today=today, dry_run=(mode == "dry_run"))
    s6 = ad6.read_start(); P2K["ad6"] = ad6; P2K["start6"] = s6; P2K["cur6"] = s6
    if s6 not in (176, 177, 178, 179): raise Abort("J6 시작 회독 %s가 176~179가 아니다. J6 이동 없음." % s6)   # R-P2k2: 첫 실행이 J6 176(잔차 2°)으로 끝나 하한을 177 → 176으로 넓힘(오므림 상태 유지, 사용자 확인 필요)
    targets = [s6] + list(cfg["j6_targets"]); ad6.set_targets(targets); times = cfg["j6_times_ms"]
    if mode == "dry_run":   # dry run: 팔이 움직이지 않으므로 정지·도착 검사는 건너뛰고 읽기·목록·속도 검사·dry_run 쓰기 기록만 한다(실제 Arm_Lib에서 새 경로 확인)
        for i, (tgt, t_ms) in enumerate(zip(targets[1:], times), 1):
            ad6.confirm_hands_clear(); ad6.move(tgt, t_ms); P2K["cur6"] = tgt; say("  dry_run: J6 점 %d 이동 %d (time %d ms) — 쓰기 0회, 정지·도착 검사 생략" % (i, tgt, t_ms))
        ad6.confirm_hands_clear(); ad6.return_to_start(cfg["return_ms"]); P2K["cur6"] = s6; say("  dry_run: J6 복귀 검사(시작값 %s, %d ms) — 쓰기 0회" % (s6, cfg["return_ms"])); return
    pts = pose_rec.setdefault("j6_points", [])
    def snap(label, tgt): return {"label": label, "j6_target": tgt, "raw_pre_arm": raw_set(arm), "start_epoch": round(time.time(), 1)}
    pt = snap("start", s6); pt.update(p2k_hold(plan, "0", arm, pose_rec, a)); pt["raw_post_arm"] = raw_set(arm); pts.append(pt)
    for i, (tgt, t_ms) in enumerate(zip(targets[1:], times), 1):
        before6 = P2K["cur6"]; ad6.confirm_hands_clear(); time.sleep(QUIET_S); ad6.move(tgt, t_ms); P2K["cur6"] = tgt
        wait = 2 * t_ms + 1000; series = poll(ad6, wait); last = next((v for _, v in reversed(series) if v is not None), None)   # 가짜 서보도 실시간으로 움직이므로 대기를 줄이지 않는다; last = next((v for _, v in reversed(series) if v is not None), None)
        if last is not None and abs(last - before6) <= 1 and abs(tgt - before6) >= 10 and RETRIES["n"] < MAX_RETRIES:   # 쓰기 누락 의심(회독이 이전 값 ±1): 읽기를 멈춘 뒤 같은 목표를 1회만 다시 보낸다(질문 없음, 이동 쓰기 재전송 누적 상한 공유)
            say("  ! J6가 움직이지 않았습니다(회독 %s, 목표 %d). 쓰기 누락으로 보고 같은 목표를 1회 다시 보냅니다." % (last, tgt)); RETRIES["n"] += 1; time.sleep(QUIET_S); ad6.confirm_hands_clear(); ad6.repeat_last(t_ms); pose_rec.setdefault("j6_retries", []).append({"target": tgt})
            series = poll(ad6, wait); last = next((v for _, v in reversed(series) if v is not None), None)
        if last is None or abs(last - tgt) > 3: raise Abort("J6 도착 실패(회독 %s, 목표 %d, 허용 3°). J6 추가 이동 없음, 복귀 질문으로 이어집니다." % (last, tgt))
        if abs(last - tgt) >= 2: say("  ! 경고: J6 회독 잔차가 %d°입니다. 소리·떨림·발열을 확인하세요." % abs(last - tgt))
        pt = snap(str(i), tgt); pt.update({"j6_series_tail": series[-3:], "j6_final_readback": last}); pt.update(p2k_hold(plan, str(i), arm, pose_rec, a)); pt["raw_post_arm"] = raw_set(arm); pts.append(pt)
        if pose_rec.get("drift") and pose_rec["drift"][-1]["point"] == str(i): raise Abort("정지 중 J2~J4 처짐 의심(점 %d). 이후 J6 이동 없음, 복귀로 이어집니다." % i)


def poll_stall(ad, tgt, cmd_ms, max_ms, poll_ms=None):
    """접촉 닫기 한 단계의 읽기: 최대 max_ms까지 읽되, **명령 시간(cmd_ms)이 지난 뒤 같은 값이 3회 연속**이면 바로 끝낸다(서보가 아직 움직이는 중의 우연한 두 번 같은 값을 정체로 오인하지 않기 위해, 누름 시간도 최소화). 반환: (시계열, settled)"""
    series = []; t0 = time.monotonic(); settled = False
    while (time.monotonic() - t0) * 1000.0 < max_ms:
        series.append((round((time.monotonic() - t0) * 1000.0), ad.read_position())); time.sleep((poll_ms or 100) / 1000.0)
        if series[-1][0] >= cmd_ms + 200:
            vv = [v for _, v in series if v is not None]
            if len(vv) >= 3 and vv[-1] == vv[-2] == vv[-3]: settled = True; break
    return series, settled


class _DryJ6(object):
    """dry run 전용: J6 읽기를 '열린 상태(100)'로 대신해 접촉 구간 어댑터 목록·창·속도 검사를 실제 승인 값으로 확인한다(쓰기는 dry_run이라 어댑터가 보내지 않는다)."""
    def __init__(self, arm, v): self._a = arm; self._v = v
    def Arm_serial_servo_read(self, i): return self._v if i == 6 else self._a.Arm_serial_servo_read(i)
    def __getattr__(self, n): return getattr(self._a, n)


def run2_hook(plan, pid, arm, pose_rec, a, mode):
    """Run 2(첫 접촉, 세트 29): 자세 도착마다 호출. 2901(50 mm): J6 열기 → 정지 사진 → 정렬 확인(사람 y/N) / 2902·2903: 하강 중 정지 사진 /
    2904(20 mm): 정지 사진 → P4a 닫기(P2h 정체 규칙, 정체 시 3초 안 열기) → 열린 뒤 사진 / 2905(50 mm 복귀): J6 178로 닫기. 들어올림 없음."""
    cfg = plan["run2"]; today = datetime.date.today(); dry = (mode == "dry_run")
    def mk(sub_cfg, start_ok=None):
        return GripperTestAdapter(arm, sub_cfg, today=today, dry_run=dry)
    if pid == cfg["align_pose"]:
        ad6 = mk({"approved": True, "expires": today.isoformat(), "allowed_times_ms": [3000, 4800], "first_move_min_ms": 3000, "max_writes": 6, "delta_deg": 5, "joint_id": 6, "direction": -1, "angle_window": list(plan["p2k"]["window"]), "pattern": "list"})
        s6 = ad6.read_start(); P2K["ad6"] = ad6; P2K["start6"] = s6; P2K["cur6"] = s6; P2K["logs"]["open"] = ad6.log
        if s6 not in (176, 177, 178, 179): raise Abort("J6 시작 회독 %s가 176~179가 아니다. J6 이동 없음." % s6)
        mid = cfg.get("open_mid", 140); ad6.set_targets([s6, mid, cfg["open_j6"]])   # P2k에서 검증된 3000 ms 두 단계(176 → 140 → 100)
        for tgt_ in (mid, cfg["open_j6"]):
            ad6.confirm_hands_clear(); time.sleep(QUIET_S); ad6.move(tgt_, cfg["open_ms"]); P2K["cur6"] = tgt_; P2K["start_all"][6] = tgt_
            if dry: say("  dry_run: J6 열기 단계 %d (time %d ms) — 쓰기 0회" % (tgt_, cfg["open_ms"])); continue
            series = poll(ad6, 2 * cfg["open_ms"] + 1000); last = next((v for _, v in reversed(series) if v is not None), None)
            if last is None or abs(last - tgt_) > 3: raise Abort("J6 열기 도착 실패(회독 %s, 목표 %d). 하강 없음, 복귀로 이어집니다." % (last, tgt_))
        if dry: P2K["ad6c_dry"] = True; return
        pose_rec.setdefault("run2", {})["open_readback"] = last
        h = p2k_hold(plan, "0", arm, {"pose_id": pid}, a); pose_rec["run2"]["hold_open50"] = h
        say("\n[정렬 확인] Mac 화면의 정렬 보고(d_x, 캡별 옆 여유)를 읽으세요. **|d_x| ≤ 3 mm 이고 두 캡 옆 여유가 모두 6 mm 이상이며 블록 마커가 기준 사진과 1 px 이내**일 때만 y. 아니거나 보고가 없으면 N(하강 없이 복귀합니다).")
        ok = ask("정렬 확인이 모두 맞습니까?"); pose_rec["run2"]["align_confirmed"] = bool(ok)
        if not ok: raise Abort("정렬 확인 N(또는 보고 없음): 하강하지 않고 복귀합니다.")
        return
    if pid in cfg["descent_poses"] and pid != cfg["contact_pose"]:
        P2K["low"] = True
        if dry: return
        pose_rec.setdefault("run2", {})["hold_%d" % pid] = p2k_hold(plan, "0", arm, {"pose_id": pid}, a); return
    if pid == cfg["contact_pose"]:
        P2K["low"] = True
        adc = GripperTestAdapter(_DryJ6(arm, cfg["open_j6"]) if dry else arm, {"approved": True, "expires": today.isoformat(), "allowed_times_ms": [1500, 3000], "first_move_min_ms": None, "max_writes": 9, "delta_deg": 5, "joint_id": 6, "direction": -1, "angle_window": list(cfg["window_close"]), "pattern": "list"}, today=today, dry_run=dry)
        s = adc.read_start()
        if not dry and abs(s - cfg["open_j6"]) > 2: raise Abort("접촉 전 J6 회독 %s가 열림 %d와 2° 넘게 다르다. 닫기 없음." % (s, cfg["open_j6"]))   # dry run은 팔이 움직이지 않아 J6가 아직 오므림이므로 검사만 생략(어댑터 목록·속도 검사는 한다)
        seq = [s] + list(cfg["close_seq"]); adc.set_targets(seq); P2K["ad6c"] = adc; P2K["logs"]["contact"] = adc.log
        if dry:
            for t_ in seq[1:]: adc.confirm_hands_clear(); adc.move(t_, cfg["close_ms"]); say("  dry_run: P4a 닫기 목표 %d (time %d ms) — 쓰기 0회" % (t_, cfg["close_ms"]))
            adc.confirm_hands_clear(); adc.return_to_start(cfg["release_ms"]); say("  dry_run: 정체 뒤 열기 검사(시작 %d, %d ms) — 쓰기 0회" % (s, cfg["release_ms"])); return
        r2 = pose_rec.setdefault("run2", {}); r2["hold_%d_open" % pid] = p2k_hold(plan, "0", arm, {"pose_id": pid}, a)
        hg = cfg.get("height_gate")   # 닫기 직전 높이 관문(사용자 승인 문턱): 원시 각도로 예측한 닫힘 모델 캡 끝 높이가 문턱 미만이면 닫지 않고 복귀한다
        if hg:
            for _try in range(3):   # 읽기 실패(None) 한 번으로 시험이 판정 불가가 되지 않게 최대 3회 읽는다(읽기 전용, 이어지는 쓰기 앞의 0.3초 조용한 시간은 그대로)
                rw = raw_set(arm); dg = [(rw.get(str(k)) or {}).get("deg") for k in (2, 3, 4)]
                if all(v is not None for v in dg): break
                time.sleep(0.05)
            if any(v is None for v in dg): raise Abort("높이 관문: J2~J4 원시 각도 읽기 실패(None). 닫지 않고 복귀합니다.")
            m_ = hg["model"]; hp = m_["h_ref"] + sum(c_ * (d_ - q_) for c_, d_, q_ in zip(m_["a"], dg, m_["q_ref"]))
            r2["height_pred_mm"] = round(hp, 2); r2["raw_at_contact"] = dg; say("  [높이 관문] 원시 J2~J4 %s → 예측 닫힘 모델 캡 끝 %.1f mm (문턱 %.1f mm 이상이어야 닫습니다)" % ([round(v, 2) for v in dg], hp, hg["h_min"]))
            if hp < hg["h_min"]: raise Abort("높이 관문 미달: 예측 %.1f mm < %.1f mm. 닫지 않고 복귀합니다(열린 채 상승)." % (hp, hg["h_min"]))
        say("\n[접촉] J6를 열린 %d에서 %s로 닫습니다(P2h 정체 규칙: 명령 − 회독 ≥ 3° 이고 같은 값 연속 2회면 정체 → 3초 안에 엽니다). 손을 팔 범위에 넣지 마세요." % (s, cfg["close_seq"]))
        steps = []; stalled = None
        for t_ in seq[1:]:
            adc.confirm_hands_clear(); time.sleep(QUIET_S); adc.move(t_, cfg["close_ms"]); P2K["cur6"] = t_
            P2K["contact"] = True; series, settled = poll_stall(adc, t_, cfg["close_ms"], 2 * cfg["close_ms"] + 1000); vals = [v for _, v in series if v is not None]   # 명령 시간 뒤 같은 값 3회 연속이면 바로 끝(누름 시간 최소화, 가짜 정체 방지)
            if FAKE.get("abort_close") == len(steps) + 1: raise KeyboardInterrupt()   # 가짜 시험: 닫기 계단 중 중단 주입
            st = {"target": t_, "series_tail": series[-4:], "final": vals[-1] if vals else None}
            st["settled"] = settled
            if settled and t_ - vals[-1] >= 3: stalled = vals[-1]; st["stall"] = True   # P2h 규칙(명령 − 회독 ≥ 3°)을 정착(명령 시간 뒤 같은 값 3회 연속)한 뒤에만 정체로 센다
            steps.append(st); say("  닫기 목표 %d → 회독 %s%s" % (t_, st["final"], " ← 정체" if st.get("stall") else ""))
            if stalled is not None: break
        r2["close_steps"] = steps; r2["stall_in_band"] = (stalled is not None and 138 <= stalled <= 146); r2["stall_readback"] = stalled; r2["stall_command"] = steps[-1]["target"] if stalled is not None else None
        adc.confirm_hands_clear(); time.sleep(QUIET_S); adc.return_to_start(cfg["release_ms"]); P2K["cur6"] = s; P2K["contact"] = False
        sr = poll(adc, 2 * cfg["release_ms"] + 1000); lastr = next((v for _, v in reversed(sr) if v is not None), None); r2["release_readback"] = lastr
        say("  열기 완료(회독 %s). 정체 %s." % (lastr, ("회독 %s" % stalled) if stalled is not None else "없음(150까지 닫혔다 = 블록 없음 곡선 또는 접촉 안 됨)"))
        if lastr is None or abs(lastr - s) > 3: raise Abort("열기 도착 실패(회독 %s, 목표 %d). 이후 이동 없음, 복귀로 이어집니다." % (lastr, s))
        r2["hold_%d_released" % pid] = p2k_hold(plan, "1", arm, {"pose_id": pid}, a); return
    if pid == cfg["ascent_pose"]:
        if dry: return
        ad6 = P2K["ad6"]; adc = P2K.get("ad6c")
        fin = mk({"approved": True, "expires": today.isoformat(), "allowed_times_ms": [3000, 4800], "first_move_min_ms": None, "max_writes": 4, "delta_deg": 5, "joint_id": 6, "direction": -1, "angle_window": list(plan["p2k"]["window"]), "pattern": "list"})
        s = fin.read_start()
        if abs(s - cfg["open_j6"]) > 2: raise Abort("50 mm 복귀 뒤 J6 회독 %s가 열림 %d와 다르다. J6 닫기 없음." % (s, cfg["open_j6"]))
        fin.set_targets([s, cfg.get("open_mid", 140), cfg["close_to"]]); P2K["logs"]["final"] = fin.log; P2K["ad6"] = fin; P2K["low"] = False
        for tgt_ in (cfg.get("open_mid", 140), cfg["close_to"]):   # 100 → 140 → 178 (3000 ms 두 단계)
            fin.confirm_hands_clear(); time.sleep(QUIET_S); fin.move(tgt_, cfg["close_to_ms"]); P2K["cur6"] = tgt_; P2K["start_all"][6] = tgt_
            series = poll(fin, 2 * cfg["close_to_ms"] + 1000); lastf = next((v for _, v in reversed(series) if v is not None), None)
        pose_rec.setdefault("run2", {})["close_readback"] = lastf; say("  J6 닫기 완료(회독 %s)." % lastf)
        P2K["start6"] = cfg["close_to"]   # 이후 복귀는 닫힌 상태가 기준
        return


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
    if plan.get("no_camera"):   # D0(세트 21): 카메라를 쓰지 않고 도착 시점 회독·원시 값만 기록(FK 높이 재계산용)
        pre = {k: ads[k].read_position() for k in (1, 2, 3, 4)}; raw_pre = raw_set(arm)
        say("\n[도착] 자세 %s (카메라 없음). J1~J4 회독 %s" % (pid, pre)); return {"no_camera": True, "pre": pre, "raw_pre": raw_pre}
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
    ap.add_argument("--fake-ignore", type=int, nargs="*", default=[]); ap.add_argument("--fake-floor", type=str, default="", help="예: 2:95 (서보:바닥값)"); ap.add_argument("--fake-abort-close", type=int, default=0, help="Run 2 가짜: N번째 닫기 단계에서 중단 주입"); ap.add_argument("--fake-ceil", type=str, default="", help="예: 6:142 (서보:천장값, 블록 정체 모사)")
    ap.add_argument("--fake-entry", type=str, default="", help="D0 가짜 시험: 단계별 입력값을 쉼표로(예: 52,45,30,x)"); ap.add_argument("--fake-drop-nth", type=int, default=None); ap.add_argument("--fake-hyst", type=str, default="", help="예: 1.5,0.3 (증가 방향 모자람°, 감소 방향 넘어감°)"); ap.add_argument("--fake-countdown-int", type=int, default=None); ap.add_argument("--fake-return-int", type=int, default=None); ap.add_argument("--fake-capture-fail", action="store_true"); ap.add_argument("--fake-read-quiet", type=float, default=0.0, help="가짜: 마지막 읽기 뒤 이 초 안의 쓰기를 버린다(실측 기제 모사)"); ap.add_argument("--run-id", default="local"); ap.add_argument("--out", default="/tmp/g01_5b4_s42_result.json"); ap.add_argument("--stop-after", type=int, default=None, help="가짜 시험: 이동 N번 뒤 사용자 중단을 흉내")
    a = ap.parse_args()
    if a.fake and a.real: sys.exit("--fake와 --real은 함께 쓸 수 없다")
    FAKE_ENTRIES[:] = [x.strip() for x in a.fake_entry.split(",") if x.strip()] if a.fake_entry else []
    FAKE.update({"countdown_int": a.fake_countdown_int, "return_int": a.fake_return_int, "capture_fail": a.fake_capture_fail})
    plan = json.load(open(a.plan, encoding="utf-8"))
    FAKE["abort_close"] = int(a.fake_abort_close) if a.fake_abort_close else None
    FAKE["ceils"] = {int(t_.split(":")[0]): int(t_.split(":")[1]) for t_ in a.fake_ceil.split(",") if ":" in t_}
    if plan["set"] == "23": WINDOWS[2] = [55, 110]; WINDOWS[1] = [70, 122]
    if plan["set"] == "31": WINDOWS[2] = [66, 110]   # Run 3: J2 창 하한 70 → 66(이 세트에 한정, 사용자 승인 N1 2026-11-17). 하강 자세 J2 69의 도착 회독 68(더 넘어가면 67)에서 복귀 쓰기가 거부되지 않게 한다. docs/gates/G07-첫접촉시험-선행조건과-최소시험안.md §17
    if plan["set"] == "24": WINDOWS[2] = [52, 110]; WINDOWS[1] = [70, 122]   # D3: J2 창 하한 55 → 52(이 세트에 한정). 2404 도착 J2가 54.9° 부근(정수 회독 54)일 수 있어 하한 55이면 복귀 쓰기가 거부된다(가짜 시험 C에서 발견, 승인 D3-5b 대기)   # D2: J2 창 하한 55(세트 22 승인값), J1 창 상한 110 → 122(이 세트에 한정, 선택 자세 J1 120, 승인 D2-1·D2-2 2026-11-16)
    if plan["set"] == "22": WINDOWS[2] = [55, 110]   # D1-A: J2 창 하한 70 → 55(이 세트에 한정, 승인 D1-2 2026-11-16)
    if plan["set"] == "21": WINDOWS[2] = [58, 110]   # D0: J2 창 하한 70 → 58(이 세트에 한정, 사용자 승인 D0-3, docs/gates/G07-D0-파지근방-손끝높이-승인-요청-문서.md §5)
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
        state = {i: read_valid(arm, i) for i in range(1, 7)}; result["state0"] = state; start_all = dict(state); P2K["start_all"] = start_all   # Run 2: J6를 일부러 움직이므로 이동 중 "명령하지 않은 관절" 감시값(start_all[6])을 훅이 갱신한다
        say("b-4-0 현재 회독(SDK 각도): " + ", ".join("J%d=%s" % (i, v) for i, v in state.items()))
        if any(state[i] is None for i in range(1, 7)): raise Abort("회독 None이 있다. 쓰기 없음.")
        for k in range(1, 7):
            if abs(state[k] - ps[str(k)]) > START_TOL: raise Abort("J%d 현재 회독 %s가 계획 시작값 %s와 %d° 넘게 다르다. 계획을 다시 만든 뒤 실행하세요. 쓰기 없음." % (k, state[k], ps[str(k)], START_TOL))
        if plan.get("p2k") and not (176 <= state[6] <= 179): raise Abort("J6 회독 %s가 176~179(오므림)가 아니다. P2k는 오므린 그리퍼에서만 시작한다. 쓰기 없음." % state[6])
        # 이동 목록 → 관절별 목표 목록(첫 항목은 실제 시작값)
        lists = {j: [state[j]] + [m["target"] for m in moves if m["joint"] == j] for j in (1, 2, 3, 4)}
        times = [m["time_ms"] for m in moves] + [3000, 1500, 8000]
        ads = make_adapters(arm, lists, times, mode, 8000)
        cmd = {j: state[j] for j in (1, 2, 3, 4)}
        say("\n계획 요약: 접기 계단 후 기준 깊은 자세 J1~J4 = %s (모델 높이 %.1f mm), 자세 순서 %s" % (plan["base"], plan["gate"]["base_z"], list(plan["poses"].keys())))
        say("자세별 모델 높이(mm) %s, 전이 최저 %s, 복귀(첫 구간) 최저 %.1f mm" % (plan["gate"]["pose_z"], plan["gate"]["transition_min"], plan["gate"]["return_j12_min"]))
        say("\n== 안전 확인(시작 시 1회) ==")
        qs = ["전원 차단 담당자가 현재 자리에 있습니까?", "낙하 받침을 준비했고, 접힌 팔(앞쪽, 보드 위)의 낙하를 받치되 팔 경로(좌우 약 ±40 mm, TCP y 약 -15~220 mm, 높이 약 70~460 mm) 안에 들어오지 않습니까?", "팔이 접히고 좌우로 도는 동안 닿을 물체·손·사람·케이블이 팔 주변과 작업대·보드 위에 없습니까? 보드 종이가 움직이지 않았습니까?", "팔이 직립(대기) 자세입니까? J4(손목)가 아래팔과 일직선입니까?", "DaBai·그리퍼 케이블이 당겨지지 않고 여유가 있어 보입니까?", "J2·J3·J4를 만져서 **뜨겁지 않습니까?** (온도 읽기 API가 없어 손으로 확인)", "위 회독값과 이동 계획이 맞다."]
        if plan["set"] in ("12", "14", "15", "16", "17", "18", "19", "20", "21", "22", "23", "24"):   # P1·큰/근거리 M3: 블록·마커가 지정 자리에 놓여 있고, 그리퍼는 오므린 채 J6를 유지한다(S4용 "작업대 위에 물체 없음" 질문은 맞지 않는다)
            qs = ["전원 차단 담당자가 현재 자리에 있습니까?", "노란 블록(30×60×30 mm, 윗면 마커 id 7)이 지정 자리(중심 x +1.4, y 약 189 mm, 앞 모서리 y 약 219 mm)에 있고 움직이지 않았습니까? 블록과 베이스 판 사이·팔 주변에 그 밖의 물체·손·사람·케이블이 없습니까?",
                  "낙하 받침을 준비했고 블록(x ±15, y 160~220 mm)과 팔 경로에 겹치지 않습니까? (받침은 사용자가 직접 놓은 자리이며 Claude가 낙하 영역을 계산해 제안한 것이 아니다)", "팔이 K1 기립 자세이고 그리퍼는 오므려져 있습니까(J6 회독 177~179)? 이번 실행은 J6를 건드리지 않고 지금 상태를 유지합니다.",
                  "DaBai·그리퍼 케이블이 당겨지지 않고 여유가 있어 보입니까?", "J2·J3·J4를 만져서 **뜨겁지 않습니까?** (온도 읽기 API가 없어 손으로 확인)", "위 회독값과 이동 계획이 맞다."]
        if plan["set"] in ("14", "15", "16", "17", "18", "19", "20", "21", "22", "23", "24"): qs = [q.replace("J2·J3·J4를 만져서", "J2·J3·J4와 오므린 그리퍼 서보 J6를 만져서") for q in qs]   # 큰 이동 M3: J6 발열 확인 포함(P1에서 비어 있던 항목)
        if plan["set"] in ("18", "19"): qs.insert(-1, "이번 실행은 J2(어깨)와 J3(팔꿈치)가 뒤쪽(SDK 증가 방향)으로 각각 최대 45° 기울어집니다(J2 최대 135, J3 최대 67). 충돌 계산에는 팔 뒤쪽·위쪽의 물체가 없으므로 사람이 확인합니다: 팔 뒤쪽(약 300 mm)과 위쪽에 Jetson 보드·케이블·카메라 거치대·벽·손이 없고, 케이블이 뒤로 젖혀져도 당겨지지 않습니까?")   # D6: 새 영역 J2 110 초과(이 프로젝트에서 처음), 충돌 모델 밖의 뒤쪽 물체 확인
        if plan["set"] in ("20", "21", "22", "23", "24"):   # 낙하 받침 질문 대체: 사용자가 받침을 쓰지 않기로 결정(2026-11-16). 전원 차단 시 팔이 내려앉는 자리를 비워 두는 확인으로 바꾼다
            qs = [("낙하 받침 없이 진행합니다(사용자 결정 2026-11-16). 전원이 끊기면 팔이 넘어지거나 내려앉을 수 있습니다(처음 접기 구간에서는 높은 자세에서 넘어질 수 있고 그 범위는 계산하지 않았습니다). 팔 앞쪽·옆쪽(블록 너머 포함)과 팔 주변에 손·사람·케이블·물체가 없고, 눈금자는 팔 이동 범위 밖에 단단히 세워져 있습니까?" if q.startswith("낙하 받침을 준비했고") else q) for q in qs]
        if plan["set"] == "20": qs.insert(-1, "이번 실행은 P1-b2 (b): 자세 C(J2 82, J3 10, J4 11, 모델 손끝 높이 44.1 mm)까지만 가고 C2는 가지 않습니다. 이 높이(지금까지 최저 57.1 mm보다 약 13 mm 아래)는 처음 가는 영역입니다. 자세 C에서 멈추는 동안(최대 180초) 눈금자로 **종이 위 손끝 높이**를 눈으로 읽어 mm 숫자 한 개를 입력합니다(폰은 기록용). 눈금자는 팔 이동 범위 밖(옆 약 97 mm)에 있고, 읽을 때 손과 머리가 팔 이동 범위 안에 들어가지 않으며, 읽은 뒤 몸을 팔에서 떼고 Enter를 누를 수 있습니까? (측정값은 Enter 전에 한 줄로 쓰면 기록에 남습니다)")   # 결정 1 (b)·#11 D1(A2) 추가 항목, 사용자 승인 2026-11-16
        if plan["set"] == "21": qs.insert(-1, "이번 실행은 D0(파지 근방 손끝 높이 직접 측정): J1을 블록 반대쪽으로 돌려(J1 110) 손끝이 블록에서 **Gemini 사진 기준 오른쪽(사용자 오른쪽)** 옆 약 67 mm 떨어진 자리로 4단계(FK 종이 기준 높이 약 69 → 60 → 49 → 44 mm)로 내려갑니다. J2 61~69°(처음 가는 영역, 어댑터 J2 창 하한 58)입니다. 단계마다 눈금자로 손끝(캡 아래끝) 높이를 눈으로 읽어 **종이 위 mm 정수 한 값**을 입력해야 하며 숫자가 아니거나 하한(40·32·22·18) 미만이면 더 내려가지 않고 복귀합니다. 눈금자는 손끝 자리 옆에 팔 이동 범위 밖으로 단단히 세워져 있고, 읽을 때 손과 머리가 팔 범위 안에 들어가지 않으며, 읽은 뒤 몸을 팔에서 떼고 Enter를 누를 수 있습니까? **첫 자세 도착 때 손끝이 사진 왼쪽으로 가 있거나 눈금자와 약 240 mm 떨어져 읽을 수 없으면 값을 쓰지 말고 Enter만 누르세요(복귀).** 눈금자는 블록 중심에서 사진 오른쪽 약 170 mm(손끝에서 옆 약 100 mm)로 세워야 하며, 블록 중심에서 같은 쪽 약 150 mm 안(y 187 기준)에 두면 팔과 겹칩니다. (값을 쓰지 않고 Enter만 누르거나 시간이 지나면 값이 없는 것이라 더 내려가지 않고 복귀 질문으로 갑니다)")   # D0-1~5 승인 2026-11-16
        if plan["set"] == "22": qs.insert(-1, "이번 실행은 D1-A(같은 FK 높이 종이 기준 60 mm에서 자세만 다른 4개): J1 110으로 손끝이 블록에서 **Gemini 사진 기준 오른쪽(사용자 오른쪽)** 옆으로 가고, J2 71 → 65 → 61 → 57 순으로 자세가 바뀝니다(J2 57은 처음 가는 영역, 어댑터 J2 창 하한 55). 자세마다 눈금자로 손끝(캡 아래끝) 높이를 눈으로 읽어 **종이 위 mm 정수 한 값**을 입력하며, 맥이 Gemini 사진을 자동으로 1장 찍습니다(약 40초, 팔은 움직이지 않음, p620 전원이 꺼져 있으면 사진만 실패하고 시험은 계속됩니다). 입력이 숫자가 아니거나 자세별 하한(24·20·20·12 mm, 다음 자세까지 예상 추가 하강을 더한 값) 미만이면 더 내려가지 않고 복귀합니다. **도착하면 소리가 나고, 사진 촬영이 끝나 두 번째 소리(Ping)가 날 때까지 눈금자 앞에 서지 말고 팔과 사진 화면 밖에 계세요. 두 번째 소리 뒤에 읽으세요.** **눈금자는 평소 팔 이동 범위 밖에 치워 두고(탁자에 세워 두지 않음, 사용자 결정 2026-11-16), 팔이 도착해 두 번째 소리가 난 뒤에만 정지한 손끝 옆에 직접 대어(팔·손끝에 닿지 않게 최소 20 mm 띄움) 읽고, 읽은 뒤 눈금자와 손·머리를 팔 이동 범위 밖으로 완전히 치운 다음에 Enter를 누를 수 있습니까?** 팔이 움직이는 동안(Enter 뒤)에는 눈금자·손·머리가 모두 범위 밖이어야 합니다. 첫 자세 도착 때 손끝이 사진 왼쪽으로 가 있거나 눈금자와 약 240 mm 떨어져 읽을 수 없으면 값을 쓰지 말고 Enter만 누르세요(복귀). (값 없는 Enter나 시간 초과는 값이 없는 것이라 복귀 질문으로 갑니다)")
        if plan["set"] == "23": qs.insert(-1, "이번 실행은 D2(낮은 높이 오프셋 + 수평 오프셋): J1 110으로 손끝이 블록에서 **Gemini 사진 기준 오른쪽(사용자 오른쪽)** 옆으로 가서 계획 캡 아래끝 높이 32 → (J1 120으로 돌려 옆 약 100 mm에서 같은 높이 32) → 27 → 23.7 mm(예상 실제 도착 25 → 25 → 20 → 16.7 mm, 최악 약 10 mm)로 내려갑니다. J1 창 상한을 이 세트에 한해 122로 넓힙니다(J1 120 자세). 자세마다 눈금자로 손끝(캡 아래끝) 높이를 눈으로 읽어 **종이 위 mm 정수 한 값**을 입력하며, 맥이 Gemini 사진을 자동으로 1장 찍습니다(약 40초, 팔은 움직이지 않음, p620 전원이 꺼져 있으면 사진만 실패하고 시험은 계속됩니다). 입력이 숫자가 아니거나 17 mm 미만(마지막 자세는 12 mm 미만)이면 더 내려가지 않고 복귀합니다. **도착하면 소리가 나고, 사진이 끝나 두 번째 소리(Ping)가 날 때까지 눈금자 앞에 서지 말고 팔과 사진 화면 밖에 계세요.** **눈금자는 평소 팔 이동 범위 밖에 치워 두고(탁자에 세워 두지 않음), 두 번째 소리 뒤에만 정지한 손끝 옆에 직접 대어(팔·손끝에 닿지 않게 최소 20 mm 띄움) 읽고, 읽은 뒤 눈금자와 손·머리를 팔 이동 범위 밖으로 완전히 치운 다음 Enter를 누를 수 있습니까?** J1 120 자세에서는 손끝이 블록에서 사진 오른쪽으로 더 멀리(옆 약 100 mm) 갑니다. 첫 자세 도착 때 손끝이 사진 왼쪽에 있거나 읽을 수 없으면 값을 쓰지 말고 Enter만 누르세요(복귀).")
        if plan["set"] == "24": qs = [q_.replace("눈금자는 팔 이동 범위 밖에 단단히 세워져 있습니까?", "눈금자는 탁자 밖(손에 들고 대기)에 있습니까?") for q_ in qs]   # 새 운용(승인 D3): 눈금자를 탁자에 세워 두지 않는다
        if plan["set"] == "24": qs.insert(-1, "이번 실행은 D3(실제 캡 아래끝 15~20 mm 도달 + J1 120 위치 비교): J1 110으로 손끝이 **Gemini 사진 기준 오른쪽(사용자 오른쪽)** 옆으로 가서 (2401) 기준 높이 약 23.5 mm에서 눈 읽기·사진 → (2402) J1만 120으로 돌려 같은 높이에서 눈 읽기·사진 → J1 110 복귀 후 (2403) J4 +5°(측정 없음, 원시 각도 판정) → (2404) J2 −3° 하강(예상 캡 아래끝 약 15.4 mm, 최악 약 14.7 mm). **2403 도착 뒤 원시 J3·J4와 J2 54.5°로 예측한 캡 아래끝이 12.5 mm 미만이면 하강하지 않고, 2404 도착 뒤 예측 캡 아래끝이 12.5 mm 미만이거나 J2 원시가 54.5° 미만이면 측정 없이 복귀합니다(재명령 없음, 선형 높이 예측 판정). 정상인데도 예측이 문턱에 가까워 하강하지 않고 복귀할 수 있습니다.** 2401·2402 입력이 21 mm 미만이거나 값이 없으면 하강하지 않고 복귀하며, 2404 입력이 12 mm 미만이어도 복귀합니다. J1 창 상한은 이 세트에 한해 122입니다. 맥이 Gemini 사진을 자동으로 찍습니다(약 40초, 두 번째 소리 Ping 뒤에 눈금자). **눈금자는 평소 탁자 밖에서 손에 들고 대기하다가, 두 번째 소리 뒤에만 정지한 손끝 옆에 직접 대어(팔·손끝에 닿지 않게 최소 20 mm 띄움) 읽고, 읽은 뒤 눈금자와 손·머리를 팔 이동 범위 밖으로 완전히 치운 다음 Enter를 누를 수 있습니까?** **연장한 흰 종이(종이 오른쪽에 약 3 mm 겹쳐 오른쪽 약 70 mm, 세로 y 95~235 mm)가 평평하게 붙어 들뜨지 않았고 탁자 가장자리를 넘지 않았습니까?** 첫 자세 도착 때 손끝이 사진 왼쪽에 있거나 읽을 수 없으면 값을 쓰지 말고 Enter만 누르세요(복귀).")
        if plan["set"] in ("25", "26", "27", "28"):   # P2k: 자세 정지 중 J6만 움직임(25~27 블록 없음, 28 Run 1은 블록 있음)
            qs = ["전원 차단 담당자가 현재 자리에 있습니까?", ("노란 블록(30×60×30 mm, 윗면 마커 id 7)이 지정 자리(중심 x +1.4, y 약 189 mm)에 있고 움직이지 않았으며, 눈금자·클램프·폰 거치대는 없고, 보드 위에 그 밖의 물체·손·사람·케이블이 없으며, 손가락이 벌어지는 범위(손가락 중심에서 좌우 ±47 mm, 블록은 캡 끝보다 약 20 mm 아래로 손가락 사이에 있음)에도 다른 것이 없습니까?" if plan["set"] == "28" else "노란 블록·눈금자·클램프·폰 거치대를 모두 치웠고, 보드 위에 종이 외 물체·손·사람·케이블이 없으며, 손가락이 벌어지는 범위(손가락 중심에서 좌우 ±47 mm, 자세 위치 기준)에도 아무것도 없습니까?"),
                  "낙하 받침 없이 진행합니다(사용자 결정 2026-11-16). 전원이 끊기면 팔이 넘어지거나 내려앉을 수 있습니다. 팔 앞쪽·옆쪽과 팔 주변에 손·사람·케이블·물체가 없습니까?",
                  "팔이 K1 기립 자세이고 그리퍼는 오므려져 있습니까(J6 회독 176~179, 지난 실행이 176으로 끝나 하한을 176으로 넓혔습니다)? 이번 실행은 블록 중심 위 자세에서 정지한 채 J6만 178 → 140 → 100 → 140 → 178로 움직입니다(열리면 손가락이 좌우 약 ±47 mm까지 벌어지고 손끝은 올라갑니다). 점마다 55초 정지하는 동안 입력하지 마세요.",
                  "DaBai·그리퍼 케이블이 당겨지지 않고 여유가 있어 보입니까?", "J2·J3·J4와 그리퍼 서보 J6를 만져서 **뜨겁지 않습니까?**", "위 회독값과 이동 계획이 맞다."]
        if plan["set"] in ("29", "31"):   # Run 2·3(첫 접촉): 블록 옆으로 내려가 닫는다. 들어올림 없음(Run 3은 더 깊이 + 닫기 직전 높이 관문)
            qs = ["전원 차단 담당자가 현재 자리에 있습니까?", "노란 블록(마커 id 7)은 **Run 3 이후 만지지 않았고** 지정 자리 그대로이며, 눈금자·클램프·폰 거치대는 없고, 보드 위에 그 밖의 물체·손·사람·케이블이 없습니까?",
                  "낙하 받침 없이 진행합니다(사용자 결정 2026-11-16). 전원이 끊기면 팔이 넘어지거나 내려앉을 수 있습니다. 팔 앞쪽·옆쪽과 팔 주변에 손·사람·케이블·물체가 없습니까?",
                  "이번 실행은 **처음으로 손가락이 블록에 닿는 시험**입니다: J1 86, 50 mm에서 J6를 100으로 열고 정렬을 확인한 뒤(Mac 화면의 보고를 읽고 y/N 1회), 열린 채 블록 양옆으로 내려가(Run 3은 Run 2보다 닫힘 모델 캡 끝이 약 10 mm 더 깊이: 예상 종이 위 26.6 → 16.4 mm) 닫기 직전 **높이 관문**(원시 각도 예측 닫힘 캡 끝이 **종이 위 7 mm 이상**, 미달이면 닫지 않고 복귀)을 통과하면 J6를 닫아 블록을 물어 정체를 읽고 3초 안에 열고 올라옵니다. 들어올림은 없습니다. 하강·닫는 동안 손을 팔 범위에 넣지 마세요. **J6를 연 뒤에도 블록이 손가락에 붙어 함께 올라오면 즉시 Ctrl-C(전원 스위치는 담당자)**.",
                  "팔이 K1 기립 자세이고 그리퍼는 오므려져 있습니까(J6 회독 176~179)?", "DaBai·그리퍼 케이블이 당겨지지 않고 여유가 있어 보입니까?", "J2·J3·J4와 그리퍼 서보 J6를 만져서 **뜨겁지 않습니까?**", "위 회독값과 이동 계획이 맞다."]
        for k, q in enumerate(qs, 1): say("  %d. %s" % (k, q))
        if not ask("위 %d개를 모두 확인했습니까? (y면 이동 시작, 이동마다 소리 뒤 대기 후 움직입니다. 자세 도착마다 촬영 중 가만히 계세요)" % len(qs)): raise Abort("사전 확인 미충족. 쓰기 없음.")
        say("\n▶ **지켜볼 관절은 이동 안내마다 바뀝니다**(J1 좌우 회전, J2 어깨, J3 팔꿈치, J4 손목). 소리가 나면 안내된 관절을 보세요. 접힌 자세에서 J1·J2를 처음 움직일 때는 처짐·케이블을 특히 봐 주세요.")
        for n, mv in enumerate(moves, 1):
            countdown = 2 if (n <= 2 or mv["time_ms"] >= 8000) else 1   # 속도 상향(사용자 2026-11-13): 첫 이동·8000 ms 이동만 2초, 나머지 1초(소리 신호는 유지)
            if a.stop_after is not None and n > a.stop_after: raise KeyboardInterrupt()
            step = run_move(ads, arm, mv, cmd, start_all, result, n, N, mode, countdown); result["steps"].append(step); done_moves = n; wrote = wrote or mode != "dry_run"
            if plan.get("raw_after") and mode != "dry_run": step["raw_after"] = raw_after_pair(arm)   # 세트 16: 이동마다 원시 값 두 시점(읽기 전용)
            if mv.get("capture") and mode == "dry_run" and plan.get("p2k") and not plan.get("run2"): p2k_j6_sequence(plan, arm, {"pose_id": mv["pose_id"]}, a, mode)
            if mv.get("capture") and mode == "dry_run" and plan.get("run2"): run2_hook(plan, mv["pose_id"], arm, {"pose_id": mv["pose_id"]}, a, mode)
            if mv.get("capture") and mode != "dry_run":
                pid = mv["pose_id"]; cap = capture(plan, pid, arm, ads, a)
                result["poses"].append({"pose_id": pid, "commanded": dict(cmd), "readback": {str(k): last_valid(step["series"][str(k)]) if k in (1, 2, 3, 4) else None for k in (1, 2, 3, 4)}, "target_pose": plan["poses"][str(pid)], "capture": cap})
                hc_ = (plan.get("height_check") or {}).get(str(pid))   # D3 진행 판정(승인 D3-5): 원시 각도로 선형 예측한 캡 아래끝 높이가 문턱 미만이면 측정·하강 없이 복귀(재명령 없음)
                if hc_ and mode != "dry_run":
                    raw_ = (cap or {}).get("raw_pre") or {}; rb_ = result["poses"][-1]["readback"]; mdl_ = plan["height_check"]["model"]; q0_ = mdl_["q_ref"]; ah_ = mdl_["a"]
                    def _dg(k_):
                        v_ = (raw_.get(str(k_)) or {}).get("deg")
                        if v_ is not None: return v_, False
                        if rb_.get(str(k_)) is None: return None, True
                        return (rb_[str(k_)] + 0.5 if k_ == 1 else rb_[str(k_)] - 1.0), True   # 원시 None 대체(보수): J1은 내림이라 정수+0, J2~J4는 올림이라 실제 ∈ (정수−1, 정수] → 정수−1
                    j2_, n2_ = _dg(2); j3_, n3_ = _dg(3); j4_, n4_ = _dg(4); bad_ = []
                    if str(pid) == "2403" and (n3_ or n4_): bad_.append("J3/J4 원시 회독 없음(None) — 판정 불가, 안전 쪽 정지")
                    elif None in (j2_, j3_, j4_): bad_.append("회독 없음")
                    else:
                        j2u_ = hc_.get("j2_assumed", j2_)   # 2403: 다음 하강의 최악 J2 가정(54.5), 2404: 실제 원시 J2
                        hp_ = mdl_["h_ref"] + ah_[0] * (j2u_ - q0_[0]) + ah_[1] * (j3_ - q0_[1]) + ah_[2] * (j4_ - q0_[2])
                        result["poses"][-1]["pred_cap_mm"] = round(hp_, 2)
                        if hp_ < hc_["h_min"]: bad_.append("예측 캡 아래끝 %.1f mm < %.1f (J2 %.2f 가정/실제, J3 %.2f, J4 %.2f)" % (hp_, hc_["h_min"], j2u_, j3_, j4_))
                        if "j2_min" in hc_ and j2_ < hc_["j2_min"]: bad_.append("J2 원시 %.2f < %.1f" % (j2_, hc_["j2_min"]))
                    result["poses"][-1]["height_check"] = {"rule": hc_, "bad": bad_}
                    if bad_: raise Abort("D3 진행 판정 실패(자세 %s): %s. 측정·하강 없이 복귀합니다." % (pid, "; ".join(bad_)))
                hold_for_measure(plan, pid, a, result["poses"][-1], arm)
                if plan.get("p2k") and not plan.get("run2"): p2k_j6_sequence(plan, arm, result["poses"][-1], a, mode)
                if plan.get("run2"): run2_hook(plan, pid, arm, result["poses"][-1], a, mode)
                if plan.get("hold_min_mm"):   # D0 중단 규칙(코드): 입력값이 숫자가 아니거나 하한 미만이면 더 내려가지 않는다(복귀 질문으로 이어짐)
                    rec_ = result["poses"][-1]
                    if a.fake and FAKE_ENTRIES and str(pid) in [str(x_) for x_ in plan.get("hold_poses", [])]: rec_["user_entry"] = FAKE_ENTRIES.pop(0)   # 가짜: 측정 대기가 있는 자세에서만 입력을 소비
                    import re as _re
                    m_ = _re.match(r"\s*(\d+)", str(rec_.get("user_entry") or "")); val_ = int(m_.group(1)) if m_ else None
                    rec_["height_entry_mm"] = val_; mn_ = plan["hold_min_mm"].get(str(pid))
                    if mn_ is not None and (val_ is None or val_ < mn_):
                        rec_["stop_rule"] = "입력 %r(%s) < 하한 %s" % (rec_.get("user_entry"), val_, mn_)
                        raise Abort("D0 중단 규칙: 자세 %s의 입력 %r가 숫자가 아니거나 하한 %s mm 미만이라 더 내려가지 않습니다. FK 높이 오차가 예상보다 클 수 있습니다." % (pid, rec_.get("user_entry"), mn_))
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
                if P2K.get("contact") and P2K.get("ad6c") is not None and mode != "dry_run":   # 닫는 중(블록을 누르는 중) 중단: 팔 복귀 전에 먼저 J6를 연다(들어올림·끌기 방지). 질문 없이 열기만 보낸다(누름 해제, 승인된 열기 3000 ms)
                    say("\n[접촉 중 중단] 팔을 움직이기 전에 J6를 %d로 열어 누름을 풉니다." % plan["run2"]["open_j6"])
                    try:
                        adc_ = P2K["ad6c"]
                        try: adc_.confirm_hands_clear(); time.sleep(QUIET_S); adc_.return_to_start(plan["run2"]["release_ms"])
                        except GripperTestError:   # 이미 열기를 한 번 보낸 어댑터면 새 어댑터로(읽은 값 → 열림, 승인 창·시간 3000 ms)
                            nad = GripperTestAdapter(arm, {"approved": True, "expires": datetime.date.today().isoformat(), "allowed_times_ms": [1500, 3000], "first_move_min_ms": None, "max_writes": 3, "delta_deg": 5, "joint_id": 6, "direction": -1, "angle_window": list(plan["run2"]["window_close"]), "pattern": "list"}, today=datetime.date.today(), dry_run=False)
                            s_ = nad.read_start(); nad.set_targets([s_, plan["run2"]["open_j6"]]); P2K["logs"]["emergency_open"] = nad.log; nad.confirm_hands_clear(); time.sleep(QUIET_S); nad.move(plan["run2"]["open_j6"], plan["run2"]["release_ms"])
                        P2K["cur6"] = plan["run2"]["open_j6"]; P2K["start_all"][6] = plan["run2"]["open_j6"]; P2K["contact"] = False; time.sleep(plan["run2"]["release_ms"] / 1000.0 + 1); result["emergency_open"] = True
                    except BaseException as e_: result["emergency_open_error"] = repr(e_); say("접촉 중 열기 실패:", repr(e_))
                j6_after = bool(P2K.get("low")) and P2K["ad6"] is not None and P2K["cur6"] != P2K["start6"]   # Run 2: 하강 중(블록 옆)에는 J6를 닫지 않는다(닫으면 블록을 물 수 있음) — 팔 복귀(상승) 뒤 K1에서 J6를 시작값으로 되돌린다
                if P2K["ad6"] is not None and P2K["cur6"] != P2K["start6"] and mode != "dry_run" and not j6_after:   # P2k: 팔 복귀 전에 J6를 시작값(오므림)으로 되돌린다(승인 시간 4800 ms 1회)
                    if ask("J6가 시작값 %s가 아닐 수 있습니다(마지막 명령 %s). 팔 자세는 그대로 두고 J6만 시작값으로 복귀 쓰기를 1회 보낼까요? 시간 %d ms (손을 뗀 뒤에만 y)" % (P2K["start6"], P2K["cur6"], plan["p2k"]["return_ms"])):
                        P2K["ad6"].confirm_hands_clear(); time.sleep(QUIET_S); P2K["ad6"].return_to_start(plan["p2k"]["return_ms"]); P2K["cur6"] = P2K["start6"]; P2K["start_all"][6] = P2K["start6"]; result["j6_returned"] = True; time.sleep(plan["p2k"]["return_ms"] / 1000.0 + 1)   # 이후 팔 복귀의 "명령하지 않은 관절" 감시값을 J6 시작값으로 되돌린다
                if ask("팔이 시작 자세가 아닐 수 있습니다. 계획된 상태별 복귀 경로(J1·J2 → 펴기 계단, 한 번에 한 관절, 각 3000 ms 이하)로 직립까지 복귀할까요? (손·도구를 뗀 뒤에만 y)"):
                    do_return(plan, {"ads": ads}, arm, cmd, start_all, state, result, PROG["sent"], a, mode, normal=False)
                    if j6_after and mode != "dry_run" and result.get("returned_to_start") and ask("팔이 K1로 돌아왔습니다. J6를 시작값 %s(오므림)으로 되돌릴까요? 시간 %d ms (손을 뗀 뒤에만 y)" % (P2K["start6"], plan["p2k"]["return_ms"])):
                        P2K["ad6"].confirm_hands_clear(); time.sleep(QUIET_S); P2K["ad6"].return_to_start(plan["p2k"]["return_ms"]); P2K["cur6"] = P2K["start6"]; P2K["start_all"][6] = P2K["start6"]; result["j6_returned"] = True; time.sleep(plan["p2k"]["return_ms"] / 1000.0 + 1)   # 이후 팔 복귀의 "명령하지 않은 관절" 감시값을 J6 시작값으로 되돌린다
                else: result["returned_to_start"] = False
            except BaseException as e: result["return_error"] = repr(e); say("복귀 처리 중단:", repr(e))
        result["adapter_log"] = {str(j): [(["write", *l[1:3], "fake" if (a.fake and l[0] == "write") else l[3], *l[4:]] if l[0] == "write" else list(l)) for l in ads[j].log] for j in ads if hasattr(ads[j], "log")}
        if P2K["ad6"] is not None: result["j6_adapter_log"] = [list(l) for l in P2K["ad6"].log]
        if P2K["logs"]: result["j6_adapter_logs"] = {k: [list(l) for l in v] for k, v in P2K["logs"].items()}   # Run 2: 열기·접촉·최종 닫기 어댑터 로그를 따로 저장
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
