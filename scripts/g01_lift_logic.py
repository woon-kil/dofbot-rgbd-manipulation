"""들어올림(P4b-1, 세트 33) 판정·시계·계산 로직. 하드웨어와 무관한 순수 함수(러너 훅·래퍼·후처리가 import한다).
근거: docs/261118-1038-들어올림-러너-구현-플랜.md §2(R1~R12), §2-1(시계표), §4(승인 P0~P9). 상수는 g01_lift_params.PARAMS 한 곳.
미확정(UNSET) 값을 쓰면 g01_lift_params.ParamsNotFinal(fail-closed)."""
from g01_lift_params import PARAMS, ParamsNotFinal

CLOSE_MS = 1500          # 닫기 계단 단계 시간(plan close_ms, g01_s42_plan_p2k.py)
QUIET_S, POLL_S = 0.3, 0.1   # 쓰기 전 조용한 시간, 폴링 간격(p2k_test.py QUIET_S·POLL_MS)
WATCH_TAIL_S = 0.1       # 이동마다 J5·J6 읽기 등 여유(플랜 §2-1 단계 상한)


# ---------- 시계(R2, P3) ----------
def step_upper_bounds(P=None):
    """누름 창의 단계 상한(초, 코드 상수). 순서 = 실행 순서."""
    P = PARAMS if P is None else P
    watch = (1.5 * P["move_time_ms"] + 1000) / 1000.0
    mv = P["move_countdown_s"] + QUIET_S + watch + WATCH_TAIL_S
    pw = P["photo_wait_max_s"]
    return [("close145", (2 * CLOSE_MS + 1000) / 1000.0 + POLL_S + QUIET_S), ("stall150", (2 * CLOSE_MS + 1000) / 1000.0 + POLL_S),
            ("photo_a", pw), ("lift1", mv), ("lift2", mv), ("lift3", mv), ("photo_b", pw), ("hold", P["hold_s"]), ("photo_c", pw)]


class PressClock:
    """145 쓰기부터 센다. 각 단계는 '경과 + 단계 상한 ≤ 한도'일 때만 시작한다(아니면 이후 전부 생략, 즉시 열기)."""
    def __init__(self, now_fn, P=None):
        self.P = PARAMS if P is None else P; self.now = now_fn; self.t0 = None; self.skipped_at = None; self.log = []
    def start(self): self.t0 = self.now()
    def elapsed(self): return self.now() - self.t0
    def may_start(self, step, ub):
        if self.skipped_at is not None: return False
        el = self.elapsed(); ok = el + ub <= self.P["clock_limit_s"]
        self.log.append((step, round(el, 3), ub, ok))
        if not ok: self.skipped_at = step
        return ok


def simulate(actuals, P=None):
    """단계별 실제 소요(초)를 받아 시계 규칙 결과를 계산한다(플랜 §2-1 표 검증용). 반환: (생략된 첫 단계 또는 None, 최종 경과)."""
    ubs = step_upper_bounds(P); t = [0.0]; c = PressClock(lambda: t[0], P); c.start()
    for (name, ub), a in zip(ubs, actuals):
        if not c.may_start(name, ub): return c.skipped_at, t[0]
        t[0] += a
    return None, t[0]


# ---------- 두 번째 J2 명령(R1, E-2b §9-1) ----------
def second_j2_command(first_target, j2_start_deg, j2_after_first_deg):
    """두 번째 J2 절대 명령 = 첫 명령 + round(clip(8 − 실제 올라간 각도 + 1.5, 3, 6)). 각도는 원시 읽기(실수)."""
    up = j2_after_first_deg - j2_start_deg; inc = min(6.0, max(3.0, 8.0 - up + 1.5))
    return int(first_target) + int(inc + 0.5), inc          # 반올림은 half-up(파이썬 round의 짝수 반올림을 쓰지 않는다)


def shortfall_abort(commanded, readback_int, tol=3):
    """보정 이동은 하지 않는다(R1): 정수 회독이 명령과 tol(=run_move TOL)를 넘게 다르면 중단·비상 열기."""
    return readback_int is None or abs(commanded - readback_int) > tol


# ---------- 같은 기동·경과 점검(R5, R6, P1) ----------
def same_start(epochs):
    """기준·A·B·C(·복귀 뒤) 사진의 started_epoch가 모두 같아야 한다. 아니면 '판정 없음'."""
    e = [x for x in epochs if x is not None]
    return len(e) == len(epochs) and len(set(e)) == 1


def pa_elapsed_ok(elapsed_s, maxsec, P=None):
    P = PARAMS if P is None else P
    return elapsed_s <= maxsec - P["pa_elapsed_margin_s"]


# ---------- E1·E3 (R8, E-2 :28·:98~100, E-7) ----------
def e1_decide(a, b, P=None):
    """a·b: detect.json(center, side_px). 위 = 영상 y 감소. (i) 위 이동 ≥ 8 px, (ii) 변 길이 증가 ≥ +0.7 px, (iii) 옆 ≤ 4 px."""
    P = PARAMS if P is None else P
    if not (a.get("found") and b.get("found")): return {"verdict": "판정 불가", "reason": "마커 미검출"}
    up = a["center"][1] - b["center"][1]; side = b["side_px"] - a["side_px"]; lat = abs(b["center"][0] - a["center"][0])
    EPS = 1e-9   # 경계값(예: 변 길이 +0.7 px)의 부동소수 오차 허용
    i, ii, iii = up >= P["e1_center_up_px"] - EPS, side >= P["e1_side_len_px"] - EPS, lat <= P["e1_lateral_px"] + EPS
    return {"verdict": "합격" if (i and ii and iii) else "불합격", "up_px": up, "side_delta_px": side, "lateral_px": lat, "i": i, "ii": ii, "iii": iii}


def e3_verdict(j6_after_close, j6_after_lift, hold_done, photo_c_done, gemini_pair=None, P=None):
    """P5(조건부 승인): 유지까지 생략 → 예외 없이 E-2 원래 규칙(판정 불가=실패). 사진 C만 생략 → J6 회독판(±1°)만으로 판정.
    gemini_pair = (B 검출, C 검출): 사진 C가 있으면 중심·변 길이 ±1 px도 함께 본다(E-2 :100)."""
    P = PARAMS if P is None else P
    if not hold_done: return {"verdict": "판정 불가", "counts_as": "실패", "rule": "E-2 원래 규칙(유지 생략)"}
    if j6_after_close is None or j6_after_lift is None: return {"verdict": "판정 불가", "counts_as": "실패", "rule": "J6 회독 없음"}
    j6_ok = abs(j6_after_lift - j6_after_close) <= 1
    if not photo_c_done: return {"verdict": "합격" if j6_ok else "불합격", "rule": "사진 C 생략: J6 회독판만", "j6_ok": j6_ok, "gemini": "생략"}
    b, c = gemini_pair
    if not (b.get("found") and c.get("found")): return {"verdict": "판정 불가", "counts_as": "실패", "rule": "사진 C 마커 미검출", "j6_ok": j6_ok}
    g_ok = abs(c["center"][0] - b["center"][0]) <= 1 and abs(c["center"][1] - b["center"][1]) <= 1 and abs(c["side_px"] - b["side_px"]) <= 1
    return {"verdict": "합격" if (j6_ok and g_ok) else "불합격", "rule": "J6 회독판 + Gemini판", "j6_ok": j6_ok, "gemini_ok": g_ok}


# ---------- S4' 관문(G1~G5) ----------
def s4_gate(ref, a, dx_depth_mm=None, P=None):
    """ref·a: 기준 사진·사진 A 검출. G3은 기록만. G2의 px→mm 환산·부호 규약은 UNSET(정독 전)이라 쓰면 ParamsNotFinal."""
    P = PARAMS if P is None else P
    if not (a.get("found") and ref.get("found")): return {"go": False, "g1": False, "reason": "G1 마커 미검출"}
    dxp = a["center"][0] - ref["center"][0]; dyp = a["center"][1] - ref["center"][1]
    g4 = abs(a["side_px"] - ref["side_px"]) <= P["g4_side_px"]
    dyaw = (a["yaw_deg"] - ref["yaw_deg"] + 180.0) % 360.0 - 180.0; g5 = abs(dyaw) <= P["g5_yaw_deg"]
    if abs(dxp) <= P["g2_deadzone_px"]: g2 = True
    else:
        mm = dxp * P["g2_mm_per_px"] * P["g2_x_sign"]          # UNSET이면 여기서 ParamsNotFinal
        lo, hi = (dx_depth_mm + P["g2_s_range_mm"][0], dx_depth_mm + P["g2_s_range_mm"][1]) if dx_depth_mm is not None else (None, None)
        sign_ok = True if (lo is None or lo * hi <= 0) else ((mm > 0) == (lo > 0))   # 예상 구간이 0을 걸치면 방향 판정 생략
        g2 = sign_ok and abs(mm) <= P["g2_magnitude_mm"]
    return {"go": bool(g2 and g4 and g5), "g1": True, "g2": g2, "g3_dy_px": dyp, "g4": g4, "g5": g5, "dyaw_deg": dyaw}


# ---------- Q3 (R8, P9) ----------
def base_shift_fresh(a_bgr, b_bgr, box):
    """호출마다 새 배열을 만들어 phaseCorrelate에 넘긴다(입력 제자리 수정·순서 의존 방지, 이슈보고서 261118-1305). 반환 (dx, dy, response)."""
    import cv2, numpy as np
    x0, y0, x1, y1 = box
    a = np.ascontiguousarray(cv2.cvtColor(a_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)[y0:y1, x0:x1])
    b = np.ascontiguousarray(cv2.cvtColor(b_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)[y0:y1, x0:x1])
    (dx, dy), r = cv2.phaseCorrelate(a, b, cv2.createHanningWindow((x1 - x0, y1 - y0), cv2.CV_32F)); return float(dx), float(dy), float(r)


OLD_BASE_BOX = (480, 0, 900, 100)   # E-0G 사전 등록 BASE_BOX(g01_e0g_analyze.py:18), P9 조건 ②: 원값 기록용


def q3_record(a_bgr, b_bgr, P=None):
    """사진 A·B에서 현행 BASE_BOX와 확정 영역의 이동·응답 원값을 모두 기록한다(P9 조건 ②). 확정 영역이 UNSET이면 ParamsNotFinal."""
    P = PARAMS if P is None else P
    box = tuple(P["q3_box"])                               # UNSET이면 여기서 ParamsNotFinal
    old = base_shift_fresh(a_bgr, b_bgr, OLD_BASE_BOX); new = base_shift_fresh(a_bgr, b_bgr, box)
    return {"old_box": list(OLD_BASE_BOX), "old": old, "box": list(box), "new": new,
            "ok": new[2] >= P["q3_response_min"] and abs(new[0]) <= P["q3_shift_max_px"] and abs(new[1]) <= P["q3_shift_max_px"]}
