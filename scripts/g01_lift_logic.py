"""들어올림(P4b-1, 세트 33) 판정·시계·계산 로직. 하드웨어와 무관한 순수 함수(러너 훅·래퍼·후처리가 import한다).
근거: docs/261118-1038-들어올림-러너-구현-플랜.md §2(R1~R12), §2-1(시계표), §4(승인 P0~P9). 상수는 g01_lift_params.PARAMS 한 곳.
미확정(UNSET) 값을 쓰면 g01_lift_params.ParamsNotFinal(fail-closed)."""
from g01_lift_params import PARAMS, ParamsNotFinal

EPS = 1e-9   # 판정 문턱(E1·E3·G4·G5·Q3)의 부동소수 경계 오차 허용. 안전 시계(PressClock)에는 두지 않는다(경계를 넘으면 시작하지 않는 쪽이 안전).
CLOSE_MS = 1500          # 닫기 계단 단계 시간(plan close_ms, g01_s42_plan_p2k.py)
QUIET_S, POLL_S = 0.3, 0.1   # 쓰기 전 조용한 시간, 폴링 간격(p2k_test.py QUIET_S·POLL_MS)
WATCH_TAIL_S = 0.1       # 이동마다 J5·J6 읽기 등 여유(플랜 §2-1 단계 상한)


# ---------- 시계(R2, P3) ----------
# 읽기 간격 가드(g01_read_guard, 플랜 v2 §5)의 단계별 대기 상한(초): 읽기 시간 0인 가짜 팔·가상 시계로 구한 단계별 최대 대기(lift1 0.2, lift2 0.2, lift3 0.3, photo_c 0.1)에
# `read_start`·`before` 재시도 최악(입구 2개 × 추가 2회 × 0.05 s = 0.2 s, 이동 단계만)을 더한 값. 새 간격 값이 아니라 기존 0.05 s 가드의 최악 누적이며, tests/test_lift_read_guard.py가 시뮬레이션 최대치 ≤ 이 값을 단언한다.
GUARD_WAIT_UB_S = {"lift1": 0.4, "lift2": 0.4, "lift3": 0.5, "photo_c": 0.1}


def step_upper_bounds(P=None):
    """누름 창의 단계 상한(초, 코드 상수). 순서 = 실행 순서."""
    P = PARAMS if P is None else P
    watch = (1.5 * P["move_time_ms"] + 1000) / 1000.0
    mv = P["move_countdown_s"] + QUIET_S + watch + WATCH_TAIL_S
    pw = P["photo_wait_max_s"]; g = GUARD_WAIT_UB_S
    return [("close145", (2 * CLOSE_MS + 1000) / 1000.0 + POLL_S + QUIET_S), ("stall150", (2 * CLOSE_MS + 1000) / 1000.0 + POLL_S),
            ("photo_a", pw), ("lift1", mv + g["lift1"]), ("lift2", mv + g["lift2"]), ("lift3", mv + g["lift3"]), ("photo_b", pw), ("hold", P["hold_s"]), ("photo_c", pw + g["photo_c"])]


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
def second_j2_command(first_target, j2_start_deg, j2_after_first_deg, P=None):
    """두 번째 J2 절대 명령(SDK 정수 각도). 승인 공식 inc = clip(8 − 실제 올라간 각도 + 1.5, 3, 6)을 **첫 이동 뒤 실제 각도에 더하고**(FK `follow()` 모델,
    E-2b `:117`), 접촉 자세마다 생성기가 계산한 최대 정수 목표 lift_j2_max_target(최악 tilt ≤ 30°, UNSET이면 ParamsNotFinal)으로 자른다:
    target = min(half_up(실제 + inc), max_target). 첫 명령이 이미 max_target을 넘으면 ValueError. target이 첫 이동 뒤 실제 각도보다 1° 이상 높지 않으면
    None(두 번째 이동 없음; 처리 명세는 M2). 서보 최소 유효 폭(세트 16: 증가 방향 +2° 불발 이력)은 이 함수가 아니라 도착 검사(run_move TOL)가 잡는다.
    각도는 원시 읽기(실수)."""
    P = PARAMS if P is None else P
    mx = P["lift_j2_max_target"]                           # UNSET이면 여기서 ParamsNotFinal
    if first_target > mx: raise ValueError("첫 J2 명령이 최대 목표(lift_j2_max_target)를 넘는다")
    up = j2_after_first_deg - j2_start_deg; inc_f = min(6.0, max(3.0, 8.0 - up + 1.5))
    t_f = int(j2_after_first_deg + inc_f + 0.5); target = min(t_f, int(mx))
    return (target if target - j2_after_first_deg >= 1.0 - EPS else None), {"up_actual": up, "inc_formula": inc_f, "target_formula": t_f, "target_max": int(mx)}


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


def sag_cap(plan_max, margin, contact_q, read, refuse_at_or_below, first_target):
    """P10 처짐 cap(사용자 결정 2026-11-18). 정점 tilt = t2 + J3 + J4 + c(민감도 1, Mac 자체 점검)이므로 같은 30° 규칙의 풀이는
    cap = floor(plan_max + margin − (ΔJ3 + ΔJ4)), margin = 30° − 접촉 원시 q에서의 plan_max 정점, Δ = 이번 읽기 − 접촉 원시.
    cap ≤ refuse_at_or_below 또는 cap < first_target이면 거부. 입력(margin·접촉 q·J3/J4 읽기) 중 하나라도 없으면 거부(fail-closed). 반환 dict."""
    import math
    try:
        d3 = read[3] - contact_q[2]; d4 = read[4] - contact_q[3]; mg = float(margin)
    except Exception: return {"cap": None, "refuse": True, "reason": "입력 없음(margin·접촉 q·J3/J4 읽기)", "margin": margin}
    delta = d3 + d4; cap = min(int(plan_max), int(math.floor(plan_max + mg - delta + 1e-9)))
    refuse = cap <= refuse_at_or_below or cap < first_target
    return {"cap": cap, "refuse": refuse, "delta_deg": round(delta, 3), "d3": round(d3, 3), "d4": round(d4, 3), "margin": mg, "plan_max": int(plan_max), "reason": ("cap %d ≤ %d(E1 선) 또는 첫 목표 %d 미만" % (cap, refuse_at_or_below, first_target)) if refuse else ""}


def e1_attempt_label(e1, lift):
    """E1 불합격 라벨(P10 조건 ③): 두 번째 J2 이동이 생략·무반응이었던 시도의 E1 불합격은 '파지 실패'가 아니라 '신호 부족 가능'일 수 있다고 표시한다(기록 전용)."""
    sj = (lift or {}).get("second_j2") or {}
    late = (((lift or {}).get("sag") or {}).get("late")) or {}
    capped = late.get("cap") is not None and late.get("plan_max") is not None and late["cap"] < late["plan_max"]      # 처짐 cap으로 둘째 목표가 낮아진 시도(P10)
    weak = bool(sj.get("skipped_reason") or sj.get("no_response") or capped)
    return dict(e1, signal_shortfall_possible=bool(weak and e1.get("verdict") == "불합격"), note=("두 번째 J2 이동 생략/무반응/처짐 cap: E1 신호 약함 가능 — 들어올림이 덜 되어 E1 불합격은 '쥠 실패'가 아니라 신호 부족일 수 있다(파지 실패의 증거로 쓰지 않는다)" if weak and e1.get("verdict") == "불합격" else ""))


# ---------- E1·E3 (R8, E-2 :28·:98~100, E-7) ----------
def e1_decide(a, b, P=None):
    """a·b: detect.json(center, side_px). 위 = 영상 y 감소. (i) 위 이동 ≥ 8 px, (ii) 변 길이 증가 ≥ +0.7 px, (iii) 옆 ≤ 4 px."""
    P = PARAMS if P is None else P
    if not (a.get("found") and b.get("found")): return {"verdict": "판정 불가", "reason": "마커 미검출"}
    up = a["center"][1] - b["center"][1]; side = b["side_px"] - a["side_px"]; lat = abs(b["center"][0] - a["center"][0])
    i, ii, iii = up >= P["e1_center_up_px"] - EPS, side >= P["e1_side_len_px"] - EPS, lat <= P["e1_lateral_px"] + EPS
    return {"verdict": "합격" if (i and ii and iii) else "불합격", "up_px": up, "side_delta_px": side, "lateral_px": lat, "i": i, "ii": ii, "iii": iii}


def e3_verdict(j6_after_close, j6_after_lift, hold_done, photo_c_done, gemini_pair=None, P=None):
    """P5(조건부 승인): 유지까지 생략 → 예외 없이 E-2 원래 규칙(판정 불가=실패). 사진 C만 생략 → J6 회독판(±1°)만으로 판정.
    gemini_pair = (B 검출, C 검출): 사진 C가 있으면 중심·변 길이 ±1 px도 함께 본다(E-2 :100)."""
    P = PARAMS if P is None else P
    if not hold_done: return {"verdict": "판정 불가", "counts_as": "실패", "rule": "E-2 원래 규칙(유지 생략)"}
    if j6_after_close is None or j6_after_lift is None: return {"verdict": "판정 불가", "counts_as": "실패", "rule": "J6 회독 없음"}
    j6_ok = abs(j6_after_lift - j6_after_close) <= 1 + EPS
    if not photo_c_done: return {"verdict": "합격" if j6_ok else "불합격", "rule": "사진 C 생략: J6 회독판만", "j6_ok": j6_ok, "gemini": "생략"}
    b, c = gemini_pair
    if not (b.get("found") and c.get("found")): return {"verdict": "판정 불가", "counts_as": "실패", "rule": "사진 C 마커 미검출", "j6_ok": j6_ok}
    import math   # P11 ②(확정): 중심은 2D 거리 ≤ 1 px, 변 길이는 |Δ| ≤ 1 px
    dxc, dyc = c["center"][0] - b["center"][0], c["center"][1] - b["center"][1]
    cen = math.hypot(dxc, dyc) if P["e3_gemini_center_metric"] == "dist" else max(abs(dxc), abs(dyc))
    g_ok = cen <= 1 + EPS and abs(c["side_px"] - b["side_px"]) <= 1 + EPS
    return {"verdict": "합격" if (j6_ok and g_ok) else "불합격", "rule": "J6 회독판 + Gemini판", "j6_ok": j6_ok, "gemini_ok": g_ok}


# ---------- S4' 관문(G1~G5) ----------
def s4_gate(ref, a, dx_block_mm=None, dx_depth_mm=None, P=None):
    """ref·a: 기준 사진·사진 A 검출. dx_block_mm = Δx_block(프로젝트 x, mm) = 사전 등록 `p2k_analyze_261117.inv_P`(높이 0.030 m 평면 역투영)로 A·기준의
    마커 중심을 변환한 x 차이(부호·환산은 그 사전 등록 코드가 정의, 호출부가 계산). dx_depth_mm = 정렬 보고의 d_x(깊이). G3은 기록만.
    불감대 밖인데 두 입력 중 하나라도 없으면 ParamsNotFinal(조용한 생략 금지). 불감대 이동 지표(x/2D)는 UNSET이면 쓸 때 ParamsNotFinal."""
    import math
    P = PARAMS if P is None else P
    if not (a.get("found") and ref.get("found")): return {"go": False, "g1": False, "reason": "G1 마커 미검출"}
    dxp = a["center"][0] - ref["center"][0]; dyp = a["center"][1] - ref["center"][1]
    g4 = abs(a["side_px"] - ref["side_px"]) <= P["g4_side_px"] + EPS
    dyaw = (a["yaw_deg"] - ref["yaw_deg"] + 180.0) % 360.0 - 180.0; g5 = abs(dyaw) <= P["g5_yaw_deg"] + EPS
    metric = P["g2_deadzone_metric"]                       # UNSET이면 여기서 ParamsNotFinal
    shift = abs(dxp) if metric == "x" else math.hypot(dxp, dyp)
    if shift <= P["g2_deadzone_px"] + EPS: g2 = True
    else:
        if dx_block_mm is None: raise ParamsNotFinal("G2 불감대 밖인데 Δx_block 입력이 없다(조용한 생략 금지)")
        if dx_depth_mm is not None: lo, hi = dx_depth_mm + P["g2_s_range_mm"][0], dx_depth_mm + P["g2_s_range_mm"][1]
        else:   # d_x(깊이) 미상(세트 33): 정렬 허용 ±bound로 보고 구간을 잡는다. 0을 걸치면 아래에서 방향 판정이 생략된다(방향 검사 생략은 승인 기록에 적는다)
            if P["g2_dx_depth_bound_mm"] is None: raise ParamsNotFinal("d_x(깊이) 입력도 가정 범위(g2_dx_depth_bound_mm)도 없다")
            b = P["g2_dx_depth_bound_mm"]; lo, hi = -b + P["g2_s_range_mm"][0], b + P["g2_s_range_mm"][1]
        x_out = abs(dxp) > P["g2_deadzone_px"] + EPS           # 2D 지표에서 y 이동만으로 불감대를 벗어나면 x 부호는 잡음이라 방향 판정을 하지 않는다(S4' G3: y는 기록만)
        sign_ok = True if (lo * hi <= 0 or not x_out) else ((dx_block_mm > 0) == (lo > 0))   # 예상 구간이 0을 걸치거나 x가 불감대 안이면 방향 판정 생략
        g2 = sign_ok and abs(dx_block_mm) <= P["g2_magnitude_mm"] + EPS
    out = {"go": bool(g2 and g4 and g5), "g1": True, "g2": g2, "g3_dy_px": dyp, "g4": g4, "g5": g5, "dyaw_deg": dyaw, "dx_block_mm": dx_block_mm, "dx_depth_assumed": dx_depth_mm is None and dx_block_mm is not None}
    if dx_block_mm is not None: out["near"] = abs(dx_block_mm) >= P["g2_near_mm"] - EPS      # 기록 전용 표시(P["g2_near_mm"])
    return out


# ---------- Q3 (R8, P9) ----------
def base_shift_fresh(a_bgr, b_bgr, box):
    """호출마다 새 배열을 만들어 phaseCorrelate에 넘긴다(입력 제자리 수정·순서 의존 방지, 이슈보고서 261118-1305). 반환 (dx, dy, response)."""
    import cv2, numpy as np
    x0, y0, x1, y1 = box
    a = np.ascontiguousarray(cv2.cvtColor(a_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)[y0:y1, x0:x1])
    b = np.ascontiguousarray(cv2.cvtColor(b_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)[y0:y1, x0:x1])
    (dx, dy), r = cv2.phaseCorrelate(a, b, cv2.createHanningWindow((x1 - x0, y1 - y0), cv2.CV_32F)); return float(dx), float(dy), float(r)


OLD_BASE_BOX = (480, 0, 900, 100)   # E-0G 사전 등록 BASE_BOX(g01_e0g_analyze.py:18), P9 조건 ②: 원값 기록용


def region_zncc(a_bgr, b_bgr, box):
    """영역(box) 안 회색조 ZNCC(제로평균 정규화 상관). 한쪽이 상수 영상이면 0.0(상관 정의 불가 → 판정 불가 쪽으로 보수적). 읽기 전용 복사본만 쓴다."""
    import cv2, numpy as np
    x0, y0, x1, y1 = box
    a = cv2.cvtColor(a_bgr, cv2.COLOR_BGR2GRAY).astype(np.float64)[y0:y1, x0:x1].ravel(); b = cv2.cvtColor(b_bgr, cv2.COLOR_BGR2GRAY).astype(np.float64)[y0:y1, x0:x1].ravel()
    a = a - a.mean(); b = b - b.mean(); d = float(np.sqrt((a * a).sum() * (b * b).sum()))
    return 0.0 if d < 1e-12 else float((a * b).sum() / d)


def q3_record(a_bgr, b_bgr, P=None, k1_bgr=None):
    """사진 A·B에서 현행 BASE_BOX와 확정 영역의 이동·응답 원값을 모두 기록한다(P9 조건 ②). 확정 영역이 UNSET이면 ParamsNotFinal.
    k1_bgr(S4' 기준 사진)이 있으면 확정 영역의 ZNCC(K1–A, K1–B, A–B) 원값도 기록한다(P9 조건 ③, 없으면 zncc=None).
    shift_resp_ok는 원값 판정일 뿐 Q3 판정이 아니다(조건 ③은 q3_verdict)."""
    P = PARAMS if P is None else P
    box = tuple(P["q3_box"])                               # UNSET이면 여기서 ParamsNotFinal
    old = base_shift_fresh(a_bgr, b_bgr, OLD_BASE_BOX); new = base_shift_fresh(a_bgr, b_bgr, box)
    z = None if k1_bgr is None else {"k1_a": region_zncc(k1_bgr, a_bgr, box), "k1_b": region_zncc(k1_bgr, b_bgr, box), "a_b": region_zncc(a_bgr, b_bgr, box)}
    return {"old_box": list(OLD_BASE_BOX), "old": old, "box": list(box), "new": new, "zncc": z,
            "shift_resp_ok": new[2] >= P["q3_response_min"] - EPS and abs(new[0]) <= P["q3_shift_max_px"] + EPS and abs(new[1]) <= P["q3_shift_max_px"] + EPS}


def q3_verdict(record, P=None):
    """Q3 판정(P9 조건 ③, 사용자 2026-11-18): 영역 ZNCC 세 쌍의 최솟값 < q3_zncc_min이면 '판정 불가'(실패로 셈, 원값은 record에 남음).
    ZNCC 원값이 없으면(K1 사진 없음) 팔 진입을 배제할 수 없으므로 판정 불가. 아니면 기존 이동·응답 규칙. 문턱이 UNSET이면 ParamsNotFinal."""
    P = PARAMS if P is None else P
    thr = P["q3_zncc_min"]                                 # UNSET이면 여기서 ParamsNotFinal
    z = record.get("zncc")
    if z is None: return {"verdict": "판정 불가", "counts_as": "실패(E-2 규칙)", "reason": "영역 ZNCC 원값 없음(K1 사진 없음)"}
    mn = min(z.values())
    if mn < thr - EPS: return {"verdict": "판정 불가", "counts_as": "실패(E-2 규칙)", "reason": "영역 ZNCC 최솟값 %.3f < %.3f (P9 ③)" % (mn, thr), "zncc_min": mn}
    return {"verdict": "합격" if record["shift_resp_ok"] else "불합격", "zncc_min": mn}


# ---------- 기록 직렬화 ----------
def params_jsonable(P=None):
    """결과 JSON에 PARAMS를 남길 때 UNSET·튜플이 json.dumps에서 깨지지 않게 변환한다(함수는 이름만)."""
    P = PARAMS if P is None else P
    def f(v):
        from g01_lift_params import _Unset
        if isinstance(v, _Unset): return "UNSET"
        if isinstance(v, (tuple, list)): return [f(x) for x in v]
        if callable(v): return getattr(v, "__name__", "callable")
        return v
    return {k: f(v) for k, v in P.items()}
