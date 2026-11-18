"""들어올림(P4b-1, 세트 33) 러너 상수 블록. 러너 플랜 docs/261118-1038-들어올림-러너-구현-플랜.md §4·§4-2.
규칙(사용자 2026-11-18): 미확정 상수는 기본값 없이 UNSET으로 두고, 하나라도 UNSET이면 --real을 거부한다(fail-closed).
확정 값은 근거 승인 번호를 주석으로 단다. 값 변경은 해당 승인 항목의 재승인이 필요하다."""


class ParamsNotFinal(Exception):
    """미확정 상수가 남아 있어 --real을 거부한다(UNSET을 값으로 쓰려 해도 이 예외)."""


class _Unset:
    """값으로 쓰면 모두 예외: `PARAMS[k] or 기본값` 같은 조용한 대체를 막는다(사용자 결정: 기본값 없음)."""
    def __repr__(self): return "UNSET"
    def _no(self, *a, **k): raise ParamsNotFinal("미확정 상수(UNSET)를 값으로 쓸 수 없다")
    __bool__ = __eq__ = __ne__ = __lt__ = __le__ = __gt__ = __ge__ = _no
    __call__ = __add__ = __radd__ = __sub__ = __mul__ = __rmul__ = __truediv__ = __getitem__ = __iter__ = __len__ = __int__ = __float__ = __index__ = __contains__ = _no
    __hash__ = object.__hash__


UNSET = _Unset()


PARAMS = {
    # --- 확정(사용자 승인 2026-11-18) ---
    "clock_start": "145_write",             # P3: 누름 시계는 145 쓰기부터
    "clock_limit_s": 26.8,                  # P3: 29.8 s(P-B 검증, 145 기준) × 0.9. 이 세트에서 '정체 후 3 s 안 열기'를 대체
    "move_countdown_s": 0,                  # P6: 누름 창 안 이동 카운트다운 0 s, 이동 직전 소리는 유지
    "move_time_ms": 1500,                   # 플랜 R1: 들어올림 이동 시간(어댑터 허용 최소). 바꾸면 재승인(사람 질문 분기가 창 안에 들어옴)
    "photo_wait_max_s": 3.0,                # P2: 사진 한 장 대기 상한, 초과 시 판정 없음·즉시 열기
    "pa_elapsed_margin_s": 300,             # P1: 접촉 닫기 직전 경과 ≤ MAXSEC − 300 s
    "j2_window": (66, 110),                 # P8: J2 각도 창(세트 33 한정, 세트 31 선례)
    "first_move_min_ms": None,              # P7: 명시적 None = 첫 이동 8000 ms 규칙을 이 새 어댑터에 적용하지 않음(UNSET과 다름)
    "photo_c_skip_e3": "j6_readback_only",  # P5: 사진 C만 생략하면 E3 Gemini판 '생략', J6 회독판만으로 판정
    "hold_skip_e3": "e2_original",          # P5: 유지까지 생략되면 예외 없이 E-2 원래 규칙(판정 불가=실패)
    "hold_s": 3.0,                          # P5 승인 범위: 유지 ≤ 3 s(E-2b F4)
    "photo_c_included": True,               # P5: 유지 확인 사진 C 포함(E-2 :100, E1-R)
    "return_photo_use": "record_only",      # P4 조건: 복귀 뒤 사진은 기록용만(P9에서 (라)를 택하면 재승인)
    # --- E-2 사전 등록 값(E-7 포함) ---
    "e1_center_up_px": 8.0, "e1_side_len_px": 0.7, "e1_lateral_px": 4.0,
    "q3_shift_max_px": 2.0, "q3_response_min": 0.3,
    "lift_j2_refuse_at_or_below": 75,       # P10 처짐 cap(사용자 결정 2026-11-18): 둘째 J2 상한 cap이 이 값 이하(E1 선 아래)이면 들어올림을 거부(시도 + no_lift, 처짐 과다). 한계: 75는 부족 0의 Q0 Δz 표(13.7 mm)에서 왔고, 실제 부족 1.1~2.0°에서는 cap 76도 Δz가 14.5 mm 미만이다(참고용 표, 플랜 §3-2). 사용자가 76으로 올릴지 정한다. Mac 재확인 ok와 훅이 같은 값을 쓴다
    "tilt_selfcheck_tol": 0.01,             # 모델 자체 점검(안전 문턱 아님): t2·J3·J4 정점 민감도가 1에서 이만큼 넘게 벗어나면 margin을 쓰지 않고 거부
    "lift_j2_min_effective_step_deg": 3.5,  # 서보 위(증가) 방향 최소 유효 폭(세트 16, docs/261115-1347-소각응답-결과보고서.md :3·:19: 요청 1.5~2.5° J2 3/3 움직임 없음, ≥ 3.5°에서 실현 비율 ≈ 0.5). 두 번째 J2 요청 폭이 이보다 작으면 보내지 않고 기록(P10 조건 ①)
    "lift_j2_no_response_deg": 0.5,         # **기록 전용 라벨, 안전 값 아님(근거 없는 숫자)**: 두 번째 J2 이동 뒤 원시 변화가 이보다 작으면 no_response로 기록(제어 흐름은 같다: 어느 경우든 J4로 진행, 재전송 없음)
    # --- S4' 관문 승인 값(E-2b `:14` 승인됨, S4' 문서 §3 `:22~28`) ---
    "g2_deadzone_px": 0.75, "g2_s_range_mm": (-1.26, -0.77), "g2_magnitude_mm": 3.0, "g2_dx_depth_bound_mm": 3.0, "g2_near_mm": 2.5,   # 사용자 결정(2026-11-18, 선택지 A): 세트 33은 d_x(깊이)를 실행 중 얻을 수 없어 정렬 허용값 ±3 mm로 보고(예상 구간이 0을 걸쳐 방향 판정 생략, 크기 ≤ 3.0 mm만 봄). g2_near_mm은 기록 전용 라벨(|Δx| ≥ 2.5면 '근접' 표시, 문턱은 실행 중 바꾸지 않음)
    "g4_side_px": 0.75, "g5_yaw_deg": 3.0,
    # --- 미확정(기본값 없음, 사용자 확인 필요) ---
    "g2_deadzone_metric": "x",              # P11 ①(사용자 2026-11-18 확정): G2 불감대 '중심 이동 ≤ 0.75 px'는 x 이동(y는 G3에서 기록만, 2D는 y만 움직여도 무작위 no-go)
    "e3_gemini_center_metric": "dist",      # P11 ②(사용자 2026-11-18 확정): E3 Gemini판 중심은 2D 거리 ≤ 1 px
    "lift_j2_max_target": UNSET,            # R1/P10(사용자 2026-11-18 77 원칙형 확정, 값은 접촉 자세마다 생성기가 계산해 계획 JSON으로 공급 → M2에서 계획 로드 시 채움, 그 전에는 --real 거부): 접촉 자세마다 생성기가 계산하는 'FK 최악(부족 0, J4 넘침 0.8) 정점 tilt ≤ 30°인 최대 정수 J2 절대 목표'. Q0(시작 68.32, 첫 명령 74)에서 76 28.8°, 77 29.8°, 78 30.8° → 77. 사용자 결정(P10)
    # --- 미확정(P9 확정 전, 기본값 없음) ---
    "q3_box": (800, 0, 900, 100),          # P9 결정 2(사용자 2026-11-18): 확정 등록. E-2의 '베이스 상부' 정의를 '고정 구조(어댑터·나무 상판), 카메라 흔들림만 검출'로 바꾸는 규칙 변경(옛 BASE_BOX (480,0,900,100)은 원값 기록용으로 병행)
    "q3_zncc_min": 0.90,                    # P9 결정 1(사용자 2026-11-18): 사전 등록. 영역 ZNCC(K1–A, K1–B, A–B) 최솟값 < 0.90이면 Q3 판정 불가(실패로 셈, 원값 기록). 실행 뒤 원값으로 여유를 점검
}


def is_lift_plan(plan):
    """들어올림 계획 판정(게이트와 훅이 같은 함수를 쓴다: 한쪽만 켜지는 fail-open 방지). 세트 33이거나 'lift' 절이 있으면 들어올림 계획이다."""
    return plan.get("set") == "33" or "lift" in plan


def params_for_plan(plan):
    """계획 JSON(세트 33)이 공급하는 값을 PARAMS 사본에 채운다: 접촉 자세마다 생성기가 계산한 두 번째 J2 최대 정수 목표(P10).
    plan[\"lift\"][\"j2_max_target\"]가 정수가 아니면 UNSET을 유지한다(fail-closed)."""
    import copy
    p = copy.deepcopy(PARAMS); v = (plan.get("lift") or {}).get("j2_max_target")
    if isinstance(v, int) and not isinstance(v, bool): p["lift_j2_max_target"] = v
    return p


def unset_names(params=None):
    p = PARAMS if params is None else params
    return sorted(k for k, v in p.items() if isinstance(v, _Unset))


def check_real_allowed(mode, params=None):
    """허용 목록 방식: mode가 'fake'·'dry_run'이 아니면(real·모르는 문자열 포함) UNSET이 하나라도 있을 때 ParamsNotFinal."""
    if mode not in ("fake", "dry_run"):
        miss = unset_names(params)
        if miss: raise ParamsNotFinal("미확정 상수가 있어 --real을 거부한다: %s" % ", ".join(miss))
