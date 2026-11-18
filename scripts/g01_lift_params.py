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
    __add__ = __radd__ = __sub__ = __mul__ = __rmul__ = __truediv__ = __getitem__ = __iter__ = __len__ = __int__ = __float__ = __index__ = __contains__ = _no
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
    # --- S4' 관문 승인 값(E-2b `:14` 승인됨, S4' 문서 §3 `:22~28`) ---
    "g2_deadzone_px": 0.75, "g2_s_range_mm": (-1.26, -0.77), "g2_magnitude_mm": 3.0, "g4_side_px": 0.75, "g5_yaw_deg": 3.0,
    # --- 미확정(기본값 없음; 정렬 보고 계산 `p2k_align_report.py`·`p2k_analyze_261117.py` 정독 전) ---
    "g2_mm_per_px": UNSET,                  # G2: 중심 이동(px) → Δx_block(mm) 환산(S4' 문서는 '0.75 px ≈ 0.6 mm'로만 적음)
    "g2_x_sign": UNSET,                     # G2: 영상 x 이동 부호 → 프로젝트 x 부호 규약(프로젝트 x와 월드 x가 반대, 구현 전 원 기록으로 확정)
    # --- 미확정(P9 확정 전, 기본값 없음) ---
    "q3_box": UNSET,                        # P9: (나') (800,0,900,100) 원칙 동의, 확정은 사전 등록 전
    "q3_arm_in_box_criterion": UNSET,       # P9 조건 ③: 들어올림 자세에서 팔이 영역에 들어오면 판정 불가로 처리하는 기준
}


def unset_names(params=None):
    p = PARAMS if params is None else params
    return sorted(k for k, v in p.items() if isinstance(v, _Unset))


def check_real_allowed(mode, params=None):
    """허용 목록 방식: mode가 'fake'·'dry_run'이 아니면(real·모르는 문자열 포함) UNSET이 하나라도 있을 때 ParamsNotFinal."""
    if mode not in ("fake", "dry_run"):
        miss = unset_names(params)
        if miss: raise ParamsNotFinal("미확정 상수가 있어 --real을 거부한다: %s" % ", ".join(miss))
