#!/usr/bin/env python3
"""세트 33(들어올림) 계획 생성(Mac, 오프라인): 검증된 생성기 `g01_s42_plan_p2k.py --set 31`이 만든 계획을 읽어 세트 33 계획으로 바꾼다(생성기·세트 31은 수정하지 않는다).
추가하는 것(플랜 docs/261118-1038-들어올림-러너-구현-플랜.md §2 R1·R12, P10):
 ① plan["lift"]: 첫 J2 명령, **두 번째 J2의 최대 정수 목표 j2_max_target = FK 최악(증가 방향 부족 0, J4 감소 넘침 0.8)에서 정점 tilt ≤ 30°인 최대 정수 목표**(P10, ADR-007 개정 ≤ 30°),
 ② return_routes 끝에 '들어올린 상태'의 복귀 항목(두 번째 J2 후보 × J4 전/후)과 색인표(F6): 러너가 PROG["sent"]를 이 색인으로 설정한다.
모델 값 = UNVERIFIED FK. 사용: .venv/bin/python -I scripts/g01_s42_plan_lift.py <세트 31 계획.json> <출력.json>   (저장소 루트에서 실행)"""
import copy, importlib, json, sys
from pathlib import Path

TILT_LIMIT_DEG = 30.0        # ADR-007 개정 2026-11-16(파지 접근 기울기 ≤ 30°). 들어올린 자세에 적용한 것은 E-2b §9-1의 확장(플랜 P10 조건 ②)
DEC_OVERSHOOT_DEG = 0.8      # 감소 방향 넘침 최대(메모리 project_small_up_step_servo_noop: 약 0~0.8°)
J2_FIRST_UP = 5              # E-6'' 첫 J2 명령 +5
J4_DELTA = -5                # E-6'' J4 −5
SHORTFALL_BELOW = 2          # 복귀 항목의 두 번째 J2 후보 하한 = 첫 명령 − 2(서보 부족 1.1~2.0°, do_return의 상태 허용 3°)


def _load_modules():
    root = Path(__file__).resolve().parents[1]
    for p in (root / "scripts", root / "src", root / "evidence" / "G06"): sys.path.insert(0, str(p))
    import os; os.chdir(root)                                  # 생성기 모듈이 상대 경로로 파일을 연다
    G = importlib.import_module("g01_s42_plan_p2k"); C = importlib.import_module("carried_block_path_261117"); return G, C


def worst_case_peak_tilt(C, q0, first_target, target2):
    """부족 0·J4 넘침 0.8 최악에서 [첫 J2, 두 번째 J2, J4 −5] 경로의 정점 tilt와 통과 여부(블록 바닥 ≥ 게이트)."""
    dq1 = first_target - q0[1]; dq2 = target2 - first_target                       # 부족 0: 두 이동 모두 명령대로 도달
    st, ok = C.follow([(2, dq1), (2, dq2), (4, J4_DELTA)], q0=list(q0), inc_shortfall_deg=0.0, dec_overshoot_deg=DEC_OVERSHOOT_DEG)
    return max(x["tilt"] for x in st), bool(ok)


def max_target(C, q0, first_target, limit=TILT_LIMIT_DEG, search=12):
    """정점 tilt ≤ limit이고 경로가 통과하는 최대 정수 두 번째 J2 목표(없으면 None)."""
    best = None
    for t in range(first_target, first_target + search + 1):
        peak, ok = worst_case_peak_tilt(C, q0, first_target, t)
        if peak <= limit + 1e-9 and ok: best = (t, peak)
        elif best is not None: break
    return best


def make_lift_plan(plan31, G=None, C=None):
    if G is None or C is None: G, C = _load_modules()
    plan = copy.deepcopy(plan31)
    if plan.get("set") != "31": raise ValueError("세트 31 계획이 아니다")
    run2 = plan["run2"]; pid = run2["contact_pose"]
    k = max(i for i, m in enumerate(plan["moves"], 1) if m.get("pose_id") == pid)          # 접촉 자세에 도착한 마지막 이동 번호(= 완료한 이동 수)
    st = plan["return_routes"][k]["state"]                                                # 접촉 자세 명령 상태 J1~J4
    use_q0 = [int(v) for v in st] == [86, 69, 12, 32]
    q0 = list(C.Q0) if use_q0 else [float(v) for v in st]                                 # Run 4 접촉 원시 자세와 같으면 그 값, 아니면 명령값 대용(정확도 낮음 → 표시)
    first_target = int(st[1]) + J2_FIRST_UP
    mt = max_target(C, q0, first_target)
    if mt is None: raise ValueError("tilt ≤ %.0f° 안의 두 번째 J2 목표가 없다" % TILT_LIMIT_DEG)
    t_max, peak = mt
    ret_target = plan["return_target"]; floor = min(G.BASE_MIN, G.POSE_MIN["31"]); rets = plan["return_routes"]; index = {}; errs = []
    for j2 in range(first_target - SHORTFALL_BELOW, t_max + 1):
        for stage, j4 in (("before_j4", st[3]), ("after_j4", st[3] + J4_DELTA)):
            state = [st[0], j2, st[2], j4]; lo, route = G.return_route(state, ret_target); full = G.route_min(state, route)
            if full < floor - G.EPS: errs.append("들어올린 상태 %s 복귀 경로 최저 %.1f < %.1f" % (state, full, floor))
            idx = len(rets); rets.append({"after_move": idx, "state": state, "route": [{"joint": j, "target": t, "time_ms": tm} for j, t, tm in route], "lift_stage": stage}); index["%s:%d" % (stage, j2)] = idx
    plan["p2k"] = dict(plan["p2k"], hold_s=3.0)   # 세트 33: Gemini 36 s 사진(55 s 정지)을 쓰지 않는다(사진은 P-A 스냅, 기준 사진은 실행 전). 정지는 처짐 감시용 짧은 대기뿐(누름 없음)
    plan["set"] = "33"; plan["lift"] = {"j2_first_target": first_target, "j2_max_target": int(t_max), "j4_delta": J4_DELTA, "contact_moves_done": k, "q0": q0, "q0_source": "run4_raw" if use_q0 else "command_proxy",
                                         "tilt_limit_deg": TILT_LIMIT_DEG, "tilt_peak_at_max_deg": round(peak, 2), "model": "FK UNVERIFIED, 증가 방향 부족 0·J4 넘침 %.1f 최악" % DEC_OVERSHOOT_DEG, "return_index": index}
    plan["gate_errors"] = list(plan.get("gate_errors", [])) + errs
    return plan, errs


if __name__ == "__main__":
    if len(sys.argv) != 3: sys.exit(__doc__)
    src, out = sys.argv[1], sys.argv[2]
    plan, errs = make_lift_plan(json.load(open(src, encoding="utf-8")))
    json.dump(plan, open(out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    L = plan["lift"]; print("세트 33: 첫 J2 명령 %d, 두 번째 J2 최대 정수 목표 %d(정점 tilt %.2f° ≤ %.0f°, q0 %s), 복귀 항목 %d개 추가" % (L["j2_first_target"], L["j2_max_target"], L["tilt_peak_at_max_deg"], L["tilt_limit_deg"], L["q0_source"], len(L["return_index"])))
    if errs: print("게이트 실패:", errs); sys.exit(1)
    print("게이트 통과")
