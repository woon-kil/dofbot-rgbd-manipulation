"""③ 근거리 M3 자세 선정(오프라인, 이동 명령 없음): 마커가 두 자세 모두에서 보이는 범위(단일 25 mm 마커, 깊이 ≥ 148 mm)에서 tip 이동 10·15·20 mm급 쌍을 공통 출발 S로 고른다.
배경: 큰 이동 M3(50~100 mm)는 가시성 필터가 주석에 묻힌 버그로 마커가 보이지 않는 자세를 골랐고 `--real`에서 6/6 미검출(docs/*큰이동M3-가시성필터-버그-이슈보고서.md).
**규칙은 후보를 계산하기 전에 고정했다(이 docstring이 사전 등록):**
 ① 관측 후보: 마커 4모서리(위치 ±6 mm 흔들림 9조합)가 영상 가장자리에서 **MARGIN px** 안쪽, 입사각 ≤ 70°, 캡 가림 없음, 깊이 ≥ 148 mm(결정 7 범위), J2 74~105·J3·J4 ≤ 105, 여유 게이트(종이 55·블록 20·베이스 25 mm).
 ② MARGIN = max(40 px, 2 × P1 실촬영에서 확인한 FK 투영 최대 오차 23.5 px) = 47 px. (P1 자세 A·B·B2의 FK 예측 대 실제 마커 모서리 오차 평균 16.1~21.8, 최대 23.5 px, FK가 실제보다 체계적으로 위쪽.)
 ③ 쌍: tip 이동 구간 **처음 등록: 10 mm(8~13)·15 mm(13~18)·20 mm(18~25) → 20 mm급 쌍 0개라 변경(결과를 본 뒤): 5 mm(3~8)·10 mm(8~13)·15 mm(13~18)**, 최대 관절 이동 ≥ 2°, 단일 관절 번호 순 이동 경로가 여유 게이트를 통과. 공통 출발 S에서 세 구간 도착이 모두 있어야 한다(쌍은 순서가 없어 양 끝을 모두 출발 후보로 본다).
 ④ 순위: 네 자세(S·E10·E15·E20)의 ‘과거 실행 상태와의 J2~J4 최대 편차’ 중 최악이 작은 S(동점은 관절 이동 합이 작은 쪽). 각 구간 도착은 편차가 가장 작은(동점은 관절 이동 작은) 것.
 ⑤ 독립 검증: 선정 자세마다 FK 카메라 자세로 마커 모서리 픽셀을 다시 투영해(visible_mask와 별개 코드 경로) 가장자리 여유를 표로 낸다.
출력: evidence/G06/p1_near_m3_pairs.json (후보·표·선정)."""
import itertools
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from kinematics.urdf_fk import UrdfFK                                   # noqa: E402
from planning.collision_3a import Box, load_link_hull                   # noqa: E402
from planning.dry_run import BEST, HULL_KEYS, TABLE_Z                   # noqa: E402
from planning.observe import PoseGrid, camera_pose, project            # noqa: E402
from planning import p1_poses as P                                      # noqa: E402
from planning import p1_b2_poses as B                                   # noqa: E402
from planning.p1_plan_check import eval_moves                           # noqa: E402
from planning.p1_bigm3_select import executed_states, novelty           # noqa: E402

MARGIN = 47.0
DEPTH_MIN = 0.148
GATES = {"table": 55.0, "box": 20.0, "base": 25.0}
J2_RANGE = (74.0, 105.0)
BANDS = [("5", 3.0, 8.0), ("10", 8.0, 13.0), ("15", 13.0, 18.0)]   # 2026-11-15 변경: 처음 규칙은 10·15·20 mm급이었으나 여유 47 px에서 20 mm급 쌍이 0개라 5·10·15 mm급으로 바꿨다(결과를 본 뒤의 변경, 이력은 docs/gates/G07-근거리M3-승인-요청-문서.md §0-4)
MIN_JOINT_DELTA = 2.0
IMG_W, IMG_H = 640, 480


def corner_margins(fk, q, M, h, jitter):
    """독립 경로: FK 카메라 자세로 마커 4모서리(위치 흔들림 포함)를 투영해 영상 가장자리까지의 최소 여유(px)와 모서리 픽셀을 구한다."""
    cp, Rc = camera_pose(fk, q, BEST); worst = 1e9; center_px = None
    for dx, dy in jitter:
        for sx, sy in ((-1, 1), (1, 1), (1, -1), (-1, -1)):
            u, v, z = project(M + np.array([dx + sx * h, dy + sy * h, 0.0]), cp, Rc)
            if z <= 0:
                return -1e9, None
            worst = min(worst, u, v, IMG_W - u, IMG_H - v)
    u, v, z = project(M, cp, Rc)
    return float(worst), (float(u), float(v), float(z))


def main():
    fk = UrdfFK(); grid = PoseGrid(fk, BEST, j234_min=P.J234_MIN); hulls = {k: load_link_hull(v) for k, v in HULL_KEYS.items()}
    half = np.array([P.BLOCK[0] / 2 + 0.003, P.BLOCK[1] / 2 + 0.003, P.BLOCK[2] / 2]); box = Box([B.CENTER[0], B.CENTER[1], TABLE_Z + P.BLOCK[2] / 2], half)
    top = TABLE_Z + P.BLOCK[2]; h = P.MARKER / 2; M = np.array([B.CENTER[0], B.CENTER[1] + B.MARKER_OFFSET_Y, top])
    ok = np.ones(len(grid.sdk), bool); n0 = int(ok.sum())
    for dx, dy in B.SHIFTS:
        for sx in (-1, 1):
            for sy in (-1, 1):
                o, _, _, _ = grid.visible_mask(M + np.array([dx + sx * h, dy + sy * h, 0.0]), 0.0, MARGIN, max_tilt=90.0)
                ok &= o
    print("가시성 필터(여유 %.0f px): 격자 %d → %d 자세" % (MARGIN, n0, int(ok.sum())), flush=True); assert int(ok.sum()) < n0
    v = grid.cp - M; ok &= (v[:, 2] / np.linalg.norm(v, axis=1)) >= np.cos(np.radians(70.0))
    idx = np.flatnonzero(ok); depth = np.einsum("nji,nj->ni", grid.Rc[idx], M - grid.cp[idx])[:, 2]; idx = idx[depth >= DEPTH_MIN]
    pts = [M] + [M + np.array([sx * h, sy * h, 0.0]) for sx in (-1, 1) for sy in (-1, 1)]
    S = []
    for i in idx:
        if B.occluded(fk, grid, i, pts):
            continue
        q = [90.0, *[float(x) for x in grid.sdk[i][1:]]]
        if not (J2_RANGE[0] <= q[1] <= J2_RANGE[1] and q[2] <= 105 and q[3] <= 105):
            continue
        w = B.worst(fk, hulls, q, box); wm = {k: x * 1000 for k, x in w.items()}
        if all(wm[k] >= GATES[k] for k in GATES):
            S.append((i, q, wm))
    print("관측 후보(깊이·가림·J 범위·게이트 통과) %d" % len(S), flush=True)
    St = executed_states()
    pairs = []
    for (i, qi, _), (j, qj, _) in itertools.combinations(S, 2):
        d = np.abs(np.array(qi[1:]) - np.array(qj[1:])); mv = float(np.linalg.norm(grid.tip[i] - grid.tip[j])) * 1000
        if d.max() >= MIN_JOINT_DELTA and BANDS[0][1] <= mv < BANDS[-1][2]:
            pairs.append((i, j, qi, qj, mv, float(d.max())))
    print("쌍 후보(3~18 mm) %d: 구간별 %s" % (len(pairs), {n: sum(1 for p in pairs if lo <= p[4] < hi) for n, lo, hi in BANDS}), flush=True)
    part = {}
    for pr in pairs:
        i, j, qi, qj, mv, dm = pr
        for s_, e_, qs, qe in ((i, j, qi, qj), (j, i, qj, qi)):
            for name, lo, hi in BANDS:
                if lo <= mv < hi:
                    part.setdefault((s_, tuple(qs)), {}).setdefault(name, []).append((s_, e_, qs, qe, mv, dm))
    starts = [k for k, vv in part.items() if len(vv) == len(BANDS)]
    print("세 구간 모두에 도착 후보가 있는 출발 자세 %d" % len(starts), flush=True)
    jitter = B.SHIFTS; cands = []
    for k in starts:
        ends = {}
        for name, _, _ in BANDS:
            best = None
            for (s_, e_, qs, qe, mv, dm) in sorted(part[k][name], key=lambda p: (novelty(St, p[3]), p[5])):
                cur = list(qs); moves = [(a + 1, qe[a]) for a in range(4) if qe[a] != qs[a]]
                wpath = eval_moves(fk, hulls, box, cur, moves)
                if all(wpath[g] >= GATES[g] for g in GATES):
                    pe = np.einsum("ji,j->i", grid.Rc[e_], M - grid.cp[e_]); ps = np.einsum("ji,j->i", grid.Rc[k[0]], M - grid.cp[k[0]])
                    best = {"to": qe, "tip_move_mm": round(mv, 1), "max_joint_delta_deg": dm, "joint_deltas": [round(qe[a] - qs[a], 1) for a in range(1, 4)], "path_worst_mm": wpath,
                            "depth_S_to_E_mm": [round(float(ps[2]) * 1000, 1), round(float(pe[2]) * 1000, 1)], "predicted_marker_shift_cam_mm": np.round((pe - ps) * 1000, 1).tolist()}
                    break
            if best is None:
                ends = None; break
            ends[name] = best
        if ends is None:
            continue
        worst = max([novelty(St, list(k[1]))] + [novelty(St, e["to"]) for e in ends.values()])
        cands.append((worst, sum(e["max_joint_delta_deg"] for e in ends.values()), list(k[1]), ends))
    cands.sort(key=lambda t: (t[0], t[1]))
    print("경로 검사 통과 공통 출발 후보 %d" % len(cands), flush=True)
    if not cands:
        sys.exit("선정 가능한 공통 출발이 없다")
    worst, sj, S0, ends = cands[0]
    table = []
    for name, q in [("S", S0)] + [("E" + n, e["to"]) for n, e in ends.items()]:
        m, cen = corner_margins(fk, q, M, h, jitter)
        table.append({"name": name, "q": q, "corner_edge_margin_px_with_jitter": round(m, 1), "marker_center_px_depth_mm": [round(cen[0], 1), round(cen[1], 1), round(cen[2] * 1000, 1)], "novelty_deg": round(novelty(St, q), 1)})
        assert m >= MARGIN - 1e-6, ("독립 투영 가시성이 선정 여유보다 작다", name, m)
    out = {"margin_px": MARGIN, "depth_min_m": DEPTH_MIN, "n_candidates": len(S), "n_pairs": len(pairs), "n_common_starts_checked": len(cands), "worst_novelty_deg": worst, "selected": {"S": S0, "ends": ends},
           "projection_table": table, "ranking_top3": [{"S": c[2], "worst_novelty_deg": c[0], "joint_delta_sum": c[1]} for c in cands[:3]]}
    (ROOT / "evidence/G06/p1_near_m3_pairs.json").write_text(json.dumps(out, ensure_ascii=False, indent=1, default=float), encoding="utf-8")
    print("선정 S", S0, "최악 처음조합 편차 %.1f°" % worst)
    for n, e in ends.items():
        print("  E%s %s tip %.1f mm 관절 %s 깊이 %s 예측 변위 %s 경로 %s" % (n, e["to"], e["tip_move_mm"], e["joint_deltas"], e["depth_S_to_E_mm"], e["predicted_marker_shift_cam_mm"], e["path_worst_mm"]))
    print("독립 투영 표(가장자리 여유 px, 마커 중심 u v 깊이):")
    for t in table:
        print("  %-4s %s 여유 %.1f px, 중심 %s, 편차 %.1f°" % (t["name"], t["q"], t["corner_edge_margin_px_with_jitter"], t["marker_center_px_depth_mm"], t["novelty_deg"]))


if __name__ == "__main__":
    main()
