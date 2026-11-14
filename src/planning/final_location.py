"""③ 최종 블록 위치 후보별(반경) 관측→파지 이동 길이와 경로 여유. 블록 박스·마커 4모서리·캡 가림·입사각 반영. 하드웨어 명령 없음.
마커: 'top'(블록 윗면 25 mm) 또는 'front'(베이스 쪽 앞면 25 mm). 결과는 evidence/G06/final_location.json."""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from kinematics.urdf_fk import UrdfFK                                              # noqa: E402
from planning.collision_3a import Box, clearances, finger_capsules, load_link_hull  # noqa: E402
from planning.dry_run import BEST, TRUTH, TABLE_Z, HULL_KEYS, TILTS                 # noqa: E402
from planning.observe import PoseGrid, K                                            # noqa: E402
from planning.reach import solve_ik                                                 # noqa: E402

BLOCK = (0.030, 0.060, 0.030)   # x 폭 30 × y 길이 60 × 높이 30 mm(사용자 실제 노란 블록, 2026-11-15; 긴 변이 y 방향임을 Gemini 사진으로 확인)
MARKER = 0.025   # 사용자 보유 마커 AprilTag 36h11 id 7, 검은 정사각형 약 25 mm(Gemini 3차 촬영에서 약 24.8 mm로 읽음, 정확한 값은 사용자 실측 필요)
GRASP_D = 0.020                # 파지 깊이 d(캡 끝에서 블록 윗면 아래로 내려가는 거리)
GRASP_TIP_PAPER = BLOCK[2] - GRASP_D   # tip 높이 = 블록 높이 − d
MAX_INCIDENCE_DEG = 70.0       # 마커 입사각 상한(제안)


def marker_points(r, kind):
    h = MARKER / 2
    top = TABLE_Z + BLOCK[2]
    if kind == "top":
        c = np.array([0.0, r, top]); n = np.array([0.0, 0.0, 1.0])
        corners = [c + np.array([sx * h, sy * h, 0.0]) for sx in (-1, 1) for sy in (-1, 1)]
    else:
        c = np.array([0.0, r - BLOCK[1] / 2, TABLE_Z + 0.032]); n = np.array([0.0, -1.0, 0.0])
        corners = [c + np.array([sx * h, 0.0, sz * h]) for sx in (-1, 1) for sz in (-1, 1)]
    return c, n, corners


def occluded(fk, grid, idx, pts, caps):
    out = fk.fk(np.array([list(grid.sdk[idx]) + [90.0]]), gains=BEST.gains, tool_offset=BEST.tool_offset())
    p5 = out["arm5"][0][0] + np.array([0, 0, BEST.dz]); R5 = out["arm5"][1][0]
    cp, Rc = grid.cp[idx], grid.Rc[idx]

    def proj(P):
        p = Rc.T @ (P - cp)
        return K[0] * p[0] / p[2] + K[2], K[1] * p[1] / p[2] + K[3], p[2]
    discs = []
    for a, b, rr in caps:
        for t in np.linspace(0, 1, 15):
            u, v, dep = proj(p5 + R5 @ (a + (b - a) * t))
            if dep > 0:
                discs.append((u, v, K[0] * rr / dep))
    for P in pts:
        u, v, _ = proj(P)
        if any((u - cu) ** 2 + (v - cv) ** 2 < cr ** 2 for cu, cv, cr in discs):
            return True
    return False


def evaluate(r, kind, fk, grid, hulls, k_candidates=60):
    box = Box([0.0, r, TABLE_Z + BLOCK[2] / 2], [BLOCK[0] / 2, BLOCK[1] / 2, BLOCK[2] / 2])
    tip = np.array([0.0, r, TABLE_Z + GRASP_TIP_PAPER])
    feas = [tl for tl in TILTS if solve_ik(fk, tip, tl, BEST).get("ok")]
    rec = {"r_mm": r * 1000, "marker": kind, "feasible_grasp_tilts": feas}
    if not feas:
        rec["result"] = "grasp_unreachable"
        return rec
    tl = min(feas, key=abs)
    qg = solve_ik(fk, tip, tl, BEST)["q"]
    rec["grasp_tilt"] = tl
    rec["q_grasp"] = [round(v, 1) for v in qg]
    c, n, corners = marker_points(r, kind)
    ok = np.ones(len(grid.sdk), bool)
    for C in corners:
        o, _, _, _ = grid.visible_mask(C, 0.0, 40)
        ok &= o
    v = grid.cp - c                       # 마커 중심 -> 카메라 (마커 법선이 향하는 쪽에 카메라가 있어야 보인다)
    cosang = (v @ n) / np.linalg.norm(v, axis=1)
    ok &= cosang >= np.cos(np.radians(MAX_INCIDENCE_DEG))
    idx = np.flatnonzero(ok)
    rec["n_candidates"] = int(len(idx))
    if len(idx) == 0:
        rec["result"] = "no_observation_pose"
        return rec
    dist = np.linalg.norm(grid.tip[idx] - tip, axis=1)
    order = np.argsort(dist)
    caps = finger_capsules(0.09 + BEST.tip_y)
    tried = 0
    for kk in order:
        i = idx[kk]
        if tried >= k_candidates:
            break
        tried += 1
        if occluded(fk, grid, i, corners + [c], caps):
            continue
        q_obs = np.array([qg[0], *grid.sdk[i][1:]]); q_g = np.array(qg)
        n_steps = max(2, int(np.ceil(np.abs(q_g - q_obs).max() / 0.5)) + 1)
        worst = {name: {"box": 1e9, "table": 1e9, "base": 1e9} for name in TRUTH}
        for t in np.linspace(0, 1, n_steps):
            q = q_obs + (q_g - q_obs) * t
            for name, tr in TRUTH.items():
                f = fk.fk(np.array([list(q) + [90.0]]), gains=tr.gains, tool_offset=tr.tool_offset())
                cc = clearances(f, hulls, finger_capsules(0.09 + tr.tip_y), TABLE_Z, dz=tr.dz, boxes=[box])
                w = worst[name]; w["box"] = min(w["box"], cc["box"]); w["table"] = min(w["table"], cc["table"]); w["base"] = min(w["base"], cc.get("base", 1e9))
        pess = min(v2["box"] for v2 in worst.values()); best = worst["best"]["box"]
        rec_c = {"tip_distance_mm": round(float(dist[kk]) * 1000, 1), "tilt_obs": round(float(grid.tilt[i]), 1), "q_obs": [round(float(x), 1) for x in q_obs],
                 "path_min_box_clearance_best_mm": round(best * 1000, 1), "path_min_box_clearance_pess_mm": round(pess * 1000, 1),
                 "path_min_table_pess_mm": round(min(v2["table"] for v2 in worst.values()) * 1000, 1), "path_min_base_pess_mm": round(min(v2["base"] for v2 in worst.values()) * 1000, 1),
                 "candidates_tried": tried, "nearest_unfiltered_mm": round(float(dist[order[0]]) * 1000, 1)}
        if best >= 0.005 and pess >= 0.0:     # 제안 기준: 최선 ≥ 5 mm, 비관적 ≥ 0 (동결 아님)
            rec["result"] = "ok"; rec["chosen"] = rec_c
            return rec
        rec.setdefault("rejected_examples", []).append(rec_c) if len(rec.get("rejected_examples", [])) < 2 else None
    rec["result"] = "no_collision_free_path_in_first_%d" % k_candidates
    return rec


if __name__ == "__main__":
    fk = UrdfFK(); grid = PoseGrid(fk, BEST); hulls = {k: load_link_hull(v) for k, v in HULL_KEYS.items()}
    res = []
    for r in (0.13, 0.14, 0.15, 0.16, 0.17, 0.18):
        for kind in ("top", "front"):
            rec = evaluate(r, kind, fk, grid, hulls); res.append(rec)
            ch = rec.get("chosen")
            print("r=%3.0f %-5s 파지 tilt %s 후보 %5s -> %s %s" % (r * 1000, kind, rec.get("grasp_tilt"), rec.get("n_candidates"), rec["result"],
                  "" if not ch else "이동 %.1f mm, 관측 tilt %.1f, 경로 받침대 여유 최선 %.1f/비관적 %.1f mm, 종이(비관) %.1f, 베이스(비관) %.1f" % (ch["tip_distance_mm"], ch["tilt_obs"], ch["path_min_box_clearance_best_mm"], ch["path_min_box_clearance_pess_mm"], ch["path_min_table_pess_mm"], ch["path_min_base_pess_mm"])), flush=True)
    Path(ROOT / "evidence/G06").mkdir(exist_ok=True)
    Path(ROOT / "evidence/G06/final_location.json").write_text(json.dumps({"model": BEST.__dict__, "block_mm": [v * 1000 for v in BLOCK], "marker_mm": MARKER * 1000, "criteria": "경로 받침대 여유 최선>=5 mm, 비관적>=0 (제안, 동결 아님)", "results": res}, ensure_ascii=False, indent=1, default=float), encoding="utf-8")
