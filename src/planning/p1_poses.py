"""③ P1 자세 목록 후보(오프라인 계산, 이동 명령 없음). 받침대 30×60×50 mm(윗면 마커 25 mm)를 r=170 mm, 방위각 0°에 둔 경우.
단계 a(깊이 0.18→0.15 m, 여유 게이트 ≥ GATE_A), 단계 b(0.15→0.13 m, ≥ GATE_B), 그리고 M3용 작은 이동 쌍.
게이트 값은 제안이며 동결이 아니다. 자세는 관절 공간 선형 보간 경로로 시작 자세에서 오가며 모든 모델 변형에서 여유를 검사한다."""
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from kinematics.urdf_fk import UrdfFK                                          # noqa: E402
from planning.collision_3a import Box, clearances, finger_capsules, load_link_hull   # noqa: E402
from planning.dry_run import BEST, TRUTH, TABLE_Z, HULL_KEYS                    # noqa: E402
from planning.observe import PoseGrid, K                                        # noqa: E402

R_P1 = 0.170
BLOCK = (0.030, 0.060, 0.030)   # x 폭 30 × y 길이 60 × 높이 30 mm(사용자 실제 노란 블록, 2026-11-15; 긴 변이 y 방향임을 Gemini 사진으로 확인)
MARKER = 0.025   # 사용자 보유 마커 AprilTag 36h11 id 7, 검은 정사각형 약 25 mm(Gemini 3차 촬영에서 약 24.8 mm로 읽음, 정확한 값은 사용자 실측 필요)
GATE_A, GATE_B = 0.030, 0.020
HOME = [85.0, 94.0, 98.0, 91.0]      # 직전 시험들의 시작·복귀 자세(S4-13 state0 J1~J4)
MAX_STEP_DEG = 0.5
# 지금까지 실제로 이동한 적이 있는 가장 낮은 관절 범위(42개 보정 자세: J2 74, J3 10, J4 11, `evidence/G06/offset_discrimination.json`). 그 아래(J3<10, J4<11)는 승인된 적이 없어 P1 후보에서 제외한다.
J234_MIN = (0.0, 10.0, 11.0)


def worst_clearance(fk, hulls, q, box):
    w = {"box": 1e9, "table": 1e9, "base": 1e9}
    for t in TRUTH.values():
        f = fk.fk(np.array([list(q) + [90.0]]), gains=t.gains, tool_offset=t.tool_offset())
        c = clearances(f, hulls, finger_capsules(0.09 + t.tip_y), TABLE_Z, dz=t.dz, boxes=[box])
        w["box"] = min(w["box"], c["box"]); w["table"] = min(w["table"], c["table"]); w["base"] = min(w["base"], c.get("base", 1e9))
    return w


def path_worst(fk, hulls, q0, q1, box):
    q0, q1 = np.array(q0, float), np.array(q1, float)
    n = max(2, int(np.ceil(np.abs(q1 - q0).max() / MAX_STEP_DEG)) + 1)
    w = {"box": 1e9, "table": 1e9, "base": 1e9}
    for t in np.linspace(0, 1, n):
        c = worst_clearance(fk, hulls, q0 + (q1 - q0) * t, box)
        for k in w:
            w[k] = min(w[k], c[k])
    return {k: round(v * 1000, 1) for k, v in w.items()}, float(np.abs(q1 - q0).max())


def describe(fk, grid, i, M, corners):
    p = np.einsum("ji,j->i", grid.Rc[i], M - grid.cp[i])
    px = []
    for C in corners:
        q = grid.Rc[i].T @ (C - grid.cp[i])
        px.append([round(K[0] * q[0] / q[2] + K[2], 1), round(K[1] * q[1] / q[2] + K[3], 1)])
    return {"sdk": [float(v) for v in grid.sdk[i]], "depth_mm": round(float(p[2]) * 1000, 1), "marker_corner_px": px,
            "tip_xyz_paper_mm": [round(float(grid.tip[i][0]) * 1000, 1), round(float(grid.tip[i][1]) * 1000, 1), round(float(grid.tip[i][2] - TABLE_Z) * 1000, 1)],
            "tilt_deg": round(float(grid.tilt[i]), 1)}


def build(r_p1=None):
    global R_P1
    if r_p1 is not None:
        R_P1 = r_p1
    fk = UrdfFK(); grid = PoseGrid(fk, BEST, j234_min=J234_MIN); hulls = {k: load_link_hull(v) for k, v in HULL_KEYS.items()}
    box = Box([0.0, R_P1, TABLE_Z + BLOCK[2] / 2], [BLOCK[0] / 2, BLOCK[1] / 2, BLOCK[2] / 2])
    top = TABLE_Z + BLOCK[2]
    M = np.array([0.0, R_P1, top]); h = MARKER / 2
    corners = [M + np.array([sx * h, sy * h, 0.0]) for sx in (-1, 1) for sy in (-1, 1)]
    ok = np.ones(len(grid.sdk), bool)
    for C in corners:
        o, _, _, _ = grid.visible_mask(C, 0.0, 40); ok &= o
    v = grid.cp - M
    ok &= (v[:, 2] / np.linalg.norm(v, axis=1)) >= np.cos(np.radians(70.0))
    idx = np.flatnonzero(ok)
    depth = np.einsum("nji,nj->ni", grid.Rc[idx], M - grid.cp[idx])[:, 2]
    cand = {}
    for name, lo, hi in (("a", 0.15, 0.18), ("b", 0.13, 0.15)):
        ii = idx[(depth >= lo) & (depth < hi)]
        rows = []
        for i in ii:
            q = [90.0, *[float(x) for x in grid.sdk[i][1:]]]
            w = worst_clearance(fk, hulls, q, box)
            rows.append((i, min(w["box"], w["table"]), w))
        cand[name] = rows
    ga = [r for r in cand["a"] if r[1] >= GATE_A]; gb = [r for r in cand["b"] if r[1] >= GATE_B]
    out = {"joint_min_j2_j3_j4": J234_MIN, "p1_position_mm": {"r": R_P1 * 1000, "az_deg": 0}, "gates_proposed_mm": {"a": GATE_A * 1000, "b": GATE_B * 1000}, "n_candidates": {"a": len(cand["a"]), "a_pass": len(ga), "b": len(cand["b"]), "b_pass": len(gb)}}
    if not ga or not gb:
        out["result"] = "no_pose_passes_gate"; return out
    ia = max(ga, key=lambda r: r[1])[0]                          # 단계 a: 여유 최대
    qa = np.array(grid.sdk[ia]); ib = min(gb, key=lambda r: np.linalg.norm(grid.sdk[r[0]] - qa))[0]   # 단계 b: a와 관절 거리 최소
    # M3 쌍: b와 같은 구간에서 tip 거리 8~15 mm, 게이트 통과, 관절 거리 최소
    pairs = [r for r in gb if r[0] != ib and 0.008 <= np.linalg.norm(grid.tip[r[0]] - grid.tip[ib]) <= 0.015]
    ib2 = min(pairs, key=lambda r: np.linalg.norm(grid.sdk[r[0]] - grid.sdk[ib]))[0] if pairs else None
    poses = {"HOME": {"sdk": HOME}, "A": describe(fk, grid, ia, M, corners), "B": describe(fk, grid, ib, M, corners)}
    if ib2 is not None:
        poses["B2"] = describe(fk, grid, ib2, M, corners)
        poses["B2"]["tip_distance_from_B_mm"] = round(float(np.linalg.norm(grid.tip[ib2] - grid.tip[ib])) * 1000, 1)
    for k in ("A", "B", "B2"):
        if k in poses:
            q = [90.0, *poses[k]["sdk"][1:]]
            poses[k]["sdk"] = q
            w = worst_clearance(fk, hulls, q, box)
            poses[k]["pose_worst_clearance_mm"] = {a: round(b * 1000, 1) for a, b in w.items()}
    order = [("HOME", "A"), ("A", "B")] + ([("B", "B2"), ("B2", "B")] if ib2 is not None else []) + [("B", "A"), ("A", "HOME")]
    segs = []
    for a, b in order:
        w, mx = path_worst(fk, hulls, poses[a]["sdk"], poses[b]["sdk"], box)
        segs.append({"from": a, "to": b, "max_joint_delta_deg": round(mx, 1), "path_worst_clearance_mm": w})
    out.update({"result": "ok", "poses": poses, "segments": segs})
    canon = json.dumps({"poses": {k: v["sdk"] for k, v in poses.items()}, "segments": [(s["from"], s["to"]) for s in segs]}, sort_keys=True).encode()
    out["pose_list_sha256"] = hashlib.sha256(canon).hexdigest()
    return out


if __name__ == "__main__":
    res = build(float(sys.argv[1]) / 1000 if len(sys.argv) > 1 else None)
    Path(ROOT / "evidence/G06").mkdir(exist_ok=True)
    Path(ROOT / ("evidence/G06/p1_pose_list_r%d.json" % round(R_P1 * 1000))).write_text(json.dumps(res, ensure_ascii=False, indent=1, default=float), encoding="utf-8")
    print(res["result"], res["n_candidates"])
    for k, v in res.get("poses", {}).items():
        print(k, v["sdk"], {a: v[a] for a in ("depth_mm", "tip_xyz_paper_mm", "tilt_deg", "pose_worst_clearance_mm", "tip_distance_from_B_mm") if a in v})
    for s in res.get("segments", []):
        print(s)
    print("sha256", res.get("pose_list_sha256"))
