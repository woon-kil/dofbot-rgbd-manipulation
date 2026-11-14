"""③ P1 오프라인 계산: 받침대(30×60×50 mm, 윗면 마커 25 mm)를 두었을 때 카메라-마커 깊이별로 닿을 수 있는 관측 자세와
손가락-받침대 여유(최선·비관적 모델). 하드웨어 명령 없음. 결과는 evidence/G06/p1_ladder.json."""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from kinematics.urdf_fk import UrdfFK                                           # noqa: E402
from planning.collision_3a import Box, clearances, finger_capsules, capsule_points, load_link_hull   # noqa: E402
from planning.dry_run import BEST, TRUTH, TABLE_Z, HULL_KEYS                    # noqa: E402
from planning.observe import PoseGrid                                           # noqa: E402

BLOCK = (0.030, 0.060, 0.030)   # x 폭 30 × y 길이 60 × 높이 30 mm(사용자 실제 노란 블록, 2026-11-15; 긴 변이 y 방향임을 Gemini 사진으로 확인)
MARKER = 0.025   # 사용자 보유 마커 AprilTag 36h11 id 7, 검은 정사각형 약 25 mm(Gemini 3차 촬영에서 약 24.8 mm로 읽음, 정확한 값은 사용자 실측 필요)
BINS = ((0.09, 0.11), (0.11, 0.13), (0.13, 0.15), (0.15, 0.18), (0.18, 0.22), (0.22, 0.26))


def run(r, fk=None, grid=None, hulls=None):
    fk = fk or UrdfFK()
    grid = grid or PoseGrid(fk, BEST)
    hulls = hulls or {k: load_link_hull(v) for k, v in HULL_KEYS.items()}
    box = Box([0.0, r, TABLE_Z + BLOCK[2] / 2], [BLOCK[0] / 2, BLOCK[1] / 2, BLOCK[2] / 2])
    top = TABLE_Z + BLOCK[2]
    M = np.array([0.0, r, top])
    h = MARKER / 2
    corners = [M + np.array([sx * h, sy * h, 0.0]) for sx in (-1, 1) for sy in (-1, 1)]
    ok = np.ones(len(grid.sdk), bool)
    for C in corners:
        o, _, _, _ = grid.visible_mask(C, 0.0, 40)
        ok &= o
    v = grid.cp - M                       # 마커 중심 -> 카메라, 윗면 법선(+z)과의 입사각 ≤ 70°(제안)
    ok &= (v[:, 2] / np.linalg.norm(v, axis=1)) >= np.cos(np.radians(70.0))
    idx = np.flatnonzero(ok)
    p = np.einsum("nji,nj->ni", grid.Rc[idx], M - grid.cp[idx])
    depth = p[:, 2]
    caps = finger_capsules(0.09 + BEST.tip_y)
    local = np.array([a + (b - a) * t for a, b, _ in caps for t in np.linspace(0, 1, 5)])
    rad = np.array([rr for _, _, rr in caps for _ in range(5)])
    out = {"r_mm": r * 1000, "block_mm": [v * 1000 for v in BLOCK], "marker_mm": MARKER * 1000, "n_visible_poses": int(len(idx)), "bins": []}
    for lo, hi in BINS:
        m = (depth >= lo) & (depth < hi)
        rec = {"depth_mm": [lo * 1000, hi * 1000], "n": int(m.sum())}
        if m.any():
            ii = idx[m]
            pts = grid.p5[ii][:, None, :] + np.einsum("nij,kj->nki", grid.R5[ii], local)
            clr = (box.signed_distance(pts) - rad).min(axis=1)             # 손가락 캡슐-받침대 여유(최선 모델)
            tipz = (grid.tip[ii][:, 2] - top)                               # tip 높이 - 받침대 윗면
            order = np.argsort(-clr)
            rec["best_finger_box_clearance_mm"] = round(float(clr[order[0]]) * 1000, 1)
            rec["best_pose"] = {"sdk": [round(float(v), 1) for v in grid.sdk[ii[order[0]]]], "tip_minus_top_mm": round(float(tipz[order[0]]) * 1000, 1), "tilt_deg": round(float(grid.tilt[ii[order[0]]]), 1)}
            for X in (0.030, 0.020, 0.010, 0.0):
                rec["n_clearance_ge_%d_mm" % (X * 1000)] = int((clr >= X).sum())
            # 최선 자세를 모델 변형마다 정확히 평가
            q = [90.0, *[float(v) for v in grid.sdk[ii[order[0]]][1:]]]
            ev = {}
            for name, t in TRUTH.items():
                f = fk.fk(np.array([q + [90.0]]), gains=t.gains, tool_offset=t.tool_offset())
                c = clearances(f, hulls, finger_capsules(0.09 + t.tip_y), TABLE_Z, dz=t.dz, boxes=[box])
                ev[name] = {"table": round(c["table"] * 1000, 1), "box": round(c["box"] * 1000, 1), "base": round(c.get("base", float("nan")) * 1000, 1)}
            rec["best_pose_variants"] = ev
            # 후보 전부를 변형마다 정확히 평가(상한 MAXN개, 여유 큰 순). 변형 최소 여유가 X 이상인 자세 수
            MAXN = 400
            cand = order[:MAXN]
            worst = []
            for k in cand:
                q2 = [90.0, *[float(v) for v in grid.sdk[ii[k]][1:]]]
                w = 1e9
                for name, t in TRUTH.items():
                    f2 = fk.fk(np.array([q2 + [90.0]]), gains=t.gains, tool_offset=t.tool_offset())
                    c2 = clearances(f2, hulls, finger_capsules(0.09 + t.tip_y), TABLE_Z, dz=t.dz, boxes=[box])
                    w = min(w, c2["box"], c2["table"])
                worst.append(w)
            worst = np.array(worst)
            rec["n_evaluated_exact"] = int(len(cand))
            for X in (0.030, 0.020, 0.010, 0.0):
                rec["n_all_variants_ge_%d_mm" % (X * 1000)] = int((worst >= X).sum())
            # 구간 안 최선 자세의 진단: tip 위치(받침대 중심 기준), 관측 마스크 경계, 보정 자세와 거리
            jb = ii[order[0]]
            tipw = grid.tip[jb]
            rec["best_pose"]["tip_rel_box_center_mm"] = [round(float(v) * 1000, 1) for v in (tipw - box.center)]
            rec["best_pose"]["max_joint_deg"] = round(float(grid.sdk[jb][1:].max()), 1)
        out["bins"].append(rec)
    return out


if __name__ == "__main__":
    fk = UrdfFK(); grid = PoseGrid(fk, BEST); hulls = {k: load_link_hull(v) for k, v in HULL_KEYS.items()}
    res = {"model": BEST.__dict__, "results": [run(r, fk, grid, hulls) for r in (0.13, 0.14, 0.15, 0.16, 0.17, 0.18, 0.19)]}
    Path(ROOT / "evidence/G06").mkdir(exist_ok=True)
    Path(ROOT / "evidence/G06/p1_ladder.json").write_text(json.dumps(res, ensure_ascii=False, indent=1, default=float), encoding="utf-8")
    for rr in res["results"]:
        print("r=%.0f mm, 마커 4모서리 보이는 자세 %d" % (rr["r_mm"], rr["n_visible_poses"]))
        for b in rr["bins"]:
            if b["n"] == 0:
                print("  깊이 %3d~%3d mm: 자세 없음" % tuple(b["depth_mm"])); continue
            ev = b["best_pose_variants"]; worst_box = min(v["box"] for v in ev.values()); worst_tab = min(v["table"] for v in ev.values())
            print("  깊이 %3d~%3d mm: 자세 %6d, 손가락-받침대 최선 여유 %6.1f mm (≥30: %d, ≥20: %d, ≥10: %d), 그 자세 tip−윗면 %6.1f mm, 비관적 받침대 %6.1f·종이 %6.1f mm" % (b["depth_mm"][0], b["depth_mm"][1], b["n"], b["best_finger_box_clearance_mm"], b["n_clearance_ge_30_mm"], b["n_clearance_ge_20_mm"], b["n_clearance_ge_10_mm"], b["best_pose"]["tip_minus_top_mm"], worst_box, worst_tab))
