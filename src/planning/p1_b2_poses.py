"""③ P1-b2(깊이 0.11~0.13 m) 자세 후보 계산(오프라인, 이동 명령 없음).
P1 결과 반영: 실제 블록 위치 (1.4, 189) mm, 마커 25 mm, 오므린 그리퍼 캡 간격은 사진 대조값(캡 중심 ±7.1 mm, 쌍 중심 −3 mm, UNVERIFIED)과 인접 가정(4.9·12 mm, 옵셋 0/−3 mm)의 최악값,
마커 가시성은 PnP−FK 차이(카메라 프레임 약 5~7 mm)를 월드 x·y ±6 mm 흔들림으로 넣어 9개 조합 전부에서 4모서리가 가장자리 40 px 안쪽이어야 한다.
M3 쌍: 같은 깊이 구간에서 한 관절 ≥ 5° 이동(회독 1° 분해능이 이동의 20 % 이하)."""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from kinematics.urdf_fk import UrdfFK                                   # noqa: E402
from planning.collision_3a import Box, clearances, finger_capsules, load_link_hull   # noqa: E402
from planning.dry_run import BEST, TRUTH, TABLE_Z, HULL_KEYS            # noqa: E402
from planning.observe import PoseGrid, K                                # noqa: E402
from planning import p1_poses as P                                      # noqa: E402

CENTER = (0.0014, 0.189)
MARKER_OFFSET_Y = -0.0005
DEPTH_BAND = (0.112, 0.130)
GATE = 0.010
GEOMS = [(0.0049, -0.003), (0.0071, -0.003), (0.0120, -0.003), (0.0049, 0.0), (0.0071, 0.0), (0.0120, 0.0)]
SHIFTS = [(dx, dy) for dx in (-0.006, 0.0, 0.006) for dy in (-0.006, 0.0, 0.006)]
B2_REF = np.array([89.0, 89.0, 11.0, 13.0])


def worst(fk, hulls, q, box):
    w = {"box": 1e9, "table": 1e9, "base": 1e9}
    for t in TRUTH.values():
        f = fk.fk(np.array([list(q) + [90.0]]), gains=t.gains, tool_offset=t.tool_offset())
        for cx, off in GEOMS:
            c = clearances(f, hulls, finger_capsules(0.09 + t.tip_y, cx, off), TABLE_Z, dz=t.dz, boxes=[box])
            for k, v in (("box", c["box"]), ("table", c["table"]), ("base", c.get("base", 1e9))):
                w[k] = min(w[k], v)
    return w


def occluded(fk, grid, i, pts):
    o = fk.fk(np.array([list(grid.sdk[i]) + [90.0]]), gains=BEST.gains, tool_offset=BEST.tool_offset())
    p5 = o["arm5"][0][0] + np.array([0, 0, BEST.dz]); R5 = o["arm5"][1][0]; cp, Rc = grid.cp[i], grid.Rc[i]
    def proj(Pt):
        p = Rc.T @ (Pt - cp); return K[0] * p[0] / p[2] + K[2], K[1] * p[1] / p[2] + K[3], p[2]
    for cx, off in GEOMS:
        for a, b, rr in finger_capsules(0.09 + BEST.tip_y, cx, off):
            for t in np.linspace(0, 1, 15):
                u, v, d = proj(p5 + R5 @ (a + (b - a) * t))
                if d > 0 and any((proj(Pt)[0] - u) ** 2 + (proj(Pt)[1] - v) ** 2 < (K[0] * rr / d) ** 2 for Pt in pts):
                    return True
    return False


def main():
    fk = UrdfFK(); grid = PoseGrid(fk, BEST, j234_min=P.J234_MIN); hulls = {k: load_link_hull(v) for k, v in HULL_KEYS.items()}
    half = np.array([P.BLOCK[0] / 2 + 0.003, P.BLOCK[1] / 2 + 0.003, P.BLOCK[2] / 2]); box = Box([CENTER[0], CENTER[1], TABLE_Z + P.BLOCK[2] / 2], half)
    top = TABLE_Z + P.BLOCK[2]; h = P.MARKER / 2; M = np.array([CENTER[0], CENTER[1] + MARKER_OFFSET_Y, top])
    ok = np.ones(len(grid.sdk), bool)
    for dx, dy in SHIFTS:
        for sx in (-1, 1):
            for sy in (-1, 1):
                o, _, _, _ = grid.visible_mask(M + np.array([dx + sx * h, dy + sy * h, 0.0]), 0.0, 40); ok &= o
    v = grid.cp - M; ok &= (v[:, 2] / np.linalg.norm(v, axis=1)) >= np.cos(np.radians(70.0))
    idx = np.flatnonzero(ok); depth = np.einsum("nji,nj->ni", grid.Rc[idx], M - grid.cp[idx])[:, 2]
    idx = idx[(depth >= DEPTH_BAND[0]) & (depth < DEPTH_BAND[1])]
    print("깊이 구간·강건 가시성 후보 %d개" % len(idx), flush=True)
    pts = [M] + [M + np.array([sx * h, sy * h, 0.0]) for sx in (-1, 1) for sy in (-1, 1)]
    good = []
    for i in idx:
        q = [90.0, *[float(x) for x in grid.sdk[i][1:]]]
        if occluded(fk, grid, i, pts):
            continue
        w = worst(fk, hulls, q, box)
        if min(w.values()) >= GATE:
            good.append((i, q, w))
    print("가림 없음·여유 ≥ %.0f mm 통과 %d개" % (GATE * 1000, len(good)), flush=True)
    out = {"depth_band_m": DEPTH_BAND, "gate_mm": GATE * 1000, "n_candidates": int(len(idx)), "n_pass": len(good)}
    if not good:
        out["result"] = "none"; (ROOT / "evidence/G06/p1_b2_poses.json").write_text(json.dumps(out, ensure_ascii=False, indent=1)); return
    jd = lambda q: np.abs(np.array(q[1:]) - B2_REF[1:]).max()   # noqa: E731
    gi, gq, gw = min(good, key=lambda g: (np.linalg.norm(np.array(g[1][1:]) - B2_REF[1:])))
    pairs = [g for g in good if g[0] != gi and np.abs(np.array(g[1][1:]) - np.array(gq[1:])).max() >= 5.0 and np.abs(np.array(g[1][1:]) - np.array(gq[1:])).max() <= 8.0]
    pi, pq, pw = min(pairs, key=lambda g: np.linalg.norm(np.array(g[1][1:]) - np.array(gq[1:]))) if pairs else (None, None, None)
    def desc(i, q, w):
        p = np.einsum("ji,j->i", grid.Rc[i], M - grid.cp[i])
        return {"sdk": q, "depth_mm": round(float(p[2]) * 1000, 1), "marker_center_cam_mm": np.round(p * 1000, 1).tolist(), "tip_xyz_paper_mm": [round(float(grid.tip[i][0]) * 1000, 1), round(float(grid.tip[i][1]) * 1000, 1), round(float(grid.tip[i][2] - TABLE_Z) * 1000, 1)], "tilt_deg": round(float(grid.tilt[i]), 1), "worst_clearance_mm": {k: round(x * 1000, 1) for k, x in w.items()}}
    out.update({"result": "ok", "C": desc(gi, gq, gw), "joint_distance_from_B2_ref": round(float(np.linalg.norm(np.array(gq[1:]) - B2_REF[1:])), 1)})
    if pi is not None:
        out["C2"] = desc(pi, pq, pw); out["C_to_C2_max_joint_delta_deg"] = float(np.abs(np.array(pq[1:]) - np.array(gq[1:])).max())
        out["predicted_marker_shift_C_to_C2_cam_mm"] = np.round((np.array(out["C2"]["marker_center_cam_mm"]) - np.array(out["C"]["marker_center_cam_mm"])), 1).tolist()
    (ROOT / "evidence/G06/p1_b2_poses.json").write_text(json.dumps(out, ensure_ascii=False, indent=1, default=float), encoding="utf-8"); print(json.dumps(out, ensure_ascii=False, indent=1, default=float))


if __name__ == "__main__":
    main()
