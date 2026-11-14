"""③ P1 자세를 오므린 그리퍼(J6=179, 2026-11-15 K1 기립 자세에서 읽음)로 재평가한다(오프라인, 이동 없음).
이전 계산은 J6=98(안쪽 간격 48.7 mm, 캡 중심 x ±29.2 mm)을 가정했다. 오므린 그리퍼의 캡 간격은 실측하지 않았다(UNVERIFIED).
그래서 캡 중심 x를 4.85 mm(간격 0, 캡 두께 9.7 mm가 맞닿음)부터 29.2 mm(열림)까지 바꿔 가며 최악값을 본다. 마커 4모서리·중심이 캡 원판에 가려지는지, 블록·종이·베이스 여유를 계산한다."""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from kinematics.urdf_fk import UrdfFK                                   # noqa: E402
from planning.collision_3a import Box, finger_capsules, load_link_hull, clearances   # noqa: E402
from planning.dry_run import BEST, TRUTH, TABLE_Z, HULL_KEYS            # noqa: E402
from planning.observe import PoseGrid, K                                # noqa: E402
from planning import p1_poses as P                                      # noqa: E402

CENTER = (0.0014, 0.189)
GAPS_CENTER_X = (0.00485, 0.012, 0.020, 0.0292)   # 닫힘(간격 0) ~ 열림(48.7 mm)


def occluded_points(fk, grid, i, pts, caps):
    out = fk.fk(np.array([list(grid.sdk[i]) + [90.0]]), gains=BEST.gains, tool_offset=BEST.tool_offset())
    p5 = out["arm5"][0][0] + np.array([0, 0, BEST.dz]); R5 = out["arm5"][1][0]
    cp, Rc = grid.cp[i], grid.Rc[i]
    proj = lambda Pt: (lambda p: (K[0] * p[0] / p[2] + K[2], K[1] * p[1] / p[2] + K[3], p[2]))(Rc.T @ (Pt - cp))   # noqa: E731
    discs = []
    for a, b, rr in caps:
        for t in np.linspace(0, 1, 15):
            u, v, d = proj(p5 + R5 @ (a + (b - a) * t))
            if d > 0:
                discs.append((u, v, K[0] * rr / d))
    return [any((proj(Pt)[0] - cu) ** 2 + (proj(Pt)[1] - cv) ** 2 < cr ** 2 for cu, cv, cr in discs) for Pt in pts]


def main():
    src = json.load(open(ROOT / "evidence/G06/p1_pose_list_r190.json"))
    fk = UrdfFK(); hulls = {k: load_link_hull(v) for k, v in HULL_KEYS.items()}
    grid = PoseGrid(fk, BEST, j234_min=P.J234_MIN)
    half = np.array([P.BLOCK[0] / 2 + 0.003, P.BLOCK[1] / 2 + 0.003, P.BLOCK[2] / 2])
    box = Box([CENTER[0], CENTER[1], TABLE_Z + P.BLOCK[2] / 2], half)
    top = TABLE_Z + P.BLOCK[2]; h = P.MARKER / 2
    M = np.array([CENTER[0], CENTER[1] - 0.0005, top])
    pts = [M] + [M + np.array([sx * h, sy * h, 0.0]) for sx in (-1, 1) for sy in (-1, 1)]
    out = {}
    for name, v in src["poses"].items():
        if name == "HOME":
            continue
        q = np.array(v["sdk"], float); i = int(np.argmin(np.linalg.norm(grid.sdk - q, axis=1)))
        rows = []
        for cx in GAPS_CENTER_X:
            occ = occluded_points(fk, grid, i, pts, finger_capsules(0.09 + BEST.tip_y, cx))
            w = {"box": 1e9, "table": 1e9, "base": 1e9}
            for t in TRUTH.values():
                f = fk.fk(np.array([list(q) + [90.0]]), gains=t.gains, tool_offset=t.tool_offset())
                c = clearances(f, hulls, finger_capsules(0.09 + t.tip_y, cx), TABLE_Z, dz=t.dz, boxes=[box])
                w["box"] = min(w["box"], c["box"]); w["table"] = min(w["table"], c["table"]); w["base"] = min(w["base"], c.get("base", 1e9))
            rows.append({"cap_center_x_mm": round(cx * 1000, 2), "occluded_pts_of_5(center+4corners)": int(sum(occ)), "worst_clearance_mm": {k: round(x * 1000, 1) for k, x in w.items()}})
        out[name] = rows
    (ROOT / "evidence/G06/p1_closed_gripper.json").write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    for n, rows in out.items():
        for r in rows:
            print(n, r)


if __name__ == "__main__":
    main()
