"""③ 큰 이동 M3 자세 쌍 후보(오프라인, 이동 명령 없음). 질문: 관측 자세에서 tip을 50~100 mm 옮기는 이동의 모델 예측 오차가 이동량에 어떻게 비례하는가.
조건: 두 자세 모두 마커(25 mm)가 4모서리·강건 가시성(마커 위치 ±6 mm 흔들림, 가장자리 40 px), 깊이 0.13~0.32 m(0.19 m 이하만 보면 후보끼리 tip 최대 24 mm 떨어져 큰 이동 쌍이 없어, 멀리서 보는 구간까지 넓힘 — 카메라 이동의 FK 예측 오차는 마커 깊이와 무관하게 비교 가능), 입사각 ≤ 70°, 캡 가림 없음,
이미 이동해 본 높이 영역 안(종이 여유 ≥ 55 mm = P1 자세 B의 56.4 mm 수준, 블록 ≥ 20 mm, 베이스 ≥ 25 mm), 각 자세 J3 ≥ 10·J4 ≥ 11.
이동은 단일 관절 순서의 경로 전체도 같은 여유를 만족해야 한다. 쌍: tip 이동 50~100 mm, 관절 한 개 이상이 ≥ 5°(회독 1°가 이동의 20 % 이하), 최대 관절 이동 ≤ 20°."""
import json
import sys
import itertools
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from kinematics.urdf_fk import UrdfFK                                   # noqa: E402
from planning.collision_3a import Box, load_link_hull                   # noqa: E402
from planning.dry_run import BEST, HULL_KEYS, TABLE_Z                   # noqa: E402
from planning.observe import PoseGrid                                   # noqa: E402
from planning import p1_poses as P                                      # noqa: E402
from planning import p1_b2_poses as B                                   # noqa: E402
from planning.p1_plan_check import eval_moves                           # noqa: E402

DEPTH = (0.130, 0.320)
GATES = {"table": 55.0, "box": 20.0, "base": 25.0}
TIP_MOVE = (0.050, 0.100)
J2_RANGE = (74.0, 105.0)   # 실행 기록 최저 J2 74, 최고 약 105(러너 창 70~110 안)


def main():
    fk = UrdfFK(); grid = PoseGrid(fk, BEST, j234_min=P.J234_MIN); hulls = {k: load_link_hull(v) for k, v in HULL_KEYS.items()}
    half = np.array([P.BLOCK[0] / 2 + 0.003, P.BLOCK[1] / 2 + 0.003, P.BLOCK[2] / 2]); box = Box([B.CENTER[0], B.CENTER[1], TABLE_Z + P.BLOCK[2] / 2], half)
    top = TABLE_Z + P.BLOCK[2]; h = P.MARKER / 2; M = np.array([B.CENTER[0], B.CENTER[1] + B.MARKER_OFFSET_Y, top])
    ok = np.ones(len(grid.sdk), bool)
    for dx, dy in B.SHIFTS:
        for sx in (-1, 1):
            for sy in (-1, 1):
                o, _, _, _ = grid.visible_mask(M + np.array([dx + sx * h, dy + sy * h, 0.0]), 0.0, 40, max_tilt=90.0)   # tilt 한도는 파지 접근용이라 M3에는 제외; ok &= o
    v = grid.cp - M; ok &= (v[:, 2] / np.linalg.norm(v, axis=1)) >= np.cos(np.radians(70.0))
    idx = np.flatnonzero(ok); depth = np.einsum("nji,nj->ni", grid.Rc[idx], M - grid.cp[idx])[:, 2]
    idx = idx[(depth >= DEPTH[0]) & (depth < DEPTH[1])]
    pts = [M] + [M + np.array([sx * h, sy * h, 0.0]) for sx in (-1, 1) for sy in (-1, 1)]
    S = []
    for i in idx:
        if B.occluded(fk, grid, i, pts):
            continue
        q = [90.0, *[float(x) for x in grid.sdk[i][1:]]]
        if not (J2_RANGE[0] <= q[1] <= J2_RANGE[1] and q[2] <= 105 and q[3] <= 105):
            continue   # 러너 승인 창(J2 70~110, J3·J4 10~105)과 지금까지 이동한 J2 범위 안으로 제한
        w = B.worst(fk, hulls, q, box); wm = {k: x * 1000 for k, x in w.items()}
        if all(wm[k] >= GATES[k] for k in GATES):
            S.append((i, q, wm))
    print("관측 후보 %d → 게이트 통과 %d" % (len(idx), len(S)), flush=True)
    pairs = []
    for (i, qi, wi), (j, qj, wj) in itertools.combinations(S, 2):
        d = np.abs(np.array(qi[1:]) - np.array(qj[1:])); mv = float(np.linalg.norm(grid.tip[i] - grid.tip[j]))
        if d.max() >= 5.0 and d.max() <= 20.0 and TIP_MOVE[0] <= mv <= TIP_MOVE[1]:
            pairs.append((i, j, qi, qj, mv, float(d.max())))
    print("쌍 후보(이동 조건) %d" % len(pairs), flush=True)
    out = {"n_obs": int(len(idx)), "n_pass": len(S), "n_pairs": len(pairs), "gates_mm": GATES, "tip_move_m": TIP_MOVE, "depth_m": DEPTH, "pairs": []}
    pairs.sort(key=lambda p: (p[5], p[4]))   # 최대 관절 이동이 작은 순
    for i, j, qi, qj, mv, dm in pairs[:60]:
        # 단일 관절 순서 경로(관절 한 개씩, 번호 순)의 여유
        cur = list(qi); moves = [(k + 1, qj[k]) for k in range(4) if qj[k] != qi[k]]
        w = eval_moves(fk, hulls, box, cur, moves)
        if all(w[k] >= GATES[k] for k in GATES):
            pi = np.einsum("ji,j->i", grid.Rc[i], M - grid.cp[i]); pj = np.einsum("ji,j->i", grid.Rc[j], M - grid.cp[j])
            out["pairs"].append({"from": qi, "to": qj, "tip_move_mm": round(mv * 1000, 1), "max_joint_delta_deg": dm, "joint_deltas": [round(qj[k] - qi[k], 1) for k in range(1, 4)],
                                 "depth_from_mm": round(float(pi[2]) * 1000, 1), "depth_to_mm": round(float(pj[2]) * 1000, 1), "predicted_marker_shift_cam_mm": np.round((pj - pi) * 1000, 1).tolist(), "path_worst_mm": w})
    (ROOT / "evidence/G06/p1_bigm3_pairs.json").write_text(json.dumps(out, ensure_ascii=False, indent=1, default=float), encoding="utf-8")
    print("경로 통과 쌍 %d" % len(out["pairs"]))
    for p in out["pairs"][:8]:
        print(p["from"], "→", p["to"], "tip", p["tip_move_mm"], "mm", "관절", p["joint_deltas"], "깊이", p["depth_from_mm"], "→", p["depth_to_mm"], "예측 변위", p["predicted_marker_shift_cam_mm"], "경로", p["path_worst_mm"])


if __name__ == "__main__":
    main()
