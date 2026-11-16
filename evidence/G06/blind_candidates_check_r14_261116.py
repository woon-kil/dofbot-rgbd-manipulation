"""[2026-11-16 R14 재계산] D1-A로 확정한 TCP→캡 아래끝 오프셋 22.5 mm를 BEST.dz(-0.0225)에 반영하고 안전 변형은 dz -26 mm(오프셋 + 3 mm)로 평가한다. 원본: blind_candidates_check_261116.py
B6-3: 최소 blind 길이 후보 자세의 오프라인 안전 검증(하드웨어 없음). 마커 4모서리 가시(여유 47 px, 깊이 ≥ 100 mm), 어댑터 창(J2 ≥ 70, J3 10~105, J4 11~105),
캡 가림 없음, 정지 자세 충돌 여유(TRUTH 변형 + dz -25 mm 변형, 캡 5종 최악)가 게이트 이상인 자세를 blind 길이(d=10 파지) 순으로 정리한다."""
import sys, json, dataclasses
sys.path.insert(0, 'src')
import numpy as np
from kinematics.urdf_fk import UrdfFK
from planning.observe import PoseGrid
from planning import dry_run
from planning.dry_run import BEST, TABLE_Z, HULL_KEYS, TRUTH
from planning.collision_3a import Box, load_link_hull
from planning import p1_b2_poses as B, p1_poses as P
BEST = dataclasses.replace(BEST, dz=-0.0225); TRUTH['best'] = BEST; dry_run.BEST = BEST   # R14: 캡 아래끝 = TCP - 22.5 mm
N_TOP = int(sys.argv[1]) if len(sys.argv) > 1 else 250
GATE = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
DZ_MM = float(sys.argv[3]) if len(sys.argv) > 3 else 26.0   # 보수 가정 dz(mm): 손끝 최저점이 모델보다 낮은 정도
for k, t in list(TRUTH.items()):
    TRUTH[k + '_dzX'] = dataclasses.replace(t, dz=-DZ_MM / 1000.0)
fk = UrdfFK(); hulls = {k: load_link_hull(v) for k, v in HULL_KEYS.items()}
grid = PoseGrid(fk, BEST, step=1.0, j234_min=(70.0, 10.0, 11.0), j234_max=(110.0, 105.0, 105.0))
top = TABLE_Z + P.BLOCK[2]; h = P.MARKER / 2; M = np.array([B.CENTER[0], B.CENTER[1] + B.MARKER_OFFSET_Y, top])
half = np.array([P.BLOCK[0] / 2 + 0.003, P.BLOCK[1] / 2 + 0.003, P.BLOCK[2] / 2]); box = Box([B.CENTER[0], B.CENTER[1], TABLE_Z + P.BLOCK[2] / 2], half)
ok = np.ones(len(grid.sdk), bool)
for dx, dy in B.SHIFTS:
    for sx in (-1, 1):
        for sy in (-1, 1):
            o, u, v, d = grid.visible_mask(M + np.array([dx + sx * h, dy + sy * h, 0.0]), 0.10, 47.0, max_tilt=90.0); ok &= o
idx = np.flatnonzero(ok)
tgt10 = np.array([B.CENTER[0], B.CENTER[1], top - 0.010]); tgt20 = np.array([B.CENTER[0], B.CENTER[1], top - 0.020])
d10 = np.linalg.norm(grid.tip[idx] - tgt10, axis=1) * 1000; d20 = np.linalg.norm(grid.tip[idx] - tgt20, axis=1) * 1000
order = np.argsort(d10)[:N_TOP]
pts = [M] + [M + np.array([sx * h, sy * h, 0.0]) for sx in (-1, 1) for sy in (-1, 1)]
rows = []; n_occ = 0
for r in order:
    i = int(idx[r])
    if B.occluded(fk, grid, i, pts):
        n_occ += 1; continue
    q = [90.0, *[float(x) for x in grid.sdk[i][1:]]]
    w = {k: round(v * 1000, 1) for k, v in B.worst(fk, hulls, q, box).items()}
    p = np.einsum("ji,j->i", grid.Rc[i], M - grid.cp[i])
    rows.append({"q": q[1:], "blind_d10_mm": round(float(d10[r]), 1), "blind_d20_mm": round(float(d20[r]), 1), "FK_TCP_z_plate_mm": round(float((grid.tip[i][2] - BEST.dz) * 1000), 1),
                 "depth_mm": round(float(p[2]) * 1000, 1), "clearance_mm_dzX": w, "pass": all(w[k] >= GATE for k in w)})
print(f'가시 {len(idx)}자세 중 blind 길이 상위 {N_TOP} 평가: 캡 가림 {n_occ}, 평가 {len(rows)}, 게이트 {GATE} mm 통과 {sum(1 for r in rows if r["pass"])}', flush=True)
good = [r for r in rows if r["pass"]]
for r in good[:12]:
    print(r, flush=True)
json.dump({"gate_mm": GATE, "dz_mm": DZ_MM, "n_visible": int(len(idx)), "n_top": N_TOP, "n_occluded": n_occ, "evaluated": rows}, open('evidence/G06/blind_candidates_check_r14_all_dz%d.json' % int(DZ_MM), 'w'), ensure_ascii=False, indent=1)
print('끝', flush=True)
