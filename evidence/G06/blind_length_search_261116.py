"""B6-1(a)(b): G06 관측→파지 이동(tip 직선거리 하한)의 정의 확인과, 관절 창·높이 게이트 아래에서 마커가 보이는 자세 중 파지 tip에 가장 가까운 자세의 blind 길이 재계산(오프라인).
정의(G06 `src/planning/observe.py`): 관측 자세 tip(BEST 모델의 dz 포함)과 파지 tip 목표 사이 직선거리, 마커 4모서리(위치 흔들림 SHIFTS 포함)가 영상 가장자리 margin px 안에 보이는 자세만."""
import sys, json
sys.path.insert(0, 'src')
import numpy as np
from kinematics.urdf_fk import UrdfFK
from planning.observe import PoseGrid
from planning.dry_run import BEST, TABLE_Z
from planning import p1_b2_poses as B, p1_poses as P
fk = UrdfFK()
J2MIN = float(sys.argv[2]) if len(sys.argv) > 2 else 70.0
grid = PoseGrid(fk, BEST, step=1.0, j234_min=(J2MIN, 10.0, 11.0), j234_max=(110.0, 105.0, 105.0))     # 어댑터 창 J2 70~110, J3 10~105, J4 10~105(J4는 P1 규칙 11)
top = TABLE_Z + P.BLOCK[2]; h = P.MARKER / 2; M = np.array([B.CENTER[0], B.CENTER[1] + B.MARKER_OFFSET_Y, top])
MARGIN = float(sys.argv[1]) if len(sys.argv) > 1 else 47.0
ok = np.ones(len(grid.sdk), bool); n0 = len(ok)
for dx, dy in B.SHIFTS:
    for sx in (-1, 1):
        for sy in (-1, 1):
            o, u, v, d = grid.visible_mask(M + np.array([dx + sx * h, dy + sy * h, 0.0]), 0.10, MARGIN, max_tilt=90.0)   # 깊이 ≥ 0.10 m
            ok &= o
print(f'격자 {n0}자세 → 마커 4모서리 보임(여유 {MARGIN:.0f} px, 깊이 ≥ 100 mm) {int(ok.sum())}자세', flush=True)
idx = np.flatnonzero(ok)
tip = grid.tip[idx]; raw_z = (tip[:, 2] - BEST.dz) * 1000.0             # 판 윗면 기준 FK TCP 높이(mm, BEST dz 제거 = 명목 FK)
res = {}
for d_mm in (10, 20):
    tgt = np.array([B.CENTER[0], B.CENTER[1], TABLE_Z + P.BLOCK[2] - d_mm / 1000.0])   # 파지 TCP(윗면에서 d mm 아래, 월드)
    dist = np.linalg.norm(tip - tgt, axis=1) * 1000.0
    for gate in (0.0, 30.0, 44.0):
        m = raw_z >= gate
        if not m.any():
            res[f'd{d_mm}_gate{int(gate)}'] = None; continue
        k = int(np.argmin(np.where(m, dist, 1e9))); i = idx[k]
        res[f'd{d_mm}_gate{int(gate)}'] = {'blind_mm': round(float(dist[k]), 1), 'q(J2,J3,J4)': grid.sdk[i][1:].tolist(), 'FK_TCP_z_plate_mm': round(float(raw_z[k]), 1), 'n_candidates': int(m.sum())}
for k, v in res.items():
    print(k, v, flush=True)
json.dump({'margin_px': MARGIN, 'j2_min': J2MIN, 'n_visible': int(ok.sum()), 'results': res}, open('evidence/G06/blind_length_search_261116_m%d_j2min%d.json' % (int(MARGIN), int(J2MIN)), 'w'), ensure_ascii=False, indent=1)
