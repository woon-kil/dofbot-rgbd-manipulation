"""T1D-3 1단계 작업 2: blind 이동 경로와 관절 창 확대 후보의 충돌·높이 검토(오프라인, 하드웨어 없음).

관측 후보(전체 평가에서 dz -25 게이트를 통과한 자세 중 blind 길이가 짧은 것과 종이 여유가 큰 것)에서 파지 자세(블록 중심 윗면 d=10 mm 아래, tilt 20/30° 이내, 목표 3 mm 이내)까지
한 번에 한 관절씩 이동하는 두 가지 순서의 경로를 만들어 블록·종이·베이스 최소 여유(dz -25 변형 포함, 캡 5종 최악)와 경로 중 최소 J2·최대 J4를 보고한다.
블록 여유는 마지막 이동에서 파지(접촉)를 향하므로 음수가 되는 것이 정상이다. 종이·베이스 여유와 J2 최소·J4 최대가 창 확대 후보의 근거다.
사용: python3 -I blind_path_window_check_261116.py [후보 수]
"""
import sys, json, dataclasses, itertools
sys.path.insert(0, 'src')
import numpy as np
from kinematics.urdf_fk import UrdfFK
from planning.observe import PoseGrid
from planning import dry_run
from planning.dry_run import BEST, TABLE_Z, HULL_KEYS, TRUTH
from planning.collision_3a import Box, load_link_hull
from planning import p1_b2_poses as B, p1_poses as P
from planning import p1_plan_check as C

N_EACH = int(sys.argv[1]) if len(sys.argv) > 1 else 3
for k, t in list(TRUTH.items()):
    TRUTH[k + '_dz25'] = dataclasses.replace(t, dz=-0.025)
fk = UrdfFK(); hulls = {k: load_link_hull(v) for k, v in HULL_KEYS.items()}
half = np.array([P.BLOCK[0] / 2 + 0.003, P.BLOCK[1] / 2 + 0.003, P.BLOCK[2] / 2]); box = Box([B.CENTER[0], B.CENTER[1], TABLE_Z + P.BLOCK[2] / 2], half)
top = TABLE_Z + P.BLOCK[2]
grid = PoseGrid(fk, BEST, step=1.0, j234_min=(20.0, 0.0, 0.0), j234_max=(110.0, 150.0, 150.0))
tgt = np.array([B.CENTER[0], B.CENTER[1], top - 0.010])
dist = np.linalg.norm(grid.tip - tgt, axis=1) * 1000.0

allc = json.load(open('evidence/G06/blind_candidates_check_261116_all_dz25.json'))['evaluated']
ok = [x for x in allc if x['pass']]
short = sorted(ok, key=lambda x: x['blind_d10_mm'])[:N_EACH]
paper = sorted([x for x in ok if x['clearance_mm_dz25' if 'clearance_mm_dz25' in x else 'clearance_mm_dzX']['table'] >= 30], key=lambda x: x['blind_d10_mm'])[:N_EACH]
cands = [('짧은 blind', x) for x in short] + [('종이 여유 큰', x) for x in paper]
print(f'후보 {len(cands)}개(짧은 blind {len(short)}, 종이 여유 ≥ 30 mm {len(paper)})', flush=True)

orders = {'J4→J2→J3': (4, 2, 3), 'J2→J4→J3': (2, 4, 3)}      # 관절 번호(J2=2, J3=3, J4=4), 한 번에 한 관절
results = []
for label, c in cands:
    q0 = c['q']
    for tmax in (20.0, 30.0):
        sel = np.flatnonzero((dist <= 3.0) & (np.abs(grid.tilt) <= tmax))
        dd = np.abs(grid.sdk[sel][:, 1:] - np.array(q0)).sum(1); gi = sel[int(np.argmin(dd))]
        qg = [float(x) for x in grid.sdk[gi][1:]]
        for oname, order in orders.items():
            moves = [(j, qg[j - 2]) for j in order]      # (관절 번호, 목표 각도), qg = [J2, J3, J4]
            C._STATE_CACHE.clear()
            w = C.eval_moves(fk, hulls, box, [90.0] + list(q0), [(m[0], m[1]) for m in moves])
            # 경로 중 J2 최소·J4 최대(이동 순서대로 갱신)
            q = [90.0] + list(q0); j2min, j4max = q[1], q[3]
            for jj, tt in moves:
                q[jj - 1] = tt; j2min = min(j2min, q[1]); j4max = max(j4max, q[3])
            r = {'group': label, 'obs_q': q0, 'blind_d10_mm': c['blind_d10_mm'], 'tilt_max': tmax, 'grasp_q': qg, 'tilt_grasp': round(float(grid.tilt[gi]), 1), 'grasp_err_mm': round(float(dist[gi]), 1),
                 'order': oname, 'moves': [(f'J{m[0]}', m[1]) for m in moves], 'path_min_mm_dz25': w, 'j2_min': j2min, 'j4_max': j4max}
            results.append(r)
            print(r, flush=True)
json.dump(results, open('evidence/G06/blind_path_window_check_261116.json', 'w'), ensure_ascii=False, indent=1)
print('끝', flush=True)
