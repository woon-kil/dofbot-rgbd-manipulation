import sys, json
sys.path.insert(0,'src')
import numpy as np
from planning import p1_plan_check as C
from planning.collision_3a import Box, load_link_hull
from planning.dry_run import HULL_KEYS, TABLE_Z
plan=json.load(open('evidence/G06/p1_b2_plan21_offline_start89-92-92-92-90-177.json'))
fk=C.UrdfFK(); hulls={k:load_link_hull(v) for k,v in HULL_KEYS.items()}
start=[plan['start'][str(i)] for i in (1,2,3,4)]; fwd=[(m['joint'],m['target']) for m in plan['moves']]
print('실제 눈금자 자리(Gemini 사진 추정: 블록 중심 기준 옆 약 97 mm, 베이스 쪽 약 14 mm → 월드 y ≈ 175 mm), 단면 25(y)x3(x) mm, 높이 120 mm', flush=True)
for sx in (+1,-1):
    C._STATE_CACHE.clear()
    ruler=Box([sx*0.097,0.175,TABLE_Z+0.06],[0.003,0.0125,0.06])
    w=C.eval_moves(fk,hulls,ruler,start,fwd)['box']
    for r in plan['return_routes']:
        rm=[(m['joint'],m['target']) for m in r['route']]
        w=min(w,C.eval_moves(fk,hulls,ruler,r['state'],rm)['box'])
    print(f'x={sx*97:+d} mm, y=175 mm: 최소 간격 {w:.1f} mm', flush=True)
print('끝', flush=True)
