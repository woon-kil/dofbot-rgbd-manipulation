import sys, json
sys.path.insert(0,'src')
import numpy as np
from planning import p1_plan_check as C
from planning.collision_3a import Box, load_link_hull
from planning.dry_run import HULL_KEYS, TABLE_Z
plan=json.load(open('evidence/G06/p1_b2_plan20_offline_start89-92-92-92-90-177.json'))
fk=C.UrdfFK(); hulls={k:load_link_hull(v) for k,v in HULL_KEYS.items()}
start=[plan['start'][str(i)] for i in (1,2,3,4)]; fwd=[(m['joint'],m['target']) for m in plan['moves']]
print('눈금자(단면 10x4 mm, 높이 100 mm, 월드 y=134.3 mm)와 팔 최소 간격(mm), 정상+복귀 경로 전체', flush=True)
for sx,ox in ((+1,0.080),(-1,0.080),(+1,0.060),(+1,0.040),(+1,0.020)):
    C._STATE_CACHE.clear()
    ruler=Box([sx*ox,0.1343,TABLE_Z+0.05],[0.005,0.002,0.05])
    w=C.eval_moves(fk,hulls,ruler,start,fwd)['box']
    for r in plan['return_routes']:
        rm=[(m['joint'],m['target']) for m in r['route']]
        w=min(w,C.eval_moves(fk,hulls,ruler,r['state'],rm)['box'])
    print(f'옆 {sx*ox*1000:+.0f} mm: 최소 간격 {w:.1f} mm', flush=True)
print('끝', flush=True)
