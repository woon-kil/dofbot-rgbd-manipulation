import sys, json
sys.path.insert(0,'src')
import numpy as np
from planning import p1_plan_check as C
from planning.collision_3a import Box, load_link_hull
from planning.dry_run import HULL_KEYS, TABLE_Z
plan=json.load(open('evidence/G06/p1_b2_plan20_offline_start89-92-92-92-90-177.json'))
fk=C.UrdfFK(); hulls={k:load_link_hull(v) for k,v in HULL_KEYS.items()}
start=[plan['start'][str(i)] for i in (1,2,3,4)]; fwd=[(m['joint'],m['target']) for m in plan['moves']]
print('낙하 받침 후보(종이 면 위 두께 10 mm 상자)와 팔 최소 간격(mm), 정상+복귀 경로 전체', flush=True)
for name,cx,cy,hx,hy in (('받침 B: x 중심 0, y 92~118 (x ±40)',0.0,0.105,0.040,0.013),):
    C._STATE_CACHE.clear()
    pad=Box([cx,cy,TABLE_Z+0.005],[hx,hy,0.005])
    w=C.eval_moves(fk,hulls,pad,start,fwd)['box']
    for r in plan['return_routes']:
        rm=[(m['joint'],m['target']) for m in r['route']]
        w=min(w,C.eval_moves(fk,hulls,pad,r['state'],rm)['box'])
    print(f'{name}: 최소 간격 {w:.1f} mm', flush=True)
print('끝', flush=True)
