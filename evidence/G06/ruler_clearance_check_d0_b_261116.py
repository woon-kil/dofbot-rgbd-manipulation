import sys, json
sys.path.insert(0,'src')
import numpy as np
from planning import p1_plan_check as C
from planning.collision_3a import Box, load_link_hull
from planning.dry_run import HULL_KEYS, TABLE_Z
plan=json.load(open('evidence/G06/p1_b2_plan21_offline_start89-92-92-92-90-177.json'))
fk=C.UrdfFK(); hulls={k:load_link_hull(v) for k,v in HULL_KEYS.items()}
start=[plan['start'][str(i)] for i in (1,2,3,4)]; fwd=[(m['joint'],m['target']) for m in plan['moves']]
print('세트 21(D0) 눈금자 자리 후보: 충돌 프레임(FK) x(손끝은 x -68, y 187), 단면 25(y)x3(x) mm, 높이 120 mm', flush=True)
for y in (187,):
    for x in (-110,-130,-150,-170,+110,+150):
        C._STATE_CACHE.clear()
        ruler=Box([x/1000,y/1000,TABLE_Z+0.06],[0.0015,0.0125,0.06])
        w=C.eval_moves(fk,hulls,ruler,start,fwd)['box']
        for r in plan['return_routes']:
            rm=[(m['joint'],m['target']) for m in r['route']]
            w=min(w,C.eval_moves(fk,hulls,ruler,r['state'],rm)['box'])
        print(f'x={x:+d} y={y}: 정상+복귀 전체 최소 간격 {w:.1f} mm', flush=True)
print('끝', flush=True)
