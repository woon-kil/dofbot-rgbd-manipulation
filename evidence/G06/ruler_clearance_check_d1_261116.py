"""세트 22 최종 계획(--return-nominal, P1_DZ_EXTRA_MM=45)에서 눈금자 후보 자리의 정상+복귀 경로 최소 간격. FK x 후보와 y를 인자로: python3 -I ... plan.json x1,x2,.. y"""
import sys, json
sys.path.insert(0,'src')
from planning import p1_plan_check as C
from planning.collision_3a import Box, load_link_hull
from planning.dry_run import HULL_KEYS, TABLE_Z
plan=json.load(open(sys.argv[1])); xs=[int(v) for v in sys.argv[2].split(',')]; y=int(sys.argv[3])
fk=C.UrdfFK(); hulls={k:load_link_hull(v) for k,v in HULL_KEYS.items()}
start=[plan['start'][str(i)] for i in (1,2,3,4)]; fwd=[(m['joint'],m['target']) for m in plan['moves']]
for x in xs:
    C._STATE_CACHE.clear()
    ruler=Box([x/1000,y/1000,TABLE_Z+0.06],[0.0125,0.0125,0.06])   # 25x25 mm 단면(기울어진 눈금자 여유), 높이 120 mm
    w=C.eval_moves(fk,hulls,ruler,start,fwd)['box']
    for r in plan['return_routes']:
        rm=[(m['joint'],m['target']) for m in r['route']]
        w=min(w,C.eval_moves(fk,hulls,ruler,r['state'],rm)['box'])
    print(f'FK x={x:+d} y={y}: 최소 간격 {w:.1f} mm', flush=True)
print('끝', flush=True)
