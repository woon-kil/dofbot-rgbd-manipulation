"""실제 눈금자 자리(R13 뒤 Gemini 사진 d1_trigger_test_261116.png, 평면 모델 환산 프로젝트 x 107~129, y 174~176 ±10 mm)에서 세트 21·22 계획 정상+복귀 경로 간격.
충돌 프레임(FK) x = -프로젝트 x. 사용: python3 -I ... <plan.json>  (P1_DZ_EXTRA_MM 환경변수 반영)"""
import sys, json
sys.path.insert(0,'src')
import numpy as np
from planning import p1_plan_check as C
from planning.collision_3a import Box, load_link_hull
from planning.dry_run import HULL_KEYS, TABLE_Z
plan=json.load(open(sys.argv[1]))
fk=C.UrdfFK(); hulls={k:load_link_hull(v) for k,v in HULL_KEYS.items()}
start=[plan['start'][str(i)] for i in (1,2,3,4)]; fwd=[(m['joint'],m['target']) for m in plan['moves']]
for xp in (107,118,129):
  for sgn in (-1,+1):
    C._STATE_CACHE.clear()
    ruler=Box([sgn*xp/1000,0.175,TABLE_Z+0.06],[0.0125,0.0125,0.06])   # 안전하게 25x25 mm 단면(기울어진 눈금자 여유)
    w=C.eval_moves(fk,hulls,ruler,start,fwd)['box']
    for r in plan['return_routes']:
        rm=[(m['joint'],m['target']) for m in r['route']]
        w=min(w,C.eval_moves(fk,hulls,ruler,r['state'],rm)['box'])
    print(f'{os.path.basename(sys.argv[1]) if False else sys.argv[1].split("/")[-1][:30]} FK x={sgn*xp:+d} y=175: 최소 간격 {w:.1f} mm', flush=True)
