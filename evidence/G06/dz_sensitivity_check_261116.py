"""충돌 검증의 손끝 높이 오차(dz) 가정 민감도(오프라인, 하드웨어 없음). 사용: python3 -I dz_sensitivity_check.py 계획.json 출력.json DZ_MM(예: 25)
p1_plan_check의 TRUTH 변형 중 dz를 -DZ_MM로 바꾼 사본을 추가한 뒤 같은 검증을 돌린다(원본 변형도 유지)."""
import sys, json, dataclasses, os
sys.path.insert(0, 'src')
from planning import dry_run
from planning import p1_plan_check as C
plan_path, out_path, dz_mm = sys.argv[1], sys.argv[2], float(sys.argv[3])
extra = {}
for k, t in list(dry_run.TRUTH.items()):
    extra[k + "_dz%d" % int(dz_mm)] = dataclasses.replace(t, dz=-dz_mm / 1000.0)
dry_run.TRUTH.update(extra)
sys.argv = ['p1_plan_check', plan_path, out_path]
C.main()
