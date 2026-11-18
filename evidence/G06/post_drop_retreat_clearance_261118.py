"""낙하 뒤 복귀(물러남) 경로 간격 점검(오프라인 UNVERIFIED, 로봇 없음). 세트 33 계획의 **들어올린 상태 복귀 항목**(`lift_stage` 있는 12개, 열린 손가락 J6 100)을 `p1_plan_check.eval_moves`로 평가하되,
블록을 정적 접촉 위치가 아니라 **낙하 뒤 위치 포락선**에 둔다. 포락선은 새 숫자 없이 기존 모델에서 도출한다:
 - x: 접촉 검증과 같은 BXS(블록 월드 x = 캡 중점 + d_x, d_x ±3 mm → 7.4·13.4),
 - y: 쥔 블록이 캡 끝과 함께 움직인다는 `carried_block_path`의 `follow` 상태별 캡 y 변위 dy의 최대 절댓값(들어올림 상태 전체 −9.1~+13.8 mm → ±13.8 mm; 프레임 부호 불확실이라 대칭으로 스윕),
 - 모양: (a) 납작·정렬(30×60×30, 접촉 검증의 반폭 +3 mm), (b) 납작 + 모든 yaw를 덮는 외접 정사각(반폭 hypot(15,30)+3 mm), (c) 세워진 경우까지 덮는 외접(높이 60 mm).
블록은 축 정렬 상자(Box)로만 모델한다. 낙하 뒤 미끄러짐·튕김·기울어짐의 실제 분포는 모델 밖이다(한계).
사용: .venv/bin/python -I evidence/G06/post_drop_retreat_clearance_261118.py <세트 33 계획 JSON> [게이트 mm=2.0]"""
import json, os, sys
sys.path[:0] = ["src", "scripts", "evidence/G06"]
os.environ["P1_CAP_OPEN_ONLY_MM"] = "30.9"            # 열린 손가락(접촉 검증과 같은 값)
import numpy as np
import planning.p1_plan_check as PC
from kinematics.urdf_fk import UrdfFK
from planning.collision_3a import Box, load_link_hull
from planning.dry_run import TABLE_Z, HULL_KEYS
from planning import p1_poses as P

plan = json.load(open(sys.argv[1], encoding="utf-8")); gate = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
fk = UrdfFK(); hulls = {k: load_link_hull(v) for k, v in HULL_KEYS.items()}
routes = [r for r in plan["return_routes"] if r.get("lift_stage")]
BXS = (7.4, 13.4); YS = (-13.8, 0.0, 13.8)
bx, by, bz = P.BLOCK; circ = (bx / 2) ** 2 + (by / 2) ** 2
SHAPES = {"a 납작·정렬": (bx / 2 + 0.003, by / 2 + 0.003, bz), "b 납작·모든 yaw": (circ ** 0.5 + 0.003,) * 2 + (bz,), "c 세운 경우까지": (circ ** 0.5 + 0.003,) * 2 + (by,)}
YAWS = (0, 5, 10, 15, 20, 30, 45, 60, 90)
print("낙하 뒤 복귀 경로 간격(mm, 오프라인 UNVERIFIED): 들어올린 복귀 항목 %d개, 게이트 %.1f mm" % (len(routes), gate))
print("[1] 납작한 블록을 yaw로 회전한 상자(Box.yaw_deg, 접촉 검증의 반폭 +3 mm): 위치 포락선 x∈BXS × y∈±13.8 mm 중 최악")
yaw_min = {}
for yaw in YAWS:
    w_all = 1e9; arg = None
    for x in BXS:
        for dy in YS:
            PC._STATE_CACHE.clear()
            box = Box([x / 1000.0, PC.CENTER[1] + dy / 1000.0, TABLE_Z + bz / 2], np.array([bx / 2 + 0.003, by / 2 + 0.003, bz / 2]), yaw_deg=yaw)
            for r in routes:
                rm = [(m["joint"], m["target"]) for m in r["route"]]; e = PC.eval_moves(fk, hulls, box, r["state"], rm)
                if e["box"] < w_all: w_all = e["box"]; arg = (r["state"], x, dy)
    yaw_min[yaw] = w_all; print("  yaw %3d° | box 최소 %6.1f mm (상태·x·dy %s)" % (yaw, w_all, arg), flush=True)
ok = [y for y in YAWS if yaw_min[y] >= gate]
print("== 게이트 %.1f mm를 만족하는 yaw: %s (연속 구간의 끝 %s°)" % (gate, ok, max([y for y in YAWS if all(yaw_min[k] >= gate for k in YAWS if k <= y)] or [None])))
print("[2] (참고) 모든 yaw·세운 경우를 덮는 외접 정사각 상자(보수 상한)는 위 yaw 스윕이 대체한다.")
sys.exit(0)
