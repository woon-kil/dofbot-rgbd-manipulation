#!/usr/bin/env python3
"""S3 write6: (J3, J4) 직사각형 전체([12..92]×[11..91], J2=91 J1=J5=90)에서 작업대·보드 위 최저 높이를 모델(URDF FK)로 계산한다. 한쪽 서보만 움직인 중간 자세 포함. 하드웨어 없음, 모델 값=UNVERIFIED."""
import sys, json, numpy as np
sys.path.insert(0, "scripts")
import importlib.util
spec = importlib.util.spec_from_file_location("s3", "scripts/g01_s3_path.py"); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
fk, box = m.fk, m.box
def pts(j3, j4):
    f = fk(np.array([[90., 91., j3, j4, 90.]])); dp, dR = f["dabai"]
    P = {"tcp": f["tcp"][0][0], "arm5": f["arm5"][0][0], "arm4": f["Arm4_Joint"][0][0], "arm3": f["Arm3_Joint"][0][0], "dabai_c": dp[0]}
    for k, c in enumerate(box): P["dabai_c%d" % k] = dp[0] + dR[0] @ c
    return {k: v * 1000 for k, v in P.items()}
J3S = [92, 72, 52, 32, 12]; J4S = [91, 71, 51, 31, 11]
def scan(j3a, j3b, j4a, j4b, n=17):
    worst_t = (1e9, None, None); worst_b = (1e9, None, None)
    for u in np.linspace(0, 1, n):
        for v in np.linspace(0, 1, n):
            j3 = j3a + (j3b - j3a) * u; j4 = j4a + (j4b - j4a) * v
            for k, p in pts(j3, j4).items():
                if p[2] < worst_t[0]: worst_t = (round(p[2], 1), k, (round(j3), round(j4)))
                if abs(p[0]) <= 45 and 185 <= p[1] <= 275 and p[2] < worst_b[0]: worst_b = (round(p[2], 1), k, (round(j3), round(j4)))
    return worst_t, worst_b
out = {"grid25": [], "rectangle": None}
print("격자 25점(J3 계단값 × J4 계단값, J2=91): 작업대 위 최저 높이 mm / 보드 위(±45 x, y 185~275) 최저 mm")
for j3 in J3S:
    row = []
    for j4 in J4S:
        P = pts(j3, j4); zt = min(p[2] for p in P.values()); zb = min([p[2] for p in P.values() if abs(p[0]) <= 45 and 185 <= p[1] <= 275] or [float("nan")])
        row.append("%5.0f/%s" % (zt, "  -  " if zb != zb else "%5.0f" % zb)); out["grid25"].append({"j3": j3, "j4": j4, "min_z": round(zt, 1), "over_board_min_z": None if zb != zb else round(zb, 1), "tcp": [round(x, 1) for x in P["tcp"]]})
    print("J3=%2d: " % j3 + "  ".join(row))
wt, wb = scan(12, 92, 11, 91)
print("직사각형 전체(1° 근방 격자): 작업대 위 최저 %.1f mm (%s, J3/J4=%s), 보드 위 최저 %.1f mm (%s, J3/J4=%s)" % (wt + wb))
out["rectangle"] = {"min_z_table": wt, "min_z_over_board": wb}
json.dump(out, open("evidence/G01/s3_grid_clearance.json", "w"), ensure_ascii=False, indent=1)
