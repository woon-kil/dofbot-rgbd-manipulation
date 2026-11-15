#!/usr/bin/env python3
"""S3(J4+J3 동시 접기) 경로의 URDF 순기구학 값(오프라인, 하드웨어 없음, 모델 값=UNVERIFIED). g01_pose_generator.py의 fk를 재사용한다."""
import sys, json, numpy as np
src = open("scripts/g01_pose_generator.py", encoding="utf-8").read().split("# 후보 격자")[0]
sys.argv = ["x"]; ns = {"__file__": "scripts/g01_pose_generator.py"}; exec(compile(src, "pose_generator_head", "exec"), ns)
fk = ns["fk"]
BOARD_FRONT_Y = 230 - 30   # 보드 앞(베이스에서 가까운 쪽) 가장자리 y mm, 보드 중심 230, 앞뒤 60
box = np.array([[sx*0.045, sy*0.0125, sz*0.0125] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
def row(j2, j3, j4):
    g = np.array([[90., j2, j3, j4, 90.]]); f = fk(g); tcp = f["tcp"][0][0]*1000; dp, dR = f["dabai"]; dpm = dp[0]*1000
    zmin = (dp[0, 2] + (dR[0] @ box.T)[2]).min()*1000
    pts = {n: f[n][0][0]*1000 for n in ("Arm2_Joint", "Arm3_Joint", "Arm4_Joint", "arm5")}
    chain_z = min(min(p[2] for p in pts.values()), tcp[2], zmin)
    return {"j2": j2, "j3": j3, "j4": j4, "tcp": [round(x, 1) for x in tcp], "dabai": [round(x, 1) for x in dpm], "min_z_mm": round(chain_z, 1), "tcp_y_to_board_front_mm": round(BOARD_FRONT_Y - tcp[1], 1)}
if __name__ == "__main__":
    out = []
    for dlt in (0, 20, 40, 60, 80):
        r = row(91., 91. - dlt, 91. - dlt); r["delta"] = dlt; out.append(r); print(json.dumps(r, ensure_ascii=False))
    print("-- 후보 자세 쪽(J3=15, J4=10, J2=90):", json.dumps(row(90., 15., 10.), ensure_ascii=False))
    json.dump(out, open("evidence/G01/s3_path_fk.json", "w"), ensure_ascii=False, indent=1)
