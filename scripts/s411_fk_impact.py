#!/usr/bin/env python3
"""FK 영점 오차가 접근 단계(개방 루프)·마무리 단계(폐루프 상대량)에 미치는 영향 상한(오프라인). 하드웨어 없음.
1) 카메라→TCP 상대 변환이 J1~J4 영점 오차에 불변인지 수치 확인. 2) G04 자세 20개에서 영점 오차(J1 ±3°, J2~J4 ±5° 균등)에 따른 TCP 위치·카메라 광축 200 mm 점의 이동량(95%)과 접근 합격선 48 mm 비교."""
import sys, json
import numpy as np
src = open("scripts/g01_pose_generator.py", encoding="utf-8").read().split("# 후보 격자")[0]
sys.argv = ["x"]; ns = {"__file__": "scripts/g01_pose_generator.py"}; exec(compile(src, "pg", "exec"), ns); fk = ns["fk"]
P = [p["sdk_deg"] for f in ("s4_4_poses", "s4_5_poses", "s4_11_poses") for p in json.load(open("evidence/G02/%s.json" % f))["poses"]]
P = np.array(P, float)
def rel(q):   # 카메라 프레임에서 본 TCP 위치(mm)
    f = fk(q[None, :]); dp, dR = f["dabai"]; tp = f["tcp"][0]; return (dR[0].T @ (tp[0] - dp[0])) * 1000
base = np.array([rel(q) for q in P])
rng = np.random.default_rng(0); N = 400
var = 0.0
for _ in range(N):
    dz = np.array([rng.uniform(-3, 3), rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-5, 5), 0])
    r = np.array([rel(q + dz) for q in P]); var = max(var, np.abs(r - base).max())
print("1) 카메라→TCP 상대 위치(카메라 프레임, mm): 자세 %d개, J1~J4 영점 오차 ±(3,5,5,5)°에서 최대 변화 %.4f mm" % (len(P), var))
print("   (자세 간 상대 위치 범위: min %s max %s)" % (np.round(base.min(0), 1).tolist(), np.round(base.max(0), 1).tolist()))
tcp0 = np.array([fk(q[None, :])["tcp"][0][0] * 1000 for q in P])
def cam_pt(q, d=200.0):
    f = fk(q[None, :]); dp, dR = f["dabai"]; return (dp[0] + dR[0] @ np.array([0, d / 1000, 0])) * 1000
pt0 = np.array([cam_pt(q) for q in P])
E1 = []; E2 = []
for _ in range(N):
    dz = np.array([rng.uniform(-3, 3), rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-5, 5), 0])
    E1.append(np.linalg.norm(np.array([fk((q + dz)[None, :])["tcp"][0][0] * 1000 for q in P]) - tcp0, axis=1))
    E2.append(np.linalg.norm(np.array([cam_pt(q + dz) for q in P]) - pt0, axis=1))
E1 = np.array(E1); E2 = np.array(E2)
print("2) 영점 오차(J1 ±3°, J2~J4 ±5° 균등) 시 개방 루프 이동량(mm), 자세 %d개 x %d회:" % (len(P), N))
for nm, E in (("TCP 위치", E1), ("카메라 광축 200 mm 점", E2)):
    print("   %s: 중앙값 %.1f, 95%% %.1f, 최대 %.1f | 자세별 95%% 최대 %.1f" % (nm, np.median(E), np.percentile(E, 95), E.max(), np.percentile(E, 95, axis=0).max()))
print("   접근 합격선 48 mm 대비: 광축 점 95%%가 합격선 %s" % ("안" if np.percentile(E2, 95) <= 48 else "밖"))
# 영점 1° 당 민감도(축별)
for j, nm in enumerate(("J1", "J2", "J3", "J4")):
    e = []
    for q in P:
        dz = np.zeros(5); dz[j] = 1.0; e.append(np.linalg.norm(cam_pt(q + dz) - cam_pt(q)))
    print("   영점 1° 오차(%s)당 광축 200 mm 점 이동: 평균 %.1f mm (최소 %.1f, 최대 %.1f)" % (nm, np.mean(e), np.min(e), np.max(e)))
