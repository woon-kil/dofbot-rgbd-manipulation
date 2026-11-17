#!/usr/bin/env python3
"""S4-11 점 회전 + FK 일관성(오프라인): 자세 300·301·302·303의 점 (y, z)를 'TCP 프레임 고정 오프셋 3 + 측정틀 y·z 편향 2'로 맞추고(J2 이득 1 고정 / 자유) 잔차를 본다. 하드웨어 없음.
사용: python3 scripts/s411_fk_consistency.py"""
import sys, json
import numpy as np
from scipy.optimize import least_squares
src = open("scripts/g01_pose_generator.py", encoding="utf-8").read().split("# 후보 격자")[0]
sys.argv = ["x"]; ns = {"__file__": "scripts/g01_pose_generator.py"}; exec(compile(src, "pg", "exec"), ns); fk = ns["fk"]
d = json.load(open("evidence/G02/s411_dot_read.json"))["poses"]
Zp = 11.0   # 판 윗면 = 종이 +11 (사용자 확인)
def meas(k):
    p = d[k]; z = p["z_mm"] - ((p["strip_top_row"] - 841.5) / p["px_per_mm"] if k in ("303b", "304") else 0)
    return p["y_mm"], z - Zp
Q = {"300": [90, 90, 10, 10, 90], "301": [90, 98, 10, 10, 90], "302": [90, 105, 10, 10, 90], "303a": [90, 98, 10, 10, 90]}
J2READ = {"300": 90, "301": 97, "302": 104, "303a": 98}
def pred(k, th, g, use_read):
    q = np.array(Q[k], float); q[1] = 90 + g * ((J2READ[k] if use_read else q[1]) - 90)
    f = fk(q[None, :]); p = f["tcp"][0][0] * 1000; R = f["tcp"][1][0]; lp = p + R @ th[:3]; return lp[1] + th[3], lp[2] + th[4]
def run(keys, gfree, use_read):
    def res(t):
        g = t[5] if gfree else 1.0; r = []
        for k in keys:
            y, z = meas(k); py, pz = pred(k, t[:5], g, use_read); r += [py - y, pz - z]
        return np.array(r)
    best = None
    for s in range(12):
        t0 = np.random.default_rng(s).normal(0, 10, 6 if gfree else 5)
        if gfree: t0[5] = 1.0
        r = least_squares(res, t0)
        if best is None or r.cost < best.cost: best = r
    return best
for keys in (("300", "301", "302"), ("300", "301", "302", "303a")):
    for use_read in (False, True):
        for gfree in (False, True):
            if gfree and len(keys) < 4: continue
            r = run(keys, gfree, use_read); rr = np.round(r.fun.reshape(-1, 2), 2).tolist()
            print("자세 %s | J2 %s | g %s | 매개변수(TCP 오프셋 3, y·z 편향 %s) %s | 잔차(y,z) %s | 최대 %.2f mm" % (list(keys), "읽은값" if use_read else "명령", "자유" if gfree else "1 고정", "", np.round(r.x, 2).tolist(), rr, np.abs(r.fun).max()))
# FK가 예측하는 300→302 높이·y 변화(오프셋 무관량이 아님: 오프셋은 TCP 프레임 고정이라 같이 회전)
for k in ("300", "301", "302"):
    q = np.array(Q[k], float); f = fk(q[None, :]); p = f["tcp"][0][0] * 1000; print(k, "TCP(y,z)", np.round(p[1:], 1).tolist(), "측정 점(y,z_plate)", np.round(meas(k), 1).tolist())
