#!/usr/bin/env python3
"""S4-10 손끝 3자세 + S4-11 점 5자세 동시 맞춤(오프라인). 점→손끝 상대 벡터(영상: 자세 14/304에서 손끝이 점보다 y −2.3, z −9.3 mm)는 TCP 프레임 고정으로 둔다.
모델 A: 점 오프셋(TCP 프레임 3) + 측정틀 y·z 편향. 모델 B: A + J2·J3·J4 영점 오프셋(±5°). J2 이득 1. 사용: python3 scripts/s411_joint_fit.py"""
import sys, json
import numpy as np
from scipy.optimize import least_squares
src = open("scripts/g01_pose_generator.py", encoding="utf-8").read().split("# 후보 격자")[0]
sys.argv = ["x"]; ns = {"__file__": "scripts/g01_pose_generator.py"}; exec(compile(src, "pg", "exec"), ns); fk = ns["fk"]
dd = json.load(open("evidence/G02/s411_dot_read.json"))["poses"]; D10 = json.load(open("evidence/G02/fk_offset_data.json"))
ZP = 11.0
def dotz(k):
    p = dd[k]; return p["z_mm"] - ((p["strip_top_row"] - 841.5) / p["px_per_mm"] if k in ("303b", "304") else 0) - ZP
DOT = {"300": ([90, 90, 10, 10, 90], dd["300"]["y_mm"], dotz("300")), "301": ([90, 97, 10, 10, 90], dd["301"]["y_mm"], dotz("301")), "302": ([90, 104, 10, 10, 90], dd["302"]["y_mm"], dotz("302")),
       "303": ([90, 98, 10, 10, 90], dd["303a"]["y_mm"], dotz("303a")), "304": ([85, 90, 10, 10, 90], dd["304"]["y_mm"], dotz("304"))}
Q10 = {k: v for k, v in D10["commanded_sdk_j1_j5"].items() if k in ("14", "0", "5")}
TIP = {k: (Q10[k], D10["y_mm"][k], D10["height_plate_mm"]["video_s410_h_strip_25.5"][k]) for k in Q10}
def tcp(q, dj):
    qq = np.array(q, float); qq[1] += dj[0]; qq[2] += dj[1]; qq[3] += dj[2]; f = fk(qq[None, :]); return f["tcp"][0][0] * 1000, f["tcp"][1][0]
p14, R14 = tcp([85, 90, 10, 10, 90], (0, 0, 0)); DELTA = R14.T @ np.array([0.0, -2.3, -9.3])   # 점→손끝 벡터(TCP 프레임)
WD, WT = 2.0, 3.0   # 점 y·z 허용 2 mm, 손끝 3 mm
def resid(t, model, skip=None, j2_5=None):
    od = t[:3]; by, bz = t[3], t[4]; dj = t[5:8] if model == "B" else (0, 0, 0); r = []
    for k, (q, y, z) in DOT.items():
        if k == skip: continue
        p, R = tcp(q, dj); lp = p + R @ od; r += [(lp[1] + by - y) / (WD if k != "304" else 3.0), (lp[2] + bz - z) / (WD if k != "304" else 3.0)]
    for k, (q, y, z) in TIP.items():
        if "t" + k == skip: continue
        q = list(q); 
        if j2_5 and k == "5": q[1] += j2_5
        p, R = tcp(q, dj); lp = p + R @ (od + DELTA); r += [(lp[1] + by - y) / WT, (lp[2] + bz - z) / WT]
    return np.array(r)
def fit(model, skip=None, by_lim=10.0, j2_5=None):
    n = 8 if model == "B" else 5
    lo = np.array([-50, -50, -50, -by_lim, -3] + ([-5, -5, -5] if model == "B" else [])); hi = -lo
    best = None
    for s in range(15):
        t0 = np.clip(np.random.default_rng(s).normal(0, 4, n), lo, hi)
        r = least_squares(lambda t: resid(t, model, skip, j2_5), t0, bounds=(lo, hi))
        if best is None or r.cost < best.cost: best = r
    return best, lo
names = ["od_x", "od_y", "od_z", "y편향", "z편향", "dJ2", "dJ3", "dJ4"]
for model in ("A", "B"):
    for byl in (4.0, 10.0):
        r, lo = fit(model, by_lim=byl); n = len(r.x); res = r.fun
        touch = [names[i] for i in range(n) if abs(abs(r.x[i]) - abs(lo[i])) < 0.05]
        print("\n모델 %s, y 편향 한도 ±%.0f | 매개변수 %s | RMS(정규화) %.2f 최대 %.2f | 한도 도달 %s" % (model, byl, {names[i]: round(float(r.x[i]), 2) for i in range(n)}, np.sqrt(np.mean(res ** 2)), np.abs(res).max(), touch or "없음"))
        J = r.jac; sv = np.linalg.svd(J, compute_uv=False); print("   조건수 %.1f, 자료 %d개 매개변수 %d개" % (sv[0] / sv[-1], len(res), n))
        if byl == 10.0:
            for skip in ("t14", "t0", "t5", "300", "302", "304"):
                r2, _ = fit(model, skip=skip, by_lim=byl); print("   LOO(%s 제외): y편향 %.2f z편향 %.2f %s" % (skip, r2.x[3], r2.x[4], "영점 %s" % np.round(r2.x[5:], 2).tolist() if model == "B" else ""))
            r3, _ = fit(model, by_lim=byl, j2_5=-1.0); print("   민감도(자세 5 손끝 J2 읽기 −1°): y편향 %.2f z편향 %.2f RMS %.2f %s" % (r3.x[3], r3.x[4], np.sqrt(np.mean(r3.fun ** 2)), "영점 %s" % np.round(r3.x[5:], 2).tolist() if model == "B" else ""))
