#!/usr/bin/env python3
"""S4-11 영상 판독: 폰 영상의 자세별 정지 구간에서 빨간 점 중심을 눈금 띠 면 좌표(y mm, 종이 위 높이 z mm)로 읽고,
세 점(자세 300·301·302)으로 원을 맞춰 회전 중심·각도·J2 이득 g = 회전각/명령각을 구한다. 로봇 명령 없음.
사용: python3 scripts/s411_dot_read.py FRAMES_DIR OUT.json  (FRAMES_DIR = 영상 1 fps 프레임 0001.png, ...)"""
import sys, json, glob, os
import numpy as np, cv2
from scipy.signal import find_peaks
D, OUT = sys.argv[1], sys.argv[2]
WIN = {"300": (100, 135), "301": (142, 158), "302": (167, 180), "303a": (187, 200), "303b": (205, 220), "304": (232, 260)}   # 영상 초(1 fps 프레임 번호 = 초+1), 점 위치가 일정한 구간
H_STRIP = 25.5
def frame(t): return cv2.imread(os.path.join(D, "%04d.png" % (t + 1)))
def ticks(g):
    prof = g.astype(np.float32)[1056:1078, :].mean(axis=0)
    base = cv2.GaussianBlur(prof.reshape(1, -1), (0, 0), 15).ravel()
    pk, _ = find_peaks(base - prof, height=25, distance=60)
    pk = [int(p) for p in pk if 440 < p < 1530]
    xs = []
    for x in pk:
        w = prof[x - 6:x + 7].max() - prof[x - 6:x + 7]; xs.append(float((np.arange(x - 6, x + 7) * w).sum() / w.sum()))
    out = []   # 격자(110→457 px, 간격 96 px/10 mm)에 가까운 봉우리만 눈금으로 인정(띠 모서리 등 제외)
    for x in xs:
        k = round((x - 457.0) / 96.0)
        if 0 <= k <= 11 and abs(x - (457.0 + 96.0 * k)) < 14: out.append((110 - 10 * k, x))
    return out
def dot(im):
    h = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    m = (((h[..., 0] < 8) | (h[..., 0] > 172)) & (h[..., 1] > 150) & (h[..., 2] > 110)); m[880:, :] = False
    m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    k, lab, st, cen = cv2.connectedComponentsWithStats(m)
    if k < 2: return None
    j = 1 + int(np.argmax(st[1:, 4])); return float(cen[j][0]), float(cen[j][1]), int(st[j, 4])
def strip_top(g):   # 띠 앞면 윗 모서리 행(열 700~1400에서 840~900 구간 세로 기울기 최대 행의 중앙값)
    gb = cv2.GaussianBlur(g, (0, 0), 2).astype(np.float32); rows = []
    for c in range(700, 1400, 25):
        col = gb[830:910, c]; rows.append(830 + int(np.argmax(np.abs(np.diff(col)))))
    return float(np.median(rows))
res = {"windows": WIN, "poses": {}}; calib = []
for name, (a, b) in WIN.items():
    pts = []
    for t in range(a, b + 1):
        im = frame(t); g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY); xs = ticks(g); d = dot(im)
        if len(xs) < 10 or d is None: continue
        A = np.polyfit([x for _, x in xs], [m_ for m_, _ in xs], 1)   # u -> 눈금 m
        px_mm = abs(1.0 / A[0]); st = strip_top(g)
        m = float(np.polyval(A, d[0])); y = 199.0 - m
        z = (st + H_STRIP * px_mm - d[1]) / px_mm              # 종이 위 높이(띠 면 평면, 점 중심)
        pts.append((y, z, d[0], d[1], d[2], px_mm, st))
    P = np.array(pts)
    res["poses"][name] = {"frames": len(P), "y_mm": round(float(np.median(P[:, 0])), 2), "z_mm": round(float(np.median(P[:, 1])), 2),
        "y_std": round(float(P[:, 0].std()), 2), "z_std": round(float(P[:, 1].std()), 2), "u_px": round(float(np.median(P[:, 2])), 1), "v_px": round(float(np.median(P[:, 3])), 1),
        "area_px": int(np.median(P[:, 4])), "px_per_mm": round(float(np.median(P[:, 5])), 3), "strip_top_row": round(float(np.median(P[:, 6])), 1)}
def circle(p):
    A = np.array([[2 * (p[1][0] - p[0][0]), 2 * (p[1][1] - p[0][1])], [2 * (p[2][0] - p[0][0]), 2 * (p[2][1] - p[0][1])]])
    b = np.array([p[1][0] ** 2 - p[0][0] ** 2 + p[1][1] ** 2 - p[0][1] ** 2, p[2][0] ** 2 - p[0][0] ** 2 + p[2][1] ** 2 - p[0][1] ** 2])
    c = np.linalg.solve(A, b); return c
pp = [(res["poses"][k]["y_mm"], res["poses"][k]["z_mm"]) for k in ("300", "301", "302")]
c = circle(pp); R = [float(np.hypot(p[0] - c[0], p[1] - c[1])) for p in pp]
ang = [float(np.degrees(np.arctan2(p[1] - c[1], p[0] - c[0]))) for p in pp]
res["circle_fit_300_301_302"] = {"center_y": round(float(c[0]), 1), "center_z_above_paper": round(float(c[1]), 1), "radii": [round(r, 1) for r in R], "angles_deg": [round(a, 2) for a in ang],
    "d_angle_300_301": round(ang[1] - ang[0], 2), "d_angle_301_302": round(ang[2] - ang[1], 2), "d_angle_300_302": round(ang[2] - ang[0], 2)}
# 모델 피벗(J2 축 y 0, z 122.5 FK 원점 기준. 종이 기준은 +11 가정: UNVERIFIED)
for zp in (122.5, 133.5):
    ang2 = [float(np.degrees(np.arctan2(p[1] - zp, p[0] - 0.0))) for p in pp]
    res["model_pivot_z%.1f" % zp] = {"angles_deg": [round(a, 2) for a in ang2], "d300_301": round(ang2[1] - ang2[0], 2), "d301_302": round(ang2[2] - ang2[1], 2), "d300_302": round(ang2[2] - ang2[0], 2)}
json.dump(res, open(OUT, "w"), ensure_ascii=False, indent=2); print(json.dumps(res, ensure_ascii=False, indent=1))
