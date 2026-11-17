#!/usr/bin/env python3
"""S4-12 영상 판독: 폰(IMG_0429, 대각선)의 보드 PnP와 점 픽셀로 광선을 만들고, S4-11에서 잰 높이(+2 mm)·y로 깊이를 고정해 점의 세계 x를 구한다. 로봇 명령 없음.
사용: python3 scripts/s412_x_read.py FRAMES_DIR OUT.json"""
import sys, json, glob, os
import cv2, numpy as np
D, OUT = sys.argv[1], sys.argv[2]
dd = json.load(open("evidence/G02/s411_dot_read.json"))["poses"]
WIN = {"300": (86, 100), "301": (110, 125), "302": (135, 153), "303": (160, 175), "304": (185, 205)}
S411 = {"300": "300", "301": "301", "302": "302", "303": "303a", "304": "304"}
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50); board = cv2.aruco.CharucoBoard((6, 5), 0.012, 0.009, dic, np.arange(30, 45, dtype=np.int32)); det = cv2.aruco.CharucoDetector(board)
def frame(i): return cv2.imread(os.path.join(D, "%04d.png" % (i + 1)))
def dot(im):
    b, g, r = [im[..., k].astype(np.int32) for k in range(3)]
    m = ((r > 90) & (r > 1.6 * g) & (r > 1.6 * b)); m[:, :700] = False; m[800:, :] = False; m[:, 1500:] = False
    m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)); k, lab, st, cen = cv2.connectedComponentsWithStats(m)
    c = [(int(st[j, 4]), float(cen[j][0]), float(cen[j][1])) for j in range(1, k) if st[j, 4] > 40]
    return max(c) if c else None
pts = {}; pnp = []
for k, (a, b) in WIN.items():
    pp = []
    for i in range(a, b + 1):
        im = frame(i); d = dot(im)
        if d is not None: pp.append(d)
    P = np.array(pp); pts[k] = {"n": len(P), "u": float(np.median(P[:, 1])), "v": float(np.median(P[:, 2])), "area": float(np.median(P[:, 0])), "u_std": float(P[:, 1].std()), "v_std": float(P[:, 2].std())}
g0 = cv2.cvtColor(frame(20), cv2.COLOR_BGR2GRAY); cc, ci, _, _ = det.detectBoard(g0); obj, img = board.matchImagePoints(cc, ci); obj = obj.reshape(-1, 3); img = img.reshape(-1, 2)
res = {"windows": WIN, "dot_pixels": pts, "board_corners": int(len(obj))}
for fov in (60, 63, 66):
    f = 1920 / (2 * np.tan(np.radians(fov / 2))); K = np.array([[f, 0, 960], [0, f, 540], [0, 0, 1.0]]); ok, rv, tv = cv2.solvePnP(obj, img, K, None); R = cv2.Rodrigues(rv)[0]; C = -R.T @ tv.ravel()
    def b2w(b): b = np.array(b) * 1000; return np.array([b[0] - 36, b[1] + 199, 0.8 - b[2]])
    cw = b2w(C); out = {}
    for k, p in pts.items():
        d = R.T @ np.linalg.inv(K) @ np.array([p["u"], p["v"], 1.0]); dw = np.array([d[0], d[1], -d[2]])
        s = dd[S411[k]]; zt = s["z_mm"] + 2.0 - ((s["strip_top_row"] - 841.5) / s["px_per_mm"] if k == "304" else 0); yt = s["y_mm"]
        tz = (zt - cw[2]) / dw[2]; pz = cw + tz * dw; ty = (yt - cw[1]) / dw[1]; py = cw + ty * dw
        out[k] = {"z_plane": {"given_z": round(zt, 1), "x": round(float(pz[0]), 1), "y": round(float(pz[1]), 1)}, "y_plane": {"given_y": round(yt, 1), "x": round(float(py[0]), 1), "z": round(float(py[2]), 1)}}
    res["fov%d" % fov] = {"camera_world": np.round(cw, 1).tolist(), "points": out}
json.dump(res, open(OUT, "w"), ensure_ascii=False, indent=1)
print("점 픽셀:", {k: (round(v["u"], 1), round(v["v"], 1), v["n"], int(v["area"])) for k, v in pts.items()})
for fov in (60, 63, 66):
    r = res["fov%d" % fov]; print("FOV", fov, "카메라", r["camera_world"]); 
    for k, o in r["points"].items(): print("  ", k, "z평면: x %.1f y %.1f (z 고정 %.1f) | y평면: x %.1f z %.1f (y 고정 %.1f)" % (o["z_plane"]["x"], o["z_plane"]["y"], o["z_plane"]["given_z"], o["y_plane"]["x"], o["y_plane"]["z"], o["y_plane"]["given_y"]))
