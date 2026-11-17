#!/usr/bin/env python3
"""항 b 비접촉 추정 2단계(오프라인): S4-11 자세 300~304의 PnP(카메라→보드)와 폰 영상 점 위치(보드 좌표)로 점을 카메라 프레임에 올리고, FK+동결 hand-eye가 예측하는 TCP 위치와 비교한다."""
import sys, json, itertools
import cv2, numpy as np
from scipy.spatial.transform import Rotation as Rot
src = open("scripts/g01_pose_generator.py", encoding="utf-8").read().split("# 후보 격자")[0]
sys.argv = ["x"]; ns = {"__file__": "scripts/g01_pose_generator.py"}; exec(compile(src, "pg", "exec"), ns); fk = ns["fk"]
cap = json.load(open("evidence/G02/s412_capture_check.json")); dot = json.load(open("evidence/G02/s411_dot_read.json"))["poses"]
HE = json.load(open("evidence/G02/g04_solve.json"))["fit"]["PARK"]; Rd = Rot.from_euler("xyz", HE["rot_deg"], degrees=True).as_matrix() if False else Rot.from_rotvec(np.radians(HE["rot_deg"])).as_matrix(); tcorr = np.array(HE["trans_mm"]) / 1000.0
R_LC = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0.0]])
Q = {"300": [89, 90, 10, 11], "301": [89, 97, 10, 11], "302": [89, 104, 10, 11], "303": [89, 98, 10, 11], "304": [85, 90, 10, 11]}
ZP = 11.0
def dotz(k):
    p = dot[k]; return p["z_mm"] + 2.0 - ((p["strip_top_row"] - 841.5) / p["px_per_mm"] if k in ("303b", "304") else 0) - ZP
LM = {"300": (dot["300"]["y_mm"], dotz("300")), "301": (dot["301"]["y_mm"], dotz("301")), "302": (dot["302"]["y_mm"], dotz("302")), "303": (dot["303a"]["y_mm"], dotz("303a")), "304": (dot["304"]["y_mm"], dotz("304"))}
rows = {}
for k in Q:
    c = cap["261113-233603/p%s" % k]; t = np.array(c["tvec_mm"]) / 1000.0
    # rvec 필요: 재계산
rows = {}
import importlib
K = np.array([[479.07, 0, 315.07], [0, 478.98, 237.41], [0, 0, 1.0]]); D = np.array(json.load(open("evidence/G02/dabai_color_camera_info.json"))["D"])
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50); board = cv2.aruco.CharucoBoard((6, 5), 0.012, 0.009, dic, np.arange(30, 45, dtype=np.int32)); det = cv2.aruco.CharucoDetector(board)
for k in Q:
    c = cap["261113-233603/p%s" % k]; g = cv2.cvtColor(cv2.imread("evidence/G01/raw/s42/261113-233603/g01_s42_view/p%s/%s" % (k, c["file"])), cv2.COLOR_BGR2GRAY)
    cc, ci, _, _ = det.detectBoard(g); obj, img = board.matchImagePoints(cc, ci); ok, rv, tv = cv2.solvePnP(obj.reshape(-1, 3), img.reshape(-1, 2), K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    rows[k] = (cv2.Rodrigues(rv)[0], tv.ravel())
# 보드→세계 회전 후보 24개 중 카메라 위치(PnP 대 FK+hand-eye)가 가장 일치하는 것
def cam_fk(q):
    f = fk(np.array([q + [90.0]])); dp, dR = f["dabai"]; return dp[0] + dR[0] @ tcorr, dR[0] @ Rd @ R_LC, f
O_B = np.array([0.036, 0.030, 0.0]); W0 = np.array([0.0, 0.2275, -0.0102])   # 보드 중심(보드 좌표) → 세계 (y 229 ±2: 풀이는 227.49), z = 인쇄면 판 윗면 기준 −10.2 mm
best = None
for perm in itertools.permutations(range(3)):
    for sg in itertools.product((1, -1), repeat=3):
        Rbw = np.zeros((3, 3))
        for i, j in enumerate(perm): Rbw[j, i] = sg[i]
        if np.linalg.det(Rbw) < 0: continue
        err = []
        for k in Q:
            R, t = rows[k]; Cb = -R.T @ t; cw = Rbw @ (Cb - O_B) + W0; err.append(np.linalg.norm(cw - cam_fk(Q[k])[0]))
        s = np.mean(err)
        if best is None or s < best[0]: best = (s, Rbw, err)
print("보드→세계 회전 선택: 카메라 위치 불일치 평균 %.1f mm, 자세별 %s" % (best[0] * 1000, np.round(np.array(best[2]) * 1000, 1).tolist())); print(best[1])
Rbw = best[1]; out = {}
for k in Q:
    R, t = rows[k]; _, Rc, f = cam_fk(Q[k]); tc, _, _ = cam_fk(Q[k])
    y, z = LM[k]; xs = -6.0   # 점 x(폰 쪽 패드 바깥면, 눈대중 ±5)
    pw = np.array([xs, y, z]) / 1000.0; pb = Rbw.T @ (pw - W0) + O_B; pc_meas = R @ pb + t       # PnP 기반 점(카메라 프레임)
    tcpw = f["tcp"][0][0]; pc_pred = (Rd @ R_LC).T @ (f["dabai"][1][0].T @ (tcpw - f["dabai"][0][0]) - tcorr)   # FK+hand-eye 예측 TCP(카메라 프레임)
    out[k] = {"meas_cam_mm": np.round(pc_meas * 1000, 1).tolist(), "tcp_pred_cam_mm": np.round(pc_pred * 1000, 1).tolist(), "diff_mm": np.round((pc_meas - pc_pred) * 1000, 1).tolist()}
    print(k, out[k])
d = np.array([out[k]["diff_mm"] for k in Q]); print("차이(점 − TCP 예측, 카메라 프레임 x우·y아래·z앞) 평균 %s 표준편차 %s" % (np.round(d.mean(0), 1).tolist(), np.round(d.std(0), 1).tolist()))
json.dump(out, open("/tmp/s410/s412_step2_plus2.json", "w"), ensure_ascii=False, indent=1)
dv = []
for k in Q:
    R, t = rows[k]; Cb = -R.T @ t; cw = Rbw @ (Cb - O_B) + W0; dv.append((cw - cam_fk(Q[k])[0]) * 1000)
dv = np.array(dv); print("PnP 세계 카메라 위치 − FK+hand-eye 카메라 위치 (x,y,z mm) 평균 %s 표준편차 %s" % (np.round(dv.mean(0), 1).tolist(), np.round(dv.std(0), 1).tolist()))
