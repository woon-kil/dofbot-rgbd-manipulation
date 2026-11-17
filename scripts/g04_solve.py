#!/usr/bin/env python3
"""G04 풀이(동결 v1.2 M1): 학습 = 파일럿 12 + S4-4 10자세(학습 8 + 점검 2, 점검은 정상 자세로 사용). holdout·S4-3 점검 자세·확증 자세는 쓰지 않는다.
① cv2.calibrateHandEye 4방법 비교 ② 외부 6자유도 + J1 영점 오프셋(7번째) 비선형 적합(보드 xy 실측 prior σ1.5 mm, z 자유), 초기값 공칭·PARK 두 가지.
사용: python3 g04_solve.py <out.json>"""
import json, sys, importlib.util
from pathlib import Path
import numpy as np, cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
ARGV = list(sys.argv); sys.path.insert(0, "scripts")
spec = importlib.util.spec_from_file_location("s3", "scripts/g01_s3_path.py"); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
ci = json.load(open("evidence/G02/dabai_color_camera_info.json")); K = np.array(ci["K"]).reshape(3, 3); D = np.array(ci["D"]); R_LC = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0.0]])
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50); board = cv2.aruco.CharucoBoard((6, 5), 0.012, 0.009, dic, np.arange(30, 45, dtype=np.int32)); det = cv2.aruco.CharucoDetector(board)
meas = json.load(open("evidence/G02/board_measurement.json"))["derived_base_frame"]["board_center_xy_mm"]; BXY = np.array(meas) / 1000.0
RES = "evidence/G01/raw/"
def measure(res_f, view_d):
    out = []
    for p in json.load(open(res_f))["poses"]:
        best = None
        for f in sorted((Path(view_d) / ("p%d" % p["pose_id"])).glob("exp_*.png")):
            g = cv2.cvtColor(cv2.imread(str(f)), cv2.COLOR_BGR2GRAY); cc, cid, _, _ = det.detectBoard(g); n = 0 if cid is None else len(cid)
            if best is None or n > best[0]: best = (n, cc, cid)
        obj, img = board.matchImagePoints(best[1], best[2]); ok, rv, tv = cv2.solvePnP(obj.reshape(-1, 3), img.reshape(-1, 2), K, D)
        rb = p["readback"]; out.append({"pose_id": p["pose_id"], "q": [rb["1"], rb["2"], rb["3"], rb["4"]], "rvec": rv.ravel(), "tvec": tv.ravel()})
    return out
data = []
for rf, vd in ((RES + "g01_5b4_s422a_result-real-261112-213337.json", RES + "s42/261112-213337/g01_s42_view"), (RES + "g01_5b4_s422b_result-real-261112-215835.json", RES + "s42/261112-215835/g01_s42_view"), (RES + "g01_5b4_s424_result-real-261113-015617.json", RES + "s42/261113-015617/g01_s42_view")):
    data += measure(rf, vd)
print("학습 측정 수:", len(data))
# ① cv2.calibrateHandEye 4방법
Rg, tg, Rt, tt = [], [], [], []
for d in data:
    dp, dR = m.fk(np.array([[*d["q"], 90.0]]))["dabai"]; Rg.append(dR[0]); tg.append(dp[0].reshape(3, 1)); Rt.append(cv2.Rodrigues(d["rvec"])[0]); tt.append(d["tvec"].reshape(3, 1))
methods = {"PARK": cv2.CALIB_HAND_EYE_PARK, "TSAI": cv2.CALIB_HAND_EYE_TSAI, "HORAUD": cv2.CALIB_HAND_EYE_HORAUD, "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS}
sol_he = {}
for k, v in methods.items():
    Rcg, tcg = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=v); Rd = Rcg @ R_LC.T; sol_he[k] = {"rot_deg": np.degrees(Rot.from_matrix(Rd).as_rotvec()).round(2).tolist(), "trans_mm": (tcg.ravel() * 1000).round(1).tolist()}
    print("calibrateHandEye", k, sol_he[k])
def pair_diff(a, b):
    ra = Rot.from_rotvec(np.radians(a["rot_deg"])); rb = Rot.from_rotvec(np.radians(b["rot_deg"])); return float(np.degrees((ra.inv() * rb).magnitude())), float(np.linalg.norm(np.array(a["trans_mm"]) - np.array(b["trans_mm"])))
names = list(sol_he); maxrot = maxtr = 0.0
for i in range(4):
    for j in range(i + 1, 4):
        r, t = pair_diff(sol_he[names[i]], sol_he[names[j]]); maxrot = max(maxrot, r); maxtr = max(maxtr, t)
print("방법 간 최대 불일치: 회전 %.2f°, 이동 %.1f mm (불안정 기준 1°, 3 mm)" % (maxrot, maxtr))
# ② 7자유도 비선형 적합
Q = np.array([d["q"] for d in data], float)
def centers_normals(x):
    c, o1 = x[:6], x[6]; qq = Q.copy(); qq[:, 0] += o1; f = m.fk(np.hstack([qq, np.full((len(qq), 1), 90.0)])); dp, dR = f["dabai"]; Rd = Rot.from_rotvec(c[:3]).as_matrix()
    Pc, Nn = [], []
    for i, d in enumerate(data):
        Rc = dR[i] @ Rd @ R_LC; tc = dp[i] + dR[i] @ c[3:]; Rb = cv2.Rodrigues(d["rvec"])[0]
        Pc.append(Rc @ (Rb @ np.array([0.036, 0.030, 0.0]) + d["tvec"]) + tc); Nn.append((Rc @ Rb)[:, 2])
    return np.array(Pc), np.array(Nn)
S_POS, S_NRM, S_PRI = 2.5, 0.4, 1.5   # mm, deg, mm (관측 잡음 수준)
def resid(x):
    P, N = centers_normals(x); b = np.array([BXY[0] * 1000, BXY[1] * 1000, x[9]]) if False else np.array([x[7], x[8], x[9]]); Nm = N.mean(0); Nm /= np.linalg.norm(Nm)
    return np.concatenate([((P * 1000 - b) / S_POS).ravel(), np.degrees(np.arccos(np.clip(N @ Nm, -1, 1))) / S_NRM, (np.array([x[7], x[8]]) - BXY * 1000) / S_PRI])
# 초기값: 공칭 / PARK
def x0_from(Rd_rotvec, t_m, zb): return np.concatenate([Rd_rotvec, t_m, [0.0], [BXY[0] * 1000, BXY[1] * 1000, zb]])
P0, _ = centers_normals(np.zeros(7)); zb0 = float(P0[:, 2].mean() * 1000)
inits = {"공칭": x0_from(np.zeros(3), np.zeros(3), zb0), "PARK": x0_from(np.radians(sol_he["PARK"]["rot_deg"]), np.array(sol_he["PARK"]["trans_mm"]) / 1000, zb0)}
scale = np.array([0.02] * 3 + [0.005] * 3 + [1.0] + [1.0, 1.0, 5.0])   # 회전 rad, 이동 m, J1 영점 deg, b mm
results = {}
for k, x0 in inits.items():
    s = least_squares(resid, x0, x_scale=scale); r = resid(s.x); J = s.jac; cov = np.linalg.pinv(J.T @ J); chi2 = float(np.sum(r ** 2) / max(len(r) - len(s.x), 1)); sd = np.sqrt(np.diag(cov) * max(chi2, 1.0))
    results[k] = {"x": s.x, "chi2_red": round(chi2, 2), "cost": round(float(s.cost), 2), "sd": sd}
    print(k, "초기 → 회전(°)", np.degrees(s.x[:3]).round(2).tolist(), "이동(mm)", (s.x[3:6] * 1000).round(1).tolist(), "J1 영점(°) %.2f" % s.x[6], "보드(mm)", s.x[7:10].round(1).tolist(), "chi2_red %.2f" % chi2)
    print("   σ: 회전(°)", np.degrees(sd[:3]).round(2).tolist(), "이동(mm)", (sd[3:6] * 1000).round(1).tolist(), "J1 영점(°) %.2f" % sd[6], "보드(mm)", sd[7:10].round(1).tolist())
for k, v in results.items():
    P, N = centers_normals(v["x"]); b = v["x"][7:10]; dpos = np.linalg.norm(P * 1000 - b, axis=1); Nm = N.mean(0); Nm /= np.linalg.norm(Nm); ang = np.degrees(np.arccos(np.clip(N @ Nm, -1, 1)))
    v["resid"] = {"pos_mean_mm": round(float(dpos.mean()), 2), "pos_max_mm": round(float(dpos.max()), 2), "normal_mean_deg": round(float(ang.mean()), 2), "normal_max_deg": round(float(ang.max()), 2)}
    print(k, "적합 후 잔차(보드 위치 편차 평균/최대 mm, 법선 평균/최대 °):", v["resid"])
P0n, _ = centers_normals(np.zeros(7)); print("보정 전(공칭) 보드 중심 평균(mm):", (P0n.mean(0) * 1000).round(1).tolist(), "→ 측정 xy", (BXY * 1000).tolist())
xa, xb = results["공칭"]["x"], results["PARK"]["x"]
print("두 초기값 해 차이: 회전 %.2f°, 이동 %.1f mm, J1 영점 %.2f°" % (np.degrees((Rot.from_rotvec(xa[:3]).inv() * Rot.from_rotvec(xb[:3])).magnitude()), np.linalg.norm(xa[3:6] - xb[3:6]) * 1000, abs(xa[6] - xb[6])))
res_o = {"n_train": len(data), "calibrateHandEye": sol_he, "method_max_disagreement": {"rot_deg": round(maxrot, 2), "trans_mm": round(maxtr, 1)}, "priors_mm_deg": {"S_POS": S_POS, "S_NRM": S_NRM, "S_PRI": S_PRI}, "board_xy_prior_mm": (BXY * 1000).tolist(),
         "fit": {k: {"rot_deg": np.degrees(v["x"][:3]).round(3).tolist(), "trans_mm": (v["x"][3:6] * 1000).round(2).tolist(), "j1_zero_deg": round(float(v["x"][6]), 3), "board_xyz_mm": v["x"][7:10].round(2).tolist(), "chi2_red": v["chi2_red"], "resid": v["resid"],
                       "sigma": {"rot_deg": np.degrees(v["sd"][:3]).round(3).tolist(), "trans_mm": (v["sd"][3:6] * 1000).round(2).tolist(), "j1_zero_deg": round(float(v["sd"][6]), 3), "board_mm": v["sd"][7:10].round(2).tolist()}} for k, v in results.items()}}
Path(ARGV[1]).write_text(json.dumps(res_o, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
