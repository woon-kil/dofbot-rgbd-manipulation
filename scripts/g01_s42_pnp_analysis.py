#!/usr/bin/env python3
"""S4-2 12자세 solvePnP 분석(오프라인, 하드웨어 없음). ① 자세별 재투영 오차 ② URDF FK(회독값)로 보드 위치를 베이스 좌표로 올렸을 때 12자세 일관성(보드는 고정이므로 같아야 함)
③ (타당성 확인만, 보정 아님) 카메라 외부 파라미터 6자유도 보정으로 일관성이 얼마나 줄어드는지. K는 드라이버 camera_info가 비어 근사값(UNVERIFIED).
사용: python3 g01_s42_pnp_analysis.py <result2a.json> <view2a> <result2b.json> <view2b> <out.json>"""
import json, sys, importlib.util
from pathlib import Path
import cv2, numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
ARGV = list(sys.argv)   # g01_s3_path 불러오기가 sys.argv를 덮어쓴다
sys.path.insert(0, "scripts")
spec = importlib.util.spec_from_file_location("s3", "scripts/g01_s3_path.py"); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
K = np.array([[479.07, 0, 315.07], [0, 478.98, 237.41], [0, 0, 1.0]]); D = np.array(json.load(open("evidence/G02/dabai_color_camera_info.json"))["D"])   # 드라이버 camera_info 왜곡(rational_polynomial 8계수)
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50); board = cv2.aruco.CharucoBoard((6, 5), 0.012, 0.009, dic, np.arange(30, 45, dtype=np.int32)); det = cv2.aruco.CharucoDetector(board)
R_LC = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0.0]])    # 카메라(OpenCV: x우 y아래 z앞) -> 링크 좌표: p_link = (xc, zc, -yc)
rows = []
for res_f, view_d in ((ARGV[1], ARGV[2]), (ARGV[3], ARGV[4])):
    r = json.load(open(res_f))
    for p in r["poses"]:
        pid = p["pose_id"]; best = None
        for f in sorted((Path(view_d) / ("p%d" % pid)).glob("exp_*.png")):
            g = cv2.cvtColor(cv2.imread(str(f)), cv2.COLOR_BGR2GRAY); cc, ci, _, _ = det.detectBoard(g)
            n = 0 if ci is None else len(ci)
            if best is None or n > best[0]: best = (n, f.stem, cc, ci)
        n, ex, cc, ci = best
        obj, img = board.matchImagePoints(cc, ci); obj = obj.reshape(-1, 3); img = img.reshape(-1, 2)
        ok, rv, tv = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
        proj, _ = cv2.projectPoints(obj, rv, tv, K, D); err = np.linalg.norm(proj.reshape(-1, 2) - img, axis=1)
        rb = p["readback"]; q = np.array([[rb["1"], rb["2"], rb["3"], rb["4"], 90.0]])
        dp, dR = m.fk(q)["dabai"]; rows.append({"pose": pid, "exp": ex, "n_corners": int(n), "rvec": rv.ravel(), "tvec": tv.ravel(), "reproj_mean": float(err.mean()), "reproj_max": float(err.max()), "dp": dp[0], "dR": dR[0], "q": q[0]})
def board_in_base(row, corr=None):
    """corr: 6 (회전벡터 3 + 이동 3 m) = 링크 좌표계에서 카메라 위치 보정"""
    Rcb = cv2.Rodrigues(row["rvec"])[0]; tcb = row["tvec"] / 1000.0 if False else row["tvec"]   # tvec 단위: 보드 정의가 m이므로 m
    Rc = row["dR"] @ R_LC; tc = row["dp"].copy()
    if corr is not None: Rd = Rot.from_rotvec(corr[:3]).as_matrix(); Rc = row["dR"] @ Rd @ R_LC; tc = row["dp"] + row["dR"] @ corr[3:]
    return Rc @ Rcb, Rc @ tcb + tc
def stats(corr=None):
    P = np.array([board_in_base(r, corr)[1] for r in rows]); N = np.array([board_in_base(r, corr)[0][:, 2] for r in rows])
    ang = np.degrees(np.arccos(np.clip(np.abs(N[:, 2]), -1, 1)))
    Nm = N.mean(0); Nm /= np.linalg.norm(Nm); spread = np.degrees(np.arccos(np.clip(N @ Nm, -1, 1)))
    return P, ang, spread
P0, ang0, sp0 = stats()
c = P0.mean(0); pos_dev0 = np.linalg.norm(P0 - c, axis=1) * 1000
def resid(x):
    P, ang, sp = stats(x); return np.concatenate([((P - P.mean(0)) * 1000).ravel() / 1.0, np.radians(sp) * 100.0])
sol = least_squares(resid, np.zeros(6), x_scale=[0.02] * 3 + [0.005] * 3)
P1, ang1, sp1 = stats(sol.x); pos_dev1 = np.linalg.norm(P1 - P1.mean(0), axis=1) * 1000
out = {"K_approx": K.tolist(), "n_poses": len(rows), "per_pose": [{"pose": r["pose"], "exp": r["exp"], "corners": r["n_corners"], "reproj_mean_px": round(r["reproj_mean"], 3), "reproj_max_px": round(r["reproj_max"], 3), "board_dist_mm": round(float(np.linalg.norm(r["tvec"]) * 1000), 1),
    "board_base_xyz_mm": [round(float(v), 1) for v in (P0[i] * 1000)], "normal_tilt_from_vertical_deg": round(float(ang0[i]), 2), "pos_dev_from_mean_mm": round(float(pos_dev0[i]), 1)} for i, r in enumerate(rows)],
    "nominal": {"board_origin_mean_base_mm": [round(float(v), 1) for v in c * 1000], "pos_dev_mean_mm": round(float(pos_dev0.mean()), 1), "pos_dev_max_mm": round(float(pos_dev0.max()), 1), "normal_spread_mean_deg": round(float(sp0.mean()), 2), "normal_spread_max_deg": round(float(sp0.max()), 2), "normal_tilt_mean_deg": round(float(ang0.mean()), 2)},
    "feasibility_extrinsic_correction": {"note": "타당성 확인만(보정 결과 아님, 외부 파라미터 6자유도 최소제곱, 과적합 가능)", "rotvec_deg": [round(float(np.degrees(v)), 2) for v in sol.x[:3]], "trans_mm": [round(float(v) * 1000, 1) for v in sol.x[3:]], "pos_dev_mean_mm": round(float(pos_dev1.mean()), 1), "pos_dev_max_mm": round(float(pos_dev1.max()), 1), "normal_spread_mean_deg": round(float(sp1.mean()), 2), "normal_spread_max_deg": round(float(sp1.max()), 2), "normal_tilt_mean_deg": round(float(ang1.mean()), 2)}}
Path(ARGV[5]).write_text(json.dumps(out, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
print("자세별: 재투영(평균/최대 px), 보드 거리(mm), 보드 베이스 xyz(mm), 법선 기울기(°), 평균 대비 편차(mm)")
for d in out["per_pose"]: print(" %3d 노출%s 코너%2d | %.2f/%.2f | %5.1f | %s | %.2f | %.1f" % (d["pose"], d["exp"][4:], d["corners"], d["reproj_mean_px"], d["reproj_max_px"], d["board_dist_mm"], d["board_base_xyz_mm"], d["normal_tilt_from_vertical_deg"], d["pos_dev_from_mean_mm"]))
print("공칭(URDF) 일관성:", out["nominal"]); print("외부 6자유도 보정 타당성:", out["feasibility_extrinsic_correction"])
