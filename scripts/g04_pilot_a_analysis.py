#!/usr/bin/env python3
"""폐루프 파일럿 항 a 분석(탐색, 합격선 없음): 자세별 정지 10프레임의 ① 프레임 중복 여부 ② ChArUco 검출 ③ PnP 위치·자세 흩어짐 ④ 동결 해로 구한 보드 xy 예측의 흩어짐(관측 잡음 하한).
사용: python3 g04_pilot_a_analysis.py <result.json> <view_dir> <out.json>"""
import json, sys, hashlib, importlib.util
from pathlib import Path
import numpy as np, cv2
from scipy.spatial.transform import Rotation as Rot
ARGV = list(sys.argv)
spec = importlib.util.spec_from_file_location("s3", "scripts/g01_s3_path.py"); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
ci = json.load(open("evidence/G02/dabai_color_camera_info.json")); K = np.array(ci["K"]).reshape(3, 3); D = np.array(ci["D"]); R_LC = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0.0]])
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50); board = cv2.aruco.CharucoBoard((6, 5), 0.012, 0.009, dic, np.arange(30, 45, dtype=np.int32)); det = cv2.aruco.CharucoDetector(board)
F = json.load(open("evidence/G02/g04_solve.json"))["fit"]["공칭"]; Rd = Rot.from_rotvec(np.radians(F["rot_deg"])).as_matrix(); tr = np.array(F["trans_mm"]) / 1000; j1z = F["j1_zero_deg"]
res_f, view_d, out_f = ARGV[1:4]; out = {}
for p in json.load(open(res_f))["poses"]:
    pid = p["pose_id"]; d = Path(view_d) / ("p%d" % pid); files = sorted(d.glob("exp_*.png")); info = json.load(open(d / "expsweep.json"))
    hashes = [hashlib.sha256(f.read_bytes()).hexdigest()[:12] for f in files]; imgs = [cv2.imread(str(f)).astype(np.int16) for f in files]
    diffs = [float(np.abs(imgs[i] - imgs[i + 1]).mean()) for i in range(len(imgs) - 1)]
    rb = p["readback"]; q = np.array([[rb["1"] + j1z, rb["2"], rb["3"], rb["4"], 90.0]]); dp, dR = m.fk(q)["dabai"]; Rc = dR[0] @ Rd @ R_LC; tc = dp[0] + dR[0] @ tr
    tv_l, rv_l, xy_l, n_ok = [], [], [], 0
    for f in files:
        g = cv2.cvtColor(cv2.imread(str(f)), cv2.COLOR_BGR2GRAY); cc, cid, mc, mid = det.detectBoard(g); nc = 0 if cid is None else len(cid); nm = 0 if mid is None else len(mid)
        if nc < 10 or nm < 8: continue
        obj, img = board.matchImagePoints(cc, cid); ok, rv, tv = cv2.solvePnP(obj.reshape(-1, 3), img.reshape(-1, 2), K, D); n_ok += 1
        tv_l.append(tv.ravel() * 1000); rv_l.append(np.degrees(rv.ravel())); P = Rc @ (cv2.Rodrigues(rv)[0] @ np.array([0.036, 0.030, 0.0]) + tv.ravel()) + tc; xy_l.append(P[:2] * 1000)
    row = {"readback": [rb["1"], rb["2"], rb["3"], rb["4"]], "n_frames": len(files), "n_unique_hash": len(set(hashes)), "frames_in_settle": [i.get("frames_in_settle") for i in info if "frames_in_settle" in i], "mean_abs_diff_consecutive": [round(x, 2) for x in diffs], "n_detected": n_ok}
    if n_ok >= 3:
        tv_a = np.array(tv_l); rv_a = np.array(rv_l); xy_a = np.array(xy_l)
        row.update({"cam_dist_mm": round(float(np.linalg.norm(tv_a.mean(0))), 1), "tvec_sd_mm": tv_a.std(0, ddof=1).round(3).tolist(), "rvec_sd_deg": rv_a.std(0, ddof=1).round(3).tolist(), "xy_pred_mean_mm": xy_a.mean(0).round(2).tolist(), "xy_pred_sd_mm": xy_a.std(0, ddof=1).round(3).tolist(), "xy_pred_range_mm": (xy_a.max(0) - xy_a.min(0)).round(3).tolist()})
    out[str(pid)] = row; print(pid, json.dumps(row, ensure_ascii=False))
Path(ARGV[3]).write_text(json.dumps(out, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
