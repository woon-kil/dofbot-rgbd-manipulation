#!/usr/bin/env python3
"""폐루프 파일럿 항 c 분석(탐색, 합격선 없음): 같은 자세를 여러 번 재도달해 방문별로 ChArUco PnP → 동결 해로 보드 xy 예측. 방문 간 흩어짐(재도달 + 관측)과 방문 내 흩어짐(관측 잡음)을 분리해 보고한다.
사용: python3 g04_pilot_c_analysis.py <result.json> <view_dir> <out.json>"""
import json, sys, importlib.util
from pathlib import Path
import numpy as np, cv2
from scipy.spatial.transform import Rotation as Rot
ARGV = list(sys.argv)
spec = importlib.util.spec_from_file_location("s3", "scripts/g01_s3_path.py"); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
ci = json.load(open("evidence/G02/dabai_color_camera_info.json")); K = np.array(ci["K"]).reshape(3, 3); D = np.array(ci["D"]); R_LC = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0.0]])
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50); board = cv2.aruco.CharucoBoard((6, 5), 0.012, 0.009, dic, np.arange(30, 45, dtype=np.int32)); det = cv2.aruco.CharucoDetector(board)
F = json.load(open("evidence/G02/g04_solve.json"))["fit"]["공칭"]; Rd = Rot.from_rotvec(np.radians(F["rot_deg"])).as_matrix(); tr = np.array(F["trans_mm"]) / 1000; j1z = F["j1_zero_deg"]
res_f, view_d, out_f = ARGV[1:4]; visits = {}
for p in json.load(open(res_f))["poses"]:
    pid = p["pose_id"]; tag = p["capture"].get("view_tag", str(pid)); d = Path(view_d) / ("p" + tag); rb = p["readback"]
    q = np.array([[rb["1"] + j1z, rb["2"], rb["3"], rb["4"], 90.0]]); dp, dR = m.fk(q)["dabai"]; Rc = dR[0] @ Rd @ R_LC; tc = dp[0] + dR[0] @ tr
    xy, tv_l = [], []
    for f in sorted(d.glob("exp_*.png")):
        g = cv2.cvtColor(cv2.imread(str(f)), cv2.COLOR_BGR2GRAY); cc, cid, mc, mid = det.detectBoard(g); nc = 0 if cid is None else len(cid); nm = 0 if mid is None else len(mid)
        if nc < 10 or nm < 8: continue
        obj, img = board.matchImagePoints(cc, cid); ok, rv, tv = cv2.solvePnP(obj.reshape(-1, 3), img.reshape(-1, 2), K, D)
        xy.append((Rc @ (cv2.Rodrigues(rv)[0] @ np.array([0.036, 0.030, 0.0]) + tv.ravel()) + tc)[:2] * 1000); tv_l.append(tv.ravel() * 1000)
    visits.setdefault(pid, []).append({"tag": tag, "readback": [rb["1"], rb["2"], rb["3"], rb["4"]], "n_det": len(xy), "xy": np.array(xy).mean(0).tolist() if xy else None, "xy_within_sd": np.array(xy).std(0, ddof=1).tolist() if len(xy) > 1 else None, "tvec": np.array(tv_l).mean(0).tolist() if tv_l else None})
out = {}
for pid, vs in visits.items():
    ok = [v for v in vs if v["xy"] is not None]; row = {"n_visits": len(vs), "n_with_detection": len(ok), "readbacks": [v["readback"] for v in vs], "n_det_per_visit": [v["n_det"] for v in vs]}
    if len(ok) >= 3:
        xy = np.array([v["xy"] for v in ok]); tv = np.array([v["tvec"] for v in ok]); w = np.array([v["xy_within_sd"] for v in ok if v["xy_within_sd"]])
        row.update({"xy_between_sd_mm": xy.std(0, ddof=1).round(3).tolist(), "xy_between_range_mm": (xy.max(0) - xy.min(0)).round(3).tolist(), "xy_within_sd_mean_mm": w.mean(0).round(3).tolist(), "tvec_between_sd_mm": tv.std(0, ddof=1).round(3).tolist(), "xy_mean_mm": xy.mean(0).round(2).tolist()})
        rbs = np.array([v["readback"] for v in ok]); row["readback_sd_deg"] = rbs.std(0, ddof=1).round(2).tolist(); row["readback_all_identical"] = bool((rbs == rbs[0]).all())
    same = [v for v in ok if v["readback"] == ok[0]["readback"]] if ok else []   # 회독이 첫 방문과 같은 방문만 따로 묶은 흩어짐
    if len(same) >= 3: xs = np.array([v["xy"] for v in same]); row["xy_between_sd_same_readback_mm"] = xs.std(0, ddof=1).round(3).tolist(); row["n_same_readback"] = len(same)
    out[str(pid)] = row; print(pid, json.dumps(row, ensure_ascii=False))
ks = [k for k in out if "xy_mean_mm" in out[k]]
if len(ks) >= 2:   # 작은 이동 일관성: 서로 다른 두 자세의 평균 보드 xy 차이(모델이 맞으면 0). J1 5°·J3 5° 이동에서 FK·해의 일관성
    a_, b_ = np.array(out[ks[0]]["xy_mean_mm"]), np.array(out[ks[1]]["xy_mean_mm"]); out["pose_to_pose_mean_xy_diff_mm"] = {"poses": ks[:2], "diff": (b_ - a_).round(3).tolist(), "norm": round(float(np.linalg.norm(b_ - a_)), 3)}; print(out["pose_to_pose_mean_xy_diff_mm"])
Path(ARGV[3]).write_text(json.dumps(out, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
