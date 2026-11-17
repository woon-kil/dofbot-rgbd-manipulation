#!/usr/bin/env python3
"""G04 확증 판정(동결 합격선 v1, 풀이 재적합 없음). 입력: S4-5 결과 JSON, 영상 폴더(g01_s42_view). 동결 해(evidence/G02/g04_solve.json '공칭')를 그대로 적용한다.
예측 보드 중심 = Rc@(Rb@[0.036,0.030,0]+tvec)+tc, Rc=dR@Rd@R_LC, 관절은 회독 + J1 영점. 오차 = |P_xy - 실측 보드 xy| (mm). 점검 자세 7·5는 판정에서 제외(별도 점검).
검출 충족 = 최선 노출에서 마커 ≥8, 코너 ≥10 (S4-4와 동일). 사용: python3 g04_confirm_judge.py <result.json> <view_dir> <out.json>"""
import json, sys, importlib.util
from pathlib import Path
import numpy as np, cv2
from scipy.spatial.transform import Rotation as Rot
ARGV = list(sys.argv)   # g01_s3_path import가 argv를 바꾸므로 먼저 저장(g04_solve.py와 동일)
spec = importlib.util.spec_from_file_location("s3", "scripts/g01_s3_path.py"); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
ci = json.load(open("evidence/G02/dabai_color_camera_info.json")); K = np.array(ci["K"]).reshape(3, 3); D = np.array(ci["D"]); R_LC = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0.0]])
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50); board = cv2.aruco.CharucoBoard((6, 5), 0.012, 0.009, dic, np.arange(30, 45, dtype=np.int32)); det = cv2.aruco.CharucoDetector(board)
BXY = np.array(json.load(open("evidence/G02/board_measurement.json"))["derived_base_frame"]["board_center_xy_mm"], float)
F = json.load(open("evidence/G02/g04_solve.json"))["fit"]["공칭"]; rot = np.radians(F["rot_deg"]); tr = np.array(F["trans_mm"]) / 1000; j1z = F["j1_zero_deg"]
Rd = Rot.from_rotvec(rot).as_matrix(); CONFIRM = range(200, 208); MIN_MARK, MIN_CORN, MIN_POSES = 8, 10, 6; TH_MEAN, TH_MAX = 7.5, 11.5
res_f, view_d, out_f = ARGV[1:4]; rows = []
for p in json.load(open(res_f))["poses"]:
    pid = p["pose_id"]
    if pid not in CONFIRM: continue
    best = None
    for f in sorted((Path(view_d) / ("p%d" % pid)).glob("exp_*.png")):
        g = cv2.cvtColor(cv2.imread(str(f)), cv2.COLOR_BGR2GRAY); cc, cid, mc, mid = det.detectBoard(g); nc = 0 if cid is None else len(cid); nm = 0 if mid is None else len(mid)
        if best is None or nc > best[0]: best = (nc, nm, cc, cid)
    row = {"pose_id": pid, "corners": best[0] if best else 0, "markers": best[1] if best else 0, "detected": bool(best and best[0] >= MIN_CORN and best[1] >= MIN_MARK)}
    if row["detected"]:
        obj, img = board.matchImagePoints(best[2], best[3]); ok, rv, tv = cv2.solvePnP(obj.reshape(-1, 3), img.reshape(-1, 2), K, D); rb = p["readback"]
        q = np.array([[rb["1"] + j1z, rb["2"], rb["3"], rb["4"], 90.0]]); dp, dR = m.fk(q)["dabai"]; Rc = dR[0] @ Rd @ R_LC; tc = dp[0] + dR[0] @ tr
        P = Rc @ (cv2.Rodrigues(rv)[0] @ np.array([0.036, 0.030, 0.0]) + tv.ravel()) + tc; row["pred_xy_mm"] = (P[:2] * 1000).round(2).tolist(); row["err_mm"] = round(float(np.linalg.norm(P[:2] * 1000 - BXY)), 2)
    rows.append(row)
det_rows = [r for r in rows if r["detected"]]; n = len(det_rows); errs = np.array([r["err_mm"] for r in det_rows])
out = {"poses": rows, "n_detected": n, "frozen": {"th_mean": TH_MEAN, "th_max": TH_MAX, "min_poses": MIN_POSES}}
if n < MIN_POSES: out["verdict"] = "UNDECIDABLE(검출 %d<6)" % n
else:
    out.update({"err_mean_mm": round(float(errs.mean()), 2), "err_max_mm": round(float(errs.max()), 2)})
    if n < 8: out["verdict"] = "RECOMPUTE_THRESHOLD(결손 규칙: 부분집합 합격선 재계산 필요, 8→%d)" % n
    else: out["verdict"] = "PASS_ERROR(점검 별도)" if errs.mean() <= TH_MEAN and errs.max() <= TH_MAX else "FAIL"
    out["note"] = "PASS는 보드 불변 점검(g04_bracket_check) 통과가 추가로 필요하다"
Path(out_f).write_text(json.dumps(out, ensure_ascii=False, indent=1) + "\n", encoding="utf-8"); print(json.dumps(out, ensure_ascii=False, indent=1))
