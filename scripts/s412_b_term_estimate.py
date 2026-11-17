#!/usr/bin/env python3
"""항 b 비접촉 추정 1단계(오프라인): S4-10·S4-11 촬영에서 자세별 ChArUco 보드 검출 개수·PnP 재투영 오차·카메라→보드 거리를 점검한다. 하드웨어 없음."""
import json, sys
from pathlib import Path
import cv2, numpy as np
K = np.array([[479.07, 0, 315.07], [0, 478.98, 237.41], [0, 0, 1.0]]); D = np.array(json.load(open("evidence/G02/dabai_color_camera_info.json"))["D"])
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50); board = cv2.aruco.CharucoBoard((6, 5), 0.012, 0.009, dic, np.arange(30, 45, dtype=np.int32)); det = cv2.aruco.CharucoDetector(board)
out = {}
for run in sys.argv[1:]:
    base = Path("evidence/G01/raw/s42") / run / "g01_s42_view"
    for pd in sorted(base.glob("p*")):
        if pd.is_dir() is False or pd.name.endswith("v1") or "v" in pd.name[1:]: continue
        best = None
        for f in sorted(pd.glob("exp_*.png")):
            g = cv2.cvtColor(cv2.imread(str(f)), cv2.COLOR_BGR2GRAY); cc, ci, _, _ = det.detectBoard(g); n = 0 if ci is None else len(ci)
            if best is None or n > best[0]: best = (n, f.name, cc, ci)
        n, nm, cc, ci = best; rec = {"corners": int(n), "file": nm}
        if n >= 6:
            obj, img = board.matchImagePoints(cc, ci); obj = obj.reshape(-1, 3); img = img.reshape(-1, 2)
            ok, rv, tv = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_ITERATIVE); proj, _ = cv2.projectPoints(obj, rv, tv, K, D)
            rec.update({"reproj_mean_px": round(float(np.linalg.norm(proj.reshape(-1, 2) - img, axis=1).mean()), 2), "dist_mm": round(float(np.linalg.norm(tv) * 1000), 1), "tvec_mm": np.round(tv.ravel() * 1000, 1).tolist()})
        out["%s/%s" % (run, pd.name)] = rec; print(run, pd.name, rec)
json.dump(out, open("evidence/G02/s412_capture_check.json", "w"), ensure_ascii=False, indent=1)
