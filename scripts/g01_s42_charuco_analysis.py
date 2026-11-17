#!/usr/bin/env python3
"""S4-2 촬영 분석(오프라인): 자세별·노출별 ChArUco 마커/코너 검출. 보드 D1': 6x5, 사각형 12 mm, 마커 9 mm, DICT_4X4_50 ids 30~44.
사용: python3 g01_s42_charuco_analysis.py <view_dir> <out_json> [min_markers=8 min_corners=10]"""
import json, sys
from pathlib import Path
import cv2, numpy as np
view, out = Path(sys.argv[1]), Path(sys.argv[2]); MM, MC = (int(sys.argv[3]), int(sys.argv[4])) if len(sys.argv) > 4 else (8, 10)
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard((6, 5), 0.012, 0.009, dic, np.arange(30, 45, dtype=np.int32))
det = cv2.aruco.CharucoDetector(board)
res = {}
for pd in sorted(view.glob("p*"), key=lambda p: int(p.name[1:]) if p.is_dir() else -1):
    if not pd.is_dir(): continue
    per = {}
    for f in sorted(pd.glob("exp_*.png")):
        img = cv2.imread(str(f)); g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cc, ci, mc, mi = det.detectBoard(g)
        per[f.stem[4:]] = {"markers": 0 if mi is None else int(len(mi)), "corners": 0 if ci is None else int(len(ci)), "mean": round(float(g.mean()), 1), "sat_ratio": round(float((g >= 250).mean()), 3)}
    best = max(per.items(), key=lambda kv: (kv[1]["markers"], kv[1]["corners"])) if per else None
    ok = bool(best and best[1]["markers"] >= MM and best[1]["corners"] >= MC)
    res[pd.name[1:]] = {"by_exposure": per, "best_exposure": best[0] if best else None, "pass": ok}
n_ok = sum(1 for v in res.values() if v["pass"])
summ = {"criteria": {"min_markers": MM, "min_corners": MC}, "poses": res, "passed": n_ok, "total": len(res)}
out.write_text(json.dumps(summ, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
for k, v in res.items(): print("자세", k, "통과" if v["pass"] else "미달", "최선 노출", v["best_exposure"], {e: (d["markers"], d["corners"], d["sat_ratio"]) for e, d in v["by_exposure"].items()})
print("합계: %d/%d 자세 통과" % (n_ok, len(res)))
