#!/usr/bin/env python3
"""G02 T1 매핑 감사(오프라인): 12자세 PnP + FK로 보드 위치 일관성을 계산하되 ① 관절 부호 반전 ② 영점 오프셋 민감도 ③ 영점 오프셋 4개 적합(타당성 확인, 보정 아님)을 비교한다.
사용: python3 g02_t1_sign_audit.py <result2a.json> <view2a> <result2b.json> <view2b> <out.json>"""
import json, sys, importlib.util
from pathlib import Path
import cv2, numpy as np
from scipy.optimize import least_squares
ARGV = list(sys.argv); sys.path.insert(0, "scripts")
spec = importlib.util.spec_from_file_location("s3", "scripts/g01_s3_path.py"); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
K = np.array([[479.07, 0, 315.07], [0, 478.98, 237.41], [0, 0, 1.0]]); D = np.array(json.load(open("evidence/G02/dabai_color_camera_info.json"))["D"])   # 드라이버 camera_info 왜곡(rational_polynomial 8계수)
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50); board = cv2.aruco.CharucoBoard((6, 5), 0.012, 0.009, dic, np.arange(30, 45, dtype=np.int32)); det = cv2.aruco.CharucoDetector(board)
R_LC = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0.0]]); rows = []
for res_f, view_d in ((ARGV[1], ARGV[2]), (ARGV[3], ARGV[4])):
    for p in json.load(open(res_f))["poses"]:
        best = None
        for f in sorted((Path(view_d) / ("p%d" % p["pose_id"])).glob("exp_*.png")):
            g = cv2.cvtColor(cv2.imread(str(f)), cv2.COLOR_BGR2GRAY); cc, ci, _, _ = det.detectBoard(g); n = 0 if ci is None else len(ci)
            if best is None or n > best[0]: best = (n, cc, ci)
        obj, img = board.matchImagePoints(best[1], best[2]); ok, rv, tv = cv2.solvePnP(obj.reshape(-1, 3), img.reshape(-1, 2), K, D)
        rb = p["readback"]; rows.append((cv2.Rodrigues(rv)[0], tv.ravel(), np.array([rb["1"], rb["2"], rb["3"], rb["4"]], float)))
def dev(sign=(1, 1, 1, 1), off=(0, 0, 0, 0)):
    P = []
    for Rcb, tcb, q in rows:
        qq = 90.0 + np.array(sign) * (q - 90.0) + np.array(off); dp, dR = m.fk(np.array([[*qq, 90.0]]))["dabai"]
        P.append(dR[0] @ R_LC @ tcb + dp[0])
    P = np.array(P) * 1000; d = np.linalg.norm(P - P.mean(0), axis=1); return float(d.mean()), float(d.max())
def center(off=(0, 0, 0, 0)):
    P = []
    for Rcb, tcb, q in rows:
        qq = 90.0 + q - 90.0 + np.array(off); dp, dR = m.fk(np.array([[*qq, 90.0]]))["dabai"]
        P.append(dR[0] @ R_LC @ (Rcb @ np.array([0.036, 0.030, 0.0]) + tcb) + dp[0])
    return np.array(P).mean(0) * 1000
out = {"nominal": dev()}
c0 = center(); out["board_center_base_mm"] = [round(float(v), 1) for v in c0]
out["board_center_shift_per_offset_mm"] = {"J%d" % (j + 1): [round(float(v), 1) for v in (center(tuple(1.0 if k == j else 0 for k in range(4))) - c0)] for j in range(4)}
out["sign_flip"] = {"J%d" % (j + 1): dev(tuple(-1 if k == j else 1 for k in range(4))) for j in range(4)}
out["offset_plus1"] = {"J%d" % (j + 1): dev(off=tuple(1.0 if k == j else 0 for k in range(4))) for j in range(4)}
out["offset_plus2"] = {"J%d" % (j + 1): dev(off=tuple(2.0 if k == j else 0 for k in range(4))) for j in range(4)}
sol = least_squares(lambda o: np.array([dev(off=tuple(o))[0]]), np.zeros(4), x_scale=1.0, diff_step=0.5)
out["offset_fit_feasibility"] = {"note": "타당성 확인만, 보정 아님", "offsets_deg": [round(float(v), 2) for v in sol.x], "dev_mean_max_mm": [round(v, 2) for v in dev(off=tuple(sol.x))]}
Path("evidence/G02/pilot_pnp.json").write_text(json.dumps([{"rvec": cv2.Rodrigues(R)[0].ravel().tolist(), "tvec": tv.tolist(), "q": q.tolist()} for R, tv, q in rows], indent=1) + "\n", encoding="utf-8")   # T2 입력
Path(ARGV[5]).write_text(json.dumps(out, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
print("공칭 (평균, 최대 mm): %.2f, %.2f" % out["nominal"])
for k in ("sign_flip", "offset_plus1", "offset_plus2"):
    print(k, {a: (round(b[0], 1), round(b[1], 1)) for a, b in out[k].items()})
print("보드 중심(베이스 mm):", out["board_center_base_mm"]); print("오프셋 +1° 당 보드 중심 이동(mm):", out["board_center_shift_per_offset_mm"])
print("영점 오프셋 적합(타당성):", out["offset_fit_feasibility"])
