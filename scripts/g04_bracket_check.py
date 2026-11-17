#!/usr/bin/env python3
"""G04/S4-4 보드 이동 점검(오프라인): 점검 자세(파일럿 7·5)를 같은 자세의 파일럿 추정과 비교한다(공칭 FK·PnP, K·D는 camera_info).
자세 매칭은 순서가 아니라 2a/2b 결과 JSON의 회독값으로 한다(assert). 사용: python3 g04_bracket_check.py <result_new.json> <view_new> [<out.json>]"""
import json, sys, importlib.util
from pathlib import Path
import numpy as np, cv2
ARGV = list(sys.argv); sys.path.insert(0, "scripts")
spec = importlib.util.spec_from_file_location("s3", "scripts/g01_s3_path.py"); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
ci = json.load(open("evidence/G02/dabai_color_camera_info.json")); K = np.array(ci["K"]).reshape(3, 3); D = np.array(ci["D"]); R_LC = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0.0]])
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50); board = cv2.aruco.CharucoBoard((6, 5), 0.012, 0.009, dic, np.arange(30, 45, dtype=np.int32)); det = cv2.aruco.CharucoDetector(board)
def origin(q, rvec, tvec):
    dp, dR = m.fk(np.array([[*q, 90.0]]))["dabai"]; return (dR[0] @ R_LC) @ np.array(tvec) + dp[0]
def measure(res_f, view_d):
    out = {}
    for p in json.load(open(res_f))["poses"]:
        best = None
        for f in sorted((Path(view_d) / ("p%d" % p["pose_id"])).glob("exp_*.png")):
            g = cv2.cvtColor(cv2.imread(str(f)), cv2.COLOR_BGR2GRAY); cc, cid, _, _ = det.detectBoard(g); n = 0 if cid is None else len(cid)
            if best is None or n > best[0]: best = (n, cc, cid)
        obj, img = board.matchImagePoints(best[1], best[2]); ok, rv, tv = cv2.solvePnP(obj.reshape(-1, 3), img.reshape(-1, 2), K, D)
        rb = p["readback"]; q = [rb["1"], rb["2"], rb["3"], rb["4"]]; out[p["pose_id"]] = {"q": q, "rvec": rv.ravel(), "tvec": tv.ravel(), "origin": origin(np.array(q, float), rv, tv.ravel())}
    return out
RES = "evidence/G01/raw/"; pilot = {}
for rf, vd in ((RES + "g01_5b4_s422a_result-real-261112-213337.json", RES + "s42/261112-213337/g01_s42_view"), (RES + "g01_5b4_s422b_result-real-261112-215835.json", RES + "s42/261112-215835/g01_s42_view")): pilot.update(measure(rf, vd))
pj = json.load(open("evidence/G02/pilot_pnp.json")); P_by_q = {tuple(int(v) for v in r["q"]): r for r in pj}
new = measure(ARGV[1], ARGV[2]); rows = {}
for pid in (7, 5):
    assert pid in new and pid in pilot, "점검 자세 %d 누락" % pid
    qp, qn = pilot[pid]["q"], new[pid]["q"]; assert tuple(int(v) for v in qp) in P_by_q, "pilot_pnp.json에서 회독 일치 자세를 찾지 못함(순서 가정 금지)"
    d_base = float(np.linalg.norm(new[pid]["origin"] - pilot[pid]["origin"]) * 1000); same_q = (qp == qn)
    d_cam = float(np.linalg.norm(new[pid]["tvec"] - pilot[pid]["tvec"]) * 1000) if same_q else None
    rows[pid] = {"q_pilot": qp, "q_new": qn, "readback_identical": same_q, "base_origin_diff_mm": round(d_base, 2), "camera_frame_tvec_diff_mm": None if d_cam is None else round(d_cam, 2)}
print(json.dumps(rows, ensure_ascii=False, indent=1))
if len(ARGV) > 3: Path(ARGV[3]).write_text(json.dumps(rows, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
