#!/usr/bin/env python3
"""G02 T6 판정(오프라인). 동결 입력: docs/gates/G02-동결-임계값.md(v1), evidence/G02/pilot_pnp.json(기준 위치), evidence/G02/dabai_color_camera_info.json(K, D).
모드 pilot: 파일럿 12자세로 판정 스크립트 검증(기존 수치 재현). 모드 holdout: holdout 8자세 판정 + 점검 자세(7, 5) 보드 이동 점검.
사용: python3 g02_judge.py pilot <res2a> <view2a> <res2b> <view2b> <out.json>  |  python3 g02_judge.py holdout <res_s43> <view_s43> <out.json>"""
import json, sys, hashlib, importlib.util
from pathlib import Path
import numpy as np, cv2
ARGV = list(sys.argv); sys.path.insert(0, "scripts")
spec = importlib.util.spec_from_file_location("s3", "scripts/g01_s3_path.py"); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
ci = json.load(open("evidence/G02/dabai_color_camera_info.json")); K = np.array(ci["K"]).reshape(3, 3); D = np.array(ci["D"])
R_LC = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0.0]])
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50); board = cv2.aruco.CharucoBoard((6, 5), 0.012, 0.009, dic, np.arange(30, 45, dtype=np.int32)); det = cv2.aruco.CharucoDetector(board)
TH = {"dev_mean": 4.1, "dev_max": 8.1, "pred_mean": 4.3, "pred_max": 8.5, "normal_mean": 0.9, "normal_max": 1.9}   # docs/gates/G02-동결-임계값.md v1 (mm, mm, mm, mm, deg, deg)
BRACKET_TH = 8.5; BRACKET_IDS = (7, 5); HOLDOUT_IDS = (1, 6, 10, 12, 13, 16, 17, 19)
def cam_T(q4):
    dp, dR = m.fk(np.array([[*q4, 90.0]]))["dabai"]; return dR[0] @ R_LC, dp[0]
def measure(res_f, view_d):
    out = {}
    for p in json.load(open(res_f))["poses"]:
        best = None
        for f in sorted((Path(view_d) / ("p%d" % p["pose_id"])).glob("exp_*.png")):
            g = cv2.cvtColor(cv2.imread(str(f)), cv2.COLOR_BGR2GRAY); cc, cid, _, _ = det.detectBoard(g); n = 0 if cid is None else len(cid)
            if best is None or n > best[0]: best = (n, cc, cid)
        obj, img = board.matchImagePoints(best[1], best[2]); ok, rv, tv = cv2.solvePnP(obj.reshape(-1, 3), img.reshape(-1, 2), K, D)
        rb = p["readback"]; q = np.array([rb["1"], rb["2"], rb["3"], rb["4"]], float); Rc, tc = cam_T(q); Rb = cv2.Rodrigues(rv)[0]
        proj, _ = cv2.projectPoints(obj.reshape(-1, 3), rv, tv, K, D)
        out[p["pose_id"]] = {"origin": Rc @ tv.ravel() + tc, "normal": (Rc @ Rb)[:, 2], "reproj": float(np.linalg.norm(proj.reshape(-1, 2) - img.reshape(-1, 2), axis=1).mean()), "corners": int(best[0])}
    return out
def metrics(poses, Pbar):
    P = np.array([v["origin"] for v in poses.values()]) * 1000; N = np.array([v["normal"] for v in poses.values()])
    d = np.linalg.norm(P - P.mean(0), axis=1); dp = np.linalg.norm(P - Pbar * 1000, axis=1)
    Nm = N.mean(0); Nm /= np.linalg.norm(Nm); ang = np.degrees(np.arccos(np.clip(N @ Nm, -1, 1)))
    return {"dev_mean": d.mean(), "dev_max": d.max(), "pred_mean": dp.mean(), "pred_max": dp.max(), "normal_mean": ang.mean(), "normal_max": ang.max()}
pil = json.load(open("evidence/G02/pilot_pnp.json"))
Ps = []
for r in pil:
    Rc, tc = cam_T(np.array(r["q"])); Ps.append(Rc @ np.array(r["tvec"]) + tc)
Pbar = np.mean(Ps, 0)
mode = ARGV[1]; hashes = {k: hashlib.sha256(open(k, "rb").read()).hexdigest()[:16] for k in ("evidence/G02/pilot_pnp.json", "evidence/G02/dabai_color_camera_info.json", "docs/gates/G02-동결-임계값.md")}
if mode == "pilot":
    poses = {**measure(ARGV[2], ARGV[3]), **{(100 + k): v for k, v in measure(ARGV[4], ARGV[5]).items()}}   # 2a/2b 자세 번호 충돌 방지(키만)
    mt = metrics(poses, Pbar); res = {"mode": "pilot", "n_poses": len(poses), "metrics": {k: round(float(v), 3) for k, v in mt.items()}, "hashes": hashes}
    Path(ARGV[6]).write_text(json.dumps(res, ensure_ascii=False, indent=1) + "\n", encoding="utf-8"); print("파일럿 검증:", json.dumps(res["metrics"]))
else:
    allp = measure(ARGV[2], ARGV[3]); ho = {k: v for k, v in allp.items() if k in HOLDOUT_IDS}; br = {k: v for k, v in allp.items() if k in BRACKET_IDS}
    if len(ho) != 8: sys.exit("holdout 8자세가 아니다(%d): 결손 규칙에 따라 합격선을 재계산해야 한다" % len(ho))
    mt = metrics(ho, Pbar); passed = {k: bool(mt[k] <= TH[k]) for k in TH}
    bd = {str(k): round(float(np.linalg.norm(v["origin"] - Pbar) * 1000), 2) for k, v in br.items()}
    bb = round(float(np.linalg.norm(br[7]["origin"] - br[5]["origin"]) * 1000), 2) if len(br) == 2 else None
    moved = (len(br) != 2) or any(x > BRACKET_TH for x in bd.values()) or (bb is not None and bb > BRACKET_TH)
    d_pass = passed["pred_mean"] and passed["pred_max"]
    verdict = "PASS" if all(passed.values()) and not moved else ("조건부(보드 이동 의심, (d) 판정 불가)" if moved and passed["dev_mean"] and passed["dev_max"] and passed["normal_mean"] and passed["normal_max"] else "불합격")
    res = {"mode": "holdout", "verdict": verdict, "metrics": {k: round(float(v), 3) for k, v in mt.items()}, "thresholds": TH, "pass": passed, "bracket_dist_to_pilot_mm": bd, "bracket_mutual_mm": bb, "bracket_threshold_mm": BRACKET_TH, "board_moved_suspected": moved,
           "reproj_mean_px": {str(k): round(v["reproj"], 3) for k, v in allp.items()}, "corners": {str(k): v["corners"] for k, v in allp.items()}, "hashes": hashes, "K_source": "dabai_color_camera_info.json (장치/드라이버 구분은 Orbbec Viewer 대조 전 UNVERIFIED)"}
    Path(ARGV[4]).write_text(json.dumps(res, ensure_ascii=False, indent=1) + "\n", encoding="utf-8"); print(json.dumps(res, ensure_ascii=False, indent=1))
