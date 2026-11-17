#!/usr/bin/env python3
"""G02 T2: 회독 잡음만으로 설명되는 보드 위치 편차 분포(몬테카를로). 잡음 모델(플랜 5장, 실측으로 고정): 관절별 회독 반올림 U(-0.5,+0.5)° + 정지 흔들림 N(0, sigma_j) (J1 0, J2 0.122, J3 0, J4 0.023°).
검증: 파일럿 12자세에 같은 시뮬레이션을 적용해 관측 편차(평균 2.27, 최대 4.25 mm)와 비교. 판정 대상: holdout 8자세(후보 JSON의 sdk_deg).
사용: python3 g02_quant_sim.py <out.json> [seed]"""
import json, sys, importlib.util
from pathlib import Path
import numpy as np
ARGV = list(sys.argv); sys.path.insert(0, "scripts")
spec = importlib.util.spec_from_file_location("s3", "scripts/g01_s3_path.py"); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
R_LC = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0.0]]); SIG = np.array([0.0, 0.122, 0.0, 0.023]); rng = np.random.default_rng(int(ARGV[2]) if len(ARGV) > 2 else 0)
import cv2
pil = json.load(open("evidence/G02/pilot_pnp.json"))
def cam_T(q4):
    dp, dR = m.fk(np.array([[*q4, 90.0]]))["dabai"]; R = dR[0] @ R_LC; return R, dp[0]
# 파일럿 보드 위치(공칭 FK): 12자세 평균 원점 위치·평균 회전
Rs, Ps = [], []
for r in pil:
    Rc, tc = cam_T(r["q"]); Rb = cv2.Rodrigues(np.array(r["rvec"]))[0]; Rs.append(Rc @ Rb); Ps.append(Rc @ np.array(r["tvec"]) + tc)
Pbar = np.mean(Ps, 0); U, _, Vt = np.linalg.svd(np.mean(Rs, 0)); Rbar = U @ Vt
def sim(qs, n, deadband=0.0):
    """qs: (N,4) 참값 관절(SDK). 참 PnP 관측 = 보드 고정 + 참 FK. 회독 = 반올림 + 흔들림 (+ 선택: 데드밴드). 반환: 평균·최대 편차, 평균 대비/파일럿 평균 대비"""
    true = []
    for q in qs:
        Rc, tc = cam_T(q); true.append((Rc, tc, Rc.T @ Rbar, Rc.T @ (Pbar - tc)))     # (R_cb, t_cb)
    nb = Rbar[:, 2]
    out = []
    for _ in range(n):
        P = []; N = []
        for q, (Rc, tc, Rcb, tcb) in zip(qs, true):
            e = rng.uniform(-0.5, 0.5, 4) + rng.normal(0, SIG) + (rng.uniform(-deadband, deadband, 4) if deadband else 0)
            R2, t2 = cam_T(q + e); P.append(R2 @ tcb + t2); N.append((R2 @ Rcb)[:, 2])
        P = np.array(P) * 1000; d = np.linalg.norm(P - P.mean(0), axis=1); dp = np.linalg.norm(P - Pbar * 1000, axis=1)
        N = np.array(N); Nm = N.mean(0); Nm /= np.linalg.norm(Nm); ang = np.degrees(np.arccos(np.clip(N @ Nm, -1, 1)))
        out.append((d.mean(), d.max(), dp.mean(), dp.max(), ang.mean(), ang.max()))
    o = np.array(out); return {k: {"mean": round(float(o[:, i].mean()), 2), "p95": round(float(np.percentile(o[:, i], 95)), 2), "max": round(float(o[:, i].max()), 2)} for i, k in enumerate(["dev_mean", "dev_max", "pred_mean", "pred_max", "normal_spread_mean", "normal_spread_max"])}
qp = np.array([r["q"] for r in pil]); cand = json.load(open("evidence/G01/b4_pose_candidates_y230_72x60_j34min10_clear60.json"))["poses"]
qh = np.array([p["sdk_deg"][:4] for p in cand if p["split"] == "holdout"], float)
res = {"noise_model": {"rounding_deg": "U(-0.5,0.5)", "sigma_flicker_deg": SIG.tolist()}, "n_draw": 400,
       "pilot12_validation": sim(qp, 400), "holdout8": sim(qh, 400), "holdout8_sensitivity_deadband_pm1": sim(qh, 400, deadband=1.0)}
res["observed_pilot"] = {"dev_mean": 2.27, "dev_max": 4.25}
Path(ARGV[1]).write_text(json.dumps(res, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
for k in ("pilot12_validation", "holdout8", "holdout8_sensitivity_deadband_pm1"): print(k, json.dumps(res[k], ensure_ascii=False))
print("관측 파일럿:", res["observed_pilot"])
