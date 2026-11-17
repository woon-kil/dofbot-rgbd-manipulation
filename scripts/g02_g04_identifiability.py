#!/usr/bin/env python3
"""G04 방법 동결용(파일럿 12자세만 사용, holdout 미사용): 회독 잡음 아래에서 카메라 외부 파라미터 6자유도(회전 3 + 이동 3)가 12자세로 얼마나 정확히 정해지는지 몬테카를로.
참값 = 공칭(보정량 0). 각 반복: 회독에 잡음(반올림 U(±0.5°) + 흔들림 σ) → 보드 원점 위치 흩어짐 + 법선 흩어짐을 최소화하는 보정량 적합 → 보정량 분포(σ).
사용: python3 g02_g04_identifiability.py <out.json> [n_draw=60] [seed=3]"""
import json, sys, importlib.util
from pathlib import Path
import numpy as np, cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
ARGV = list(sys.argv); sys.path.insert(0, "scripts")
spec = importlib.util.spec_from_file_location("s3", "scripts/g01_s3_path.py"); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
R_LC = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0.0]]); SIG = np.array([0.0, 0.122, 0.0, 0.023])
N = int(ARGV[2]) if len(ARGV) > 2 else 60; rng = np.random.default_rng(int(ARGV[3]) if len(ARGV) > 3 else 3)
pil = json.load(open("evidence/G02/pilot_pnp.json")); qp = np.array([r["q"] for r in pil])
def cam(q, c=None):
    dp, dR = m.fk(np.array([[*q, 90.0]]))["dabai"]; Rd = np.eye(3) if c is None else Rot.from_rotvec(c[:3]).as_matrix(); t = dp[0] if c is None else dp[0] + dR[0] @ c[3:]
    return dR[0] @ Rd @ R_LC, t
Rs, Ps = [], []
for r, q in zip(pil, qp):
    Rc, tc = cam(q); Rb = cv2.Rodrigues(np.array(r["rvec"]))[0]; Rs.append(Rc @ Rb); Ps.append(Rc @ np.array(r["tvec"]) + tc)
Pbar = np.mean(Ps, 0); U, _, Vt = np.linalg.svd(np.mean(Rs, 0)); Rbar = U @ Vt
obs = []   # 참 관측: 보드 고정 + 참 FK(공칭)
for q in qp:
    Rc, tc = cam(q); obs.append((Rc.T @ Rbar, Rc.T @ (Pbar - tc)))
def resid_fn(qn):
    def f(c):
        P, Nn = [], []
        for q, (Rcb, tcb) in zip(qn, obs):
            Rc, tc = cam(q, c); P.append(Rc @ tcb + tc); Nn.append((Rc @ Rcb)[:, 2])
        P = np.array(P) * 1000; Nn = np.array(Nn); Nm = Nn.mean(0); Nm /= np.linalg.norm(Nm)
        return np.concatenate([(P - P.mean(0)).ravel(), np.degrees(np.arccos(np.clip(Nn @ Nm, -1, 1))) * 2.0])
    return f
fits = []
for _ in range(N):
    qn = qp + rng.uniform(-0.5, 0.5, qp.shape) + rng.normal(0, SIG, qp.shape)
    s = least_squares(resid_fn(qn), np.zeros(6), x_scale=[0.02] * 3 + [0.005] * 3); fits.append(s.x)
F = np.array(fits); std = F.std(0)
res = {"n_draw": N, "note": "참값 보정량 0, 파일럿 12자세 잡음만. 외부 파라미터 6자유도 적합의 표준편차(추정 불확실도).", "rot_std_deg": [round(float(np.degrees(v)), 2) for v in std[:3]], "trans_std_mm": [round(float(v) * 1000, 1) for v in std[3:]], "rot_mean_deg": [round(float(np.degrees(v)), 2) for v in F.mean(0)[:3]], "trans_mean_mm": [round(float(v) * 1000, 1) for v in F.mean(0)[3:]]}
Path(ARGV[1]).write_text(json.dumps(res, ensure_ascii=False, indent=1) + "\n", encoding="utf-8"); print(json.dumps(res, ensure_ascii=False))
