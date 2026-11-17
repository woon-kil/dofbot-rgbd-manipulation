#!/usr/bin/env python3
"""G04 추가 자세 설계(오프라인, 하드웨어 없음, 모델 값=UNVERIFIED). 파일럿 12자세 + 후보 격자에서 그리디로 자세를 추가하며, 회독 잡음 아래 카메라 외부 파라미터 6자유도의 추정 표준편차(선형화)를 줄인다.
카메라 자세는 J1(요)과 J2+J3+J4(피치 합)만으로 정해진다(J5 무관, 확인함). 학습 자세는 파일럿 12 + 추가 자세; holdout 8과 점검 2는 쓰지 않는다.
사용: python3 g04_pose_design.py <out.json>"""
import json, sys, importlib.util
from pathlib import Path
import numpy as np, cv2
ARGV = list(sys.argv); sys.path.insert(0, "scripts")
spec = importlib.util.spec_from_file_location("s3", "scripts/g01_s3_path.py"); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
R_LC = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0.0]]); SIG = np.sqrt(1 / 12 + np.array([0.0, 0.122, 0.0, 0.023]) ** 2)   # 관절별 총 잡음 σ(반올림 + 흔들림), 도
ROT_LIM, TRANS_LIM = 0.5, 2.0   # G04 방법 동결 M3: 회전 > 0.5° 또는 이동 > 2 mm면 추가
import os
K = (479.07, 478.98, 315.07, 237.41); W, H = 640, 480
MARGIN = int(os.environ.get("G4_MARGIN", 80)); DEPTH = (0.15, float(os.environ.get("G4_DEPTH_MAX", 0.30))); VIEW_MAX = float(os.environ.get("G4_VIEW_MAX", 40)); CLEAR = 60.0
J1R = (int(os.environ.get("G4_J1_LO", 75)), int(os.environ.get("G4_J1_HI", 105)))
STEPS = int(os.environ.get("G4_STEPS", 14))
BOARD_C = np.array([0.0, 0.230, 0.003]); BOARD_HALF = (0.036, 0.030)
box = np.array([[sx * 0.045, sy * 0.0125, sz * 0.0125] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
def cam(q4s, c=None):
    """q4s (N,4) → (Rc (N,3,3), tc (N,3)). c: (6,) 보정(회전벡터 3, 이동 3 m)"""
    f = m.fk(np.hstack([q4s, np.full((len(q4s), 1), 90.0)])); dp, dR = f["dabai"]
    if c is None: return dR @ R_LC, dp
    from scipy.spatial.transform import Rotation as Rot
    Rd = Rot.from_rotvec(c[:3]).as_matrix(); return dR @ Rd @ R_LC, dp + dR @ c[3:]
pil = json.load(open("evidence/G02/pilot_pnp.json")); qp = np.array([r["q"] for r in pil])
Rs, Ps = [], []
Rc0, tc0 = cam(qp)
for i, r in enumerate(pil):
    Rb = cv2.Rodrigues(np.array(r["rvec"]))[0]; Rs.append(Rc0[i] @ Rb); Ps.append(Rc0[i] @ np.array(r["tvec"]) + tc0[i])
Pbar = np.mean(Ps, 0); U, _, Vt = np.linalg.svd(np.mean(Rs, 0)); Rbar = U @ Vt; Nbar = Rbar[:, 2]
def jac(qs):
    """각 자세의 (위치 3, 법선 3)에 대한 c(6), e(4) 야코비안. 참 관측 = 보드 고정 + 공칭 FK."""
    n = len(qs); Rc, tc = cam(qs); Rcb = np.einsum("nji,jk->nik", Rc, Rbar); tcb = np.einsum("nji,nj->ni", Rc, (Pbar - tc))
    def out(Rc2, tc2): return np.einsum("nij,nj->ni", Rc2, tcb) + tc2, np.einsum("nij,njk->nik", Rc2, Rcb)[:, :, 2]
    P0, N0 = out(*cam(qs)); Jc = np.zeros((n, 6, 6)); Je = np.zeros((n, 6, 4)); h = 1e-4
    for k in range(6):
        c = np.zeros(6); c[k] = h; P1, N1 = out(*cam(qs, c)); Jc[:, :3, k] = (P1 - P0) * 1000 / h; Jc[:, 3:, k] = (N1 - N0) * 114.59 / h   # 법선 1 rad = 114.59(= 도당 2 mm 환산, 위치 mm와 같은 눈금)
    for k in range(4):
        d = np.zeros(4); d[k] = 0.05; P1, N1 = out(*cam(qs + d)); Je[:, :3, k] = (P1 - P0) * 1000 / 0.05; Je[:, 3:, k] = (N1 - N0) * 114.59 / 0.05
    return Jc, Je
def sigma(Jc, Je):
    """집합의 외부 파라미터 추정 σ (회전 rad→deg 3, 이동 m→mm 3). 중심화(보드 위치·법선 미지)된 야코비안."""
    n = len(Jc); Jcm = (Jc - Jc.mean(0)).reshape(n * 6, 6); Jem = np.zeros((n * 6, n * 4))
    for i in range(n):
        for j in range(n):
            Jem[i * 6:(i + 1) * 6, j * 4:(j + 1) * 4] = (Je[i] if i == j else 0) - Je[j] / n
    A = -np.linalg.pinv(Jcm) @ Jem; cov = A @ np.diag(np.tile(SIG ** 2, n)) @ A.T; sd = np.sqrt(np.diag(cov))
    sigma.cov = cov
    return np.degrees(sd[:3]), sd[3:] * 1000
Jc_p, Je_p = jac(qp); r0, t0 = sigma(Jc_p, Je_p)
print("파일럿 12 (선형화) 회전 σ°", r0.round(2), "이동 σ mm", t0.round(1), "(몬테카를로: 0.68/1.03/1.14°, 3.7/8.2/2.1 mm)")
# 후보 격자
g = np.array(np.meshgrid(np.arange(J1R[0], J1R[1] + 1, 5), np.arange(50, 151, 5), np.arange(10, 101, 5), np.arange(10, 101, 5), indexing="ij")).reshape(4, -1).T.astype(float)
f = m.fk(np.hstack([g, np.full((len(g), 1), 90.0)])); dp, dR = f["dabai"]; tcp = f["tcp"][0]
zmin = np.min(dp[:, None, 2] + np.einsum("nij,kj->nki", dR, box)[:, :, 2], axis=1)
clear = np.minimum.reduce([f["Arm2_Joint"][0][:, 2], f["Arm3_Joint"][0][:, 2], f["Arm4_Joint"][0][:, 2], f["arm5"][0][:, 2], tcp[:, 2], zmin]) * 1000
Rc, tc = dR @ R_LC, dp; corners = np.array([BOARD_C + np.array([sx * BOARD_HALF[0], sy * BOARD_HALF[1], 0]) for sx in (-1, 1) for sy in (-1, 1)])
pc = np.einsum("nji,nkj->nki", Rc, corners[None] - tc[:, None, :]); zc = pc[..., 2]; cen = np.einsum("nji,nj->ni", Rc, BOARD_C[None] - tc)
u = K[0] * pc[..., 0] / np.where(zc > 0, zc, 1) + K[2]; v = K[1] * pc[..., 1] / np.where(zc > 0, zc, 1) + K[3]
ok = (clear >= CLEAR) & (cen[:, 2] >= DEPTH[0]) & (cen[:, 2] <= DEPTH[1]) & np.all(zc > 0.05, axis=1) & np.all((u >= MARGIN) & (u <= W - MARGIN) & (v >= MARGIN) & (v <= H - MARGIN), axis=1)
axis_w = Rc[:, :, 2]; view = np.degrees(np.arccos(np.clip(-axis_w[:, 2], -1, 1))); ok &= view <= VIEW_MAX
pool = g[ok]; print("후보 격자", len(g), "필터 통과", len(pool))
cand_json = json.load(open("evidence/G01/b4_pose_candidates_y230_72x60_j34min10_clear60.json"))["poses"]
used = {tuple(map(int, q)) for q in qp} | {tuple(p["sdk_deg"][:4]) for p in cand_json if p["split"] == "holdout"} | {tuple(p["sdk_deg"][:4]) for p in cand_json if p["id"] in (7, 5)}   # 파일럿 + holdout + 점검 자세는 후보에서 제외(학습·판정 독립)
pool = np.array([q for q in pool if tuple(map(int, q)) not in used]); print('제외 후 후보', len(pool))
Jc_c, Je_c = jac(pool)
chosen = []; Jc_s, Je_s = Jc_p.copy(), Je_p.copy(); steps = []
for step in range(STEPS):
    best = None
    for idx in range(len(pool)):
        if idx in chosen: continue
        r, t = sigma(np.concatenate([Jc_s, Jc_c[idx:idx + 1]]), np.concatenate([Je_s, Je_c[idx:idx + 1]])); fobj = max(r.max() / ROT_LIM, t.max() / TRANS_LIM)
        if best is None or fobj < best[0]: best = (fobj, idx, r, t)
    chosen.append(best[1]); Jc_s = np.concatenate([Jc_s, Jc_c[best[1]:best[1] + 1]]); Je_s = np.concatenate([Je_s, Je_c[best[1]:best[1] + 1]])
    steps.append({"n_added": step + 1, "pose_sdk_j1_j4": pool[best[1]].astype(int).tolist(), "clear_mm": None, "rot_sigma_deg": best[2].round(2).tolist(), "trans_sigma_mm": best[3].round(1).tolist(), "objective": round(float(best[0]), 2)})
    print({k: steps[-1][k] for k in ('n_added','pose_sdk_j1_j4','rot_sigma_deg','trans_sigma_mm','objective')})
    if best[0] <= 1.0: break
def pred_unc(cov, Jc_eval):
    Jp = Jc_eval[:, :3, :]; return np.sqrt(np.einsum("nik,kl,nil->n", Jp, cov, Jp))
Jc_all, _ = jac(pool); out_pred = {}
for label, idxs in (("pilot12", []), ("+4", chosen[:4]), ("+8", chosen[:8]), ("+14", chosen[:14])):
    Jc_set = np.concatenate([Jc_p] + [Jc_c[i:i + 1] for i in idxs]); Je_set = np.concatenate([Je_p] + [Je_c[i:i + 1] for i in idxs]); sigma(Jc_set, Je_set); u = pred_unc(sigma.cov, Jc_all)
    out_pred[label] = {"mean_mm": round(float(u.mean()), 2), "p95_mm": round(float(np.percentile(u, 95)), 2), "max_mm": round(float(u.max()), 2)}; print("예측 위치 불확실성(보드 원점, 풀 전체 자세)", label, out_pred[label])
Path(ARGV[1]).write_text(json.dumps({"limits": {"rot_deg": ROT_LIM, "trans_mm": TRANS_LIM}, "pilot12": {"rot_sigma_deg": r0.round(2).tolist(), "trans_sigma_mm": t0.round(1).tolist()}, "pool_size": len(pool), "greedy": steps, "pred_uncertainty_over_pool_mm": out_pred}, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")

# ---- 보드 절대 위치를 알 때(실측 prior)의 식별성: 일반화 최소제곱 [외부 파라미터 c(6) + 보드 위치 b(3)] ----
def gls_cov(Jc_set, Je_set, sigma_b, sig_pnp_mm=0.5, sig_pnp_n_deg=0.3, w_n=114.59, sig_n_known_deg=None):
    n = len(Jc_set); rows, Sinv_blocks = [], []
    Sig_e = np.diag(SIG ** 2); H = np.zeros((n * 6 + 3, 6 + 3)); W = np.zeros((n * 6 + 3, n * 6 + 3))
    for i in range(n):
        H[i * 6:i * 6 + 6, :6] = Jc_set[i]; H[i * 6:i * 6 + 3, 6:9] = -np.eye(3)   # 위치 잔차 = P_i − b ; 법선 잔차는 보드 법선이 알려졌다고(수평) 두고 b와 무관
        S = Je_set[i] @ Sig_e @ Je_set[i].T + np.diag([sig_pnp_mm ** 2] * 3 + [(np.radians(sig_pnp_n_deg) * w_n) ** 2] * 3)
        W[i * 6:i * 6 + 6, i * 6:i * 6 + 6] = np.linalg.inv(S)
    H[n * 6:, 6:9] = np.eye(3); W[n * 6:, n * 6:] = np.eye(3) / sigma_b ** 2 if sigma_b < 1e8 else np.zeros((3, 3))
    info = H.T @ W @ H
    return np.linalg.inv(info + 1e-12 * np.eye(9))
print("\n[보드 절대 위치를 알 때] 외부 파라미터 σ와 base 예측 불확실성(풀 전체 자세 평균, mm)")
gls_out = {}
for label, idxs in (("pilot12", []), ("+8", chosen[:8])):
    Jc_set = np.concatenate([Jc_p] + [Jc_c[i:i + 1] for i in idxs]); Je_set = np.concatenate([Je_p] + [Je_c[i:i + 1] for i in idxs])
    for sb in (1e9, 5.0, 3.0, 1.0):
        cov9 = gls_cov(Jc_set, Je_set, sb); cc = cov9[:6, :6]; sd = np.sqrt(np.diag(cc)); u = pred_unc(cc, Jc_all)
        key = "%s_boardσ%s" % (label, "∞" if sb > 1e8 else ("%g" % sb)); gls_out[key] = {"rot_sigma_deg": np.degrees(sd[:3]).round(2).tolist(), "trans_sigma_mm": (sd[3:] * 1000).round(1).tolist(), "pred_mean_mm": round(float(u.mean()), 2)}
        print(key, gls_out[key])
d = json.load(open(ARGV[1])); d["gls_board_prior"] = gls_out; Path(ARGV[1]).write_text(json.dumps(d, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
