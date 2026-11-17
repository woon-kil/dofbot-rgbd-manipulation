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

# ---- v2: 실측 보드 위치 prior + J1 영점 오프셋 미지수(7번째 파라미터) 포함 설계 ----
def gls7(Jc_set, Je_set, sig_b_xy=1.5, sig_b_z=10.0, sig_pnp=0.5):
    n = len(Jc_set); H = np.zeros((n * 3 + 3, 10)); W = np.zeros((n * 3 + 3, n * 3 + 3)); Sig_e = np.diag(SIG ** 2)
    for i in range(n):
        H[i * 3:i * 3 + 3, :6] = Jc_set[i][:3, :]; H[i * 3:i * 3 + 3, 6] = Je_set[i][:3, 0]; H[i * 3:i * 3 + 3, 7:10] = -np.eye(3)
        S = Je_set[i][:3] @ Sig_e @ Je_set[i][:3].T + np.eye(3) * sig_pnp ** 2; W[i * 3:i * 3 + 3, i * 3:i * 3 + 3] = np.linalg.inv(S)
    H[n * 3:, 7:10] = np.eye(3); W[n * 3:, n * 3:] = np.diag([1 / sig_b_xy ** 2, 1 / sig_b_xy ** 2, 1 / sig_b_z ** 2])
    return np.linalg.inv(H.T @ W @ H + 1e-12 * np.eye(10))[:7, :7]
Jc_all, Je_all = jac(pool)
def pred7(cov7):
    Jp = np.concatenate([Jc_all[:, :3, :], Je_all[:, :3, :1]], axis=2); return np.sqrt(np.einsum("nik,kl,nil->n", Jp, cov7, Jp))
def report(label, idxs):
    Jc_set = np.concatenate([Jc_p] + [Jc_c[i:i + 1] for i in idxs]); Je_set = np.concatenate([Je_p] + [Je_c[i:i + 1] for i in idxs]); cov = gls7(Jc_set, Je_set); sd = np.sqrt(np.diag(cov)); u = pred7(cov)
    r = {"rot_sigma_deg": np.degrees(sd[:3]).round(2).tolist(), "trans_sigma_mm": (sd[3:6] * 1000).round(1).tolist(), "j1_zero_sigma_deg": round(float(sd[6]), 2), "pred_mean_mm": round(float(u.mean()), 2)}; print(label, r); return r, u.mean()
out = {"pilot12": report("pilot12", [])[0]}
chosen7 = []
for step in range(8):
    best = None
    for idx in range(len(pool)):
        if idx in chosen7: continue
        Jc_set = np.concatenate([Jc_p] + [Jc_c[i:i + 1] for i in chosen7 + [idx]]); Je_set = np.concatenate([Je_p] + [Je_c[i:i + 1] for i in chosen7 + [idx]])
        v = pred7(gls7(Jc_set, Je_set)).mean()
        if best is None or v < best[0]: best = (v, idx)
    chosen7.append(best[1]); out["+%d" % (step + 1)] = report("+%d" % (step + 1), chosen7)[0]; out["+%d" % (step + 1)]["pose"] = pool[best[1]].astype(int).tolist()
Path(ARGV[1]).write_text(json.dumps({"model": "외부 6자유도 + J1 영점 오프셋, 보드 xy 실측 prior σ1.5 mm, z σ10 mm", "result": out}, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
