#!/usr/bin/env python3
"""G04 설계 v3(시뮬레이션): 보드를 두 번째 위치(x=+BX mm)에 놓고 그 위치도 실측(σ 1.5 mm)할 때, 카메라 외부 6자유도 + J1 영점 오프셋이 얼마나 분리되는가.
사용: python3 g04_pose_design3.py <out.json> [BX=50]"""
import os, sys, json, numpy as np
from pathlib import Path
OUT = sys.argv[1]; BX = sys.argv[2] if len(sys.argv) > 2 else "50"; head = open("/tmp/g04_head.py", encoding="utf-8").read()
sys.argv = ["x", "/tmp/_unused.json"]
os.environ["G4_BX"] = "0"; ns1 = {"__file__": "scripts/g04_pose_design.py"}; exec(compile(head, "head1", "exec"), ns1)
os.environ["G4_BX"] = BX; ns2 = {"__file__": "scripts/g04_pose_design.py"}; exec(compile(head, "head2", "exec"), ns2)
SIG = ns1["SIG"]
def gls(blocks, sig_b=1.5, sig_pnp=0.5, sig_bz=10.0):
    """blocks: [(Jc (n,6,6), Je (n,6,4)), ...] 보드마다 한 블록, 보드마다 위치 미지수(3) + prior. 파라미터: c(6), o1(1), b_k(3 each)."""
    nb = len(blocks); P = 7 + 3 * nb; rows = sum(len(b[0]) * 3 for b in blocks) + 3 * nb; H = np.zeros((rows, P)); W = np.zeros((rows, rows)); r = 0
    for k, (Jc, Je) in enumerate(blocks):
        for i in range(len(Jc)):
            H[r:r + 3, :6] = Jc[i][:3, :]; H[r:r + 3, 6] = Je[i][:3, 0]; H[r:r + 3, 7 + 3 * k:10 + 3 * k] = -np.eye(3)
            S = Je[i][:3] @ np.diag(SIG ** 2) @ Je[i][:3].T + np.eye(3) * sig_pnp ** 2; W[r:r + 3, r:r + 3] = np.linalg.inv(S); r += 3
    for k in range(nb):
        H[r:r + 3, 7 + 3 * k:10 + 3 * k] = np.eye(3); W[r:r + 3, r:r + 3] = np.diag([1 / sig_b ** 2, 1 / sig_b ** 2, 1 / sig_bz ** 2]); r += 3
    return np.linalg.inv(H.T @ W @ H + 1e-12 * np.eye(P))[:7, :7]
Jc_p, Je_p = ns1["Jc_p"], ns1["Je_p"]; pool2, Jc2, Je2 = ns2["pool"], ns2["Jc_c"], ns2["Je_c"]; pool1, Jc1, Je1 = ns1["pool"], ns1["Jc_c"], ns1["Je_c"]
Jall, Eall = ns1["jac"](pool1)
def pred(cov):
    Jp = np.concatenate([Jall[:, :3, :], Eall[:, :3, :1]], axis=2); return np.sqrt(np.einsum("nik,kl,nil->n", Jp, cov, Jp)).mean()
def rep(label, cov): sd = np.sqrt(np.diag(cov)); r = {"rot_sigma_deg": np.degrees(sd[:3]).round(2).tolist(), "trans_sigma_mm": (sd[3:6] * 1000).round(1).tolist(), "j1_zero_sigma_deg": round(float(sd[6]), 2), "pred_mean_mm": round(float(pred(cov)), 2)}; print(label, r); return r
res = {"pilot12_board1": rep("pilot12(보드1만)", gls([(Jc_p, Je_p)]))}
print("보드2 후보 풀 크기:", len(pool2))
chosen = []
for step in range(8):
    best = None
    for idx in range(len(pool2)):
        if idx in chosen: continue
        idxs = chosen + [idx]; cov = gls([(Jc_p, Je_p), (Jc2[idxs], Je2[idxs])]); v = pred(cov)
        if best is None or v < best[0]: best = (v, idx)
    chosen.append(best[1]); res["+%d(보드2)" % (step + 1)] = rep("보드1 파일럿12 + 보드2 %d자세" % (step + 1), gls([(Jc_p, Je_p), (Jc2[chosen], Je2[chosen])])); res["+%d(보드2)" % (step + 1)]["pose"] = pool2[best[1]].astype(int).tolist()
Path(OUT).write_text(json.dumps({"model": "외부 6자유도 + J1 영점, 보드1(x=0)·보드2(x=+%s) xy 실측 prior σ1.5, z σ10" % BX, "result": res}, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
