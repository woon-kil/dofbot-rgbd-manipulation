#!/usr/bin/env python3
"""G04 확증 자세 설계 + 합격선 시뮬레이션(오프라인, 동결 v1.2 M4). 동결된 풀이(`g04_solve.py`)의 해·공분산을 입력으로 한다.
① 후보 풀 = 파일럿·S4-4 학습·holdout·점검 제외 ② 해의 카메라 모델로 시야 여유 ≥40 px ③ 학습/holdout/점검 자세에서 가장 먼 순서로 8개 선택
④ 예측 오차(동결된 해로 구한 보드 xy 대 실측 (0,229), 실측 σ1.5 포함) 분포 시뮬레이션 → 합격선 = 95% × 1.5 (0.5 mm 올림). 사용: python3 g04_confirm_design.py <out.json>"""
import io, json, sys, contextlib
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
OUT = sys.argv[1]
# ① 동결된 풀이 재실행(출력 억제)해 해·공분산 확보
src = open("scripts/g04_solve.py", encoding="utf-8").read(); ns = {"__file__": "scripts/g04_solve.py"}; sys.argv = ["g04_solve.py", "/tmp/_solve_tmp.json"]
with contextlib.redirect_stdout(io.StringIO()): exec(compile(src, "g04_solve", "exec"), ns)
x_star = ns["results"]["공칭"]["x"]; resid = ns["resid"]; scale = ns["scale"]; s = least_squares(resid, x_star, x_scale=scale); chi2 = float(np.sum(resid(s.x) ** 2) / max(len(resid(s.x)) - len(s.x), 1))
cov = np.linalg.pinv(s.jac.T @ s.jac) * max(chi2, 1.0); dx = np.abs(s.x - x_star); print("재적합 이동량 최대: 회전/이동 %.2e (rad,m), J1 영점 %.3f°, 보드 %.3f mm" % (dx[:6].max(), dx[6], dx[7:].max())); assert dx[:6].max() < 1e-3 and dx[6] < 0.05 and dx[7:].max() < 0.1, "해가 재현되지 않음"
m = ns["m"]; K = ns["K"]; R_LC = ns["R_LC"]; BXY = ns["BXY"]; cv2 = ns["cv2"]; data = ns["data"]
Rbar_list = [Rot.from_matrix(cv2.Rodrigues(d["rvec"])[0]) for d in data]
# ② 후보 풀: g04_pose_design 헤드
import os
os.environ.pop("G4_BX", None); head = open("/tmp/g04_head.py", encoding="utf-8").read(); ns2 = {"__file__": "scripts/g04_pose_design.py"}; sys.argv = ["x", "/tmp/_unused.json"]
with contextlib.redirect_stdout(io.StringIO()): exec(compile(head, "head", "exec"), ns2)
pool = ns2["pool"]
cand = json.load(open("evidence/G01/b4_pose_candidates_y230_72x60_j34min10_clear60.json"))["poses"]; s44 = json.load(open("evidence/G02/s4_4_poses.json"))["poses"]
used = {tuple(c["sdk_deg"][:4]) for c in cand} | {tuple(p["sdk_deg"][:4]) for p in s44}   # 파일럿 12·holdout 8·점검·S4-4 전부(후보 JSON 20 + S4-4 8)
pool = np.array([q for q in pool if tuple(int(v) for v in q) not in used], float); print("확증 후보 풀(사용 자세 제외):", len(pool))
def margin(q4, x):
    qq = np.array([q4], float); qq[:, 0] += x[6]; f = m.fk(np.hstack([qq, [[90.0]]])); dp, dR = f["dabai"]; Rd = Rot.from_rotvec(x[:3]).as_matrix(); Rc = dR[0] @ Rd @ R_LC; tc = dp[0] + dR[0] @ x[3:6]
    cs = np.array([[0.0 + sx * 0.036, BXY[1] + sy * 0.030, x[9] / 1000] for sx in (-1, 1) for sy in (-1, 1)]); pc = (Rc.T @ (cs - tc).T).T
    if np.any(pc[:, 2] <= 0.02): return -999.0
    u = K[0, 0] * pc[:, 0] / pc[:, 2] + K[0, 2]; v = K[1, 1] * pc[:, 1] / pc[:, 2] + K[1, 2]; return float(min(u.min(), 640 - u.max(), v.min(), 480 - v.max()))
ok = np.array([margin(q, x_star) >= 40 for q in pool]); pool = pool[ok]; print("해 카메라 모델로 시야 여유 ≥40 px 통과:", len(pool))
used_q = np.array([list(u) for u in used], float)
# ③ 가장 먼 순서 선택(학습/holdout/점검과 이미 선택된 확증 자세로부터)
chosen = []; ref = used_q.copy()
for _ in range(8):
    dmin = np.array([np.min(np.linalg.norm(ref - q, axis=1)) for q in pool]); dmin[[i for i in chosen]] = -1; i = int(np.argmax(dmin)); chosen.append(i); ref = np.vstack([ref, pool[i]])
conf = pool[chosen]; print("확증 자세(J1,J2,J3,J4):", conf.astype(int).tolist(), "최소 거리(°):", [round(float(np.min(np.linalg.norm(used_q - q, axis=1))), 1) for q in conf])
# ④ 예측 오차 시뮬레이션
SIG = ns["SIG"] if "SIG" in ns else np.array([0.0, 0.122, 0.0, 0.023]); rng = np.random.default_rng(11); N = 400
chol = np.linalg.cholesky(cov + 1e-14 * np.eye(len(x_star)))
def fkcam(q, x):
    qq = np.array(q, float).copy(); qq[:, 0] += x[6]; f = m.fk(np.hstack([qq, np.full((len(qq), 1), 90.0)])); dp, dR = f["dabai"]; Rd = Rot.from_matrix(Rot.from_rotvec(x[:3]).as_matrix()).as_matrix()
    return dR @ Rd @ R_LC, dp + dR @ x[3:6]
Rb_true = Rot.from_matrix(np.mean([r.as_matrix() for r in Rbar_list], 0)).as_matrix(); U, _, Vt = np.linalg.svd(Rb_true); Rb_true = U @ Vt
errs = []   # (N, n_conf) xy 오차 크기
for _ in range(N):
    xt = x_star + chol @ rng.standard_normal(len(x_star)); bt = np.array([xt[7], xt[8], xt[9]]) / 1000; b_meas = np.array([BXY[0] + rng.normal(0, 0.0015), BXY[1] + rng.normal(0, 0.0015)])
    qr = conf + 0.0   # 회독 = 지령 정수
    qt = qr + rng.uniform(-0.5, 0.5, conf.shape) + rng.normal(0, SIG, conf.shape)   # 참 관절각
    Rc_t, tc_t = fkcam(qt, xt)   # 참 카메라(참 파라미터, 참 관절)
    P_pred = []
    Rc_s, tc_s = fkcam(qr, x_star)   # 동결된 해 + 회독
    for i in range(len(conf)):
        tcb = Rc_t[i].T @ (bt - tc_t[i])   # PnP가 주는 카메라 좌표 위치(보드 중심, 완벽한 PnP + 0.5 mm 노이즈)
        tcb = tcb + rng.normal(0, 0.0005, 3); P_pred.append(Rc_s[i] @ tcb + tc_s[i])
    P_pred = np.array(P_pred); errs.append(np.linalg.norm(P_pred[:, :2] - b_meas, axis=1) * 1000)
errs = np.array(errs); mean_pose = errs.mean(1); max_pose = errs.max(1)
th = {"xy_error_mean_mm": {"p95": round(float(np.percentile(mean_pose, 95)), 2)}, "xy_error_max_mm": {"p95": round(float(np.percentile(max_pose, 95)), 2)}}
for k in th: th[k]["threshold_1p5x"] = round(float(np.ceil(th[k]["p95"] * 1.5 * 2) / 2), 1)
print("예측 xy 오차 분포(자세 평균/최대):", {k: (v["p95"], v["threshold_1p5x"]) for k, v in th.items()}, "| 평균 %.2f / 최대 %.2f mm" % (mean_pose.mean(), max_pose.mean()))
# 5단위: s4_5 자세 DB
db = {"version": "s4-5-v1", "note": "G04 확증 자세(취득 전 설계, 해 동결 후)", "poses": [{"id": 200 + i, "sdk_deg": [int(v) for v in q] + [90], "split": "g04_confirm", "source": "g04_confirm_design 최원거리 선택"} for i, q in enumerate(conf)]}
Path("evidence/G02/s4_5_poses.json").write_text(json.dumps(db, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
Path(OUT).write_text(json.dumps({"n_conf": len(conf), "confirm_poses": conf.astype(int).tolist(), "pool_after_filters": len(pool), "sim_draws": N, "thresholds": th, "model": "해 공분산에서 참 파라미터 추출, 회독 반올림+흔들림, PnP 0.5 mm, 보드 실측 σ1.5 mm, 지표 = 보드 xy 예측 오차 자세 평균/최대"}, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
