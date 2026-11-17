#!/usr/bin/env python3
"""G04 진단(파일럿 12자세만, 보정 아님): 실측 보드 중심 위치를 알려진 값으로 두고 카메라 외부 파라미터 6자유도를 적합해 공칭 모델과의 불일치 크기를 본다.
보드 중심 (x,y) = 실측, z = 미지(URDF base_link z=0 대응 UNVERIFIED). 사용: python3 g04_board_prior_diag.py"""
import json, sys, importlib.util
import numpy as np, cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
sys.path.insert(0, "scripts"); spec = importlib.util.spec_from_file_location("s3", "scripts/g01_s3_path.py"); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
R_LC = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0.0]]); pil = json.load(open("evidence/G02/pilot_pnp.json")); qp = np.array([r["q"] for r in pil])
meas = json.load(open("evidence/G02/board_measurement.json"))["derived_base_frame"]["board_center_xy_mm"]; bx, by = meas[0] / 1000, meas[1] / 1000
def centers(c, yaw_j1=0.0):
    out = []
    for r, q in zip(pil, qp):
        qq = np.array(q, float); qq[0] += yaw_j1; dp, dR = m.fk(np.array([[*qq, 90.0]]))["dabai"]; Rd = Rot.from_rotvec(c[:3]).as_matrix()
        Rc = dR[0] @ Rd @ R_LC; tc = dp[0] + dR[0] @ c[3:]; Rb = cv2.Rodrigues(np.array(r["rvec"]))[0]
        out.append(Rc @ (Rb @ np.array([0.036, 0.030, 0.0]) + np.array(r["tvec"])) + tc)
    return np.array(out) * 1000
P0 = centers(np.zeros(6)); print("공칭 FK 보드 중심 평균(mm):", P0.mean(0).round(1), "측정 (x,y):", meas, "→ 차이(측정−공칭) x,y:", (np.array(meas) - P0.mean(0)[:2]).round(1))
def fit(label, free):
    def res(p):
        c = np.zeros(6); zb = p[0]; k = 1
        for i in free: c[i] = p[k]; k += 1
        P = centers(c); return np.concatenate([(P[:, 0] - bx * 1000), (P[:, 1] - by * 1000), (P[:, 2] - zb)])
    s = least_squares(res, np.concatenate([[P0[:, 2].mean()], np.zeros(len(free))]), x_scale=[5.0] + [0.01 if i < 3 else 0.005 for i in free])
    r = res(s.x); xy = np.sqrt(np.mean(np.concatenate([r[:12], r[12:24]]) ** 2)); c = {}; k = 1
    for i in free: c[["rx", "ry", "rz", "tx", "ty", "tz"][i]] = round(float(np.degrees(s.x[k]) if i < 3 else s.x[k] * 1000), 2); k += 1
    print(label, "보드 z(mm)=%.1f" % s.x[0], "보정(회전°/이동mm)", c, "xy 잔차 RMS %.2f mm, 전체 RMS %.2f mm" % (xy, np.sqrt(np.mean(r ** 2))))
fit("(보정 없음, z만 자유)", [])
fit("카메라 이동 3개", [3, 4, 5]); fit("카메라 회전 3개", [0, 1, 2]); fit("카메라 6자유도", [0, 1, 2, 3, 4, 5])
