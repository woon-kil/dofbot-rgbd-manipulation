#!/usr/bin/env python3
"""S4-4 자세의 보드 시야 여유 점검(오프라인): 공칭 모델과 진단 적합 보정 2종(이동형·회전형, 보드 xy 실측 (0,229) mm)에서 보드 4모서리 픽셀을 예측해 영상 가장자리와의 최소 거리를 본다."""
import json, sys, importlib.util
import numpy as np
from scipy.spatial.transform import Rotation as Rot
sys.path.insert(0, "scripts"); spec = importlib.util.spec_from_file_location("s3", "scripts/g01_s3_path.py"); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
ci = json.load(open("evidence/G02/dabai_color_camera_info.json")); K = np.array(ci["K"]).reshape(3, 3); W, H = 640, 480
R_LC = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0.0]])
HYP = {"공칭": (np.zeros(6), 0.0048), "이동형(tx−18.8,ty−4.7,tz+4.8, 보드 z 11.2)": (np.array([0, 0, 0, -0.01877, -0.00473, 0.00482]), 0.0112), "회전형(rx .59,ry −2.47,rz 5.66°, 보드 z 5.1)": (np.concatenate([np.radians([0.59, -2.47, 5.66]), [0, 0, 0]]), 0.0051)}
def corners(zb): return np.array([[0.0 + sx * 0.036, 0.229 + sy * 0.030, zb] for sx in (-1, 1) for sy in (-1, 1)])
def margin(q4, c, zb):
    dp, dR = m.fk(np.array([[*q4, 90.0]]))["dabai"]; Rd = Rot.from_rotvec(c[:3]).as_matrix(); Rc = dR[0] @ Rd @ R_LC; tc = dp[0] + dR[0] @ c[3:]
    pc = (Rc.T @ (corners(zb) - tc).T).T
    if np.any(pc[:, 2] <= 0.02): return -999.0, None
    u = K[0, 0] * pc[:, 0] / pc[:, 2] + K[0, 2]; v = K[1, 1] * pc[:, 1] / pc[:, 2] + K[1, 2]
    return float(min(u.min(), W - u.max(), v.min(), H - v.max())), (u.round(0).tolist(), v.round(0).tolist())
poses = [json.loads(x) if isinstance(x, str) else x for x in json.load(open("evidence/G02/s4_4_poses.json"))["poses"]]
rows = [(p["id"], p["sdk_deg"][:4]) for p in poses] + [("점검7", [85, 95, 15, 10]), ("점검5", [90, 105, 10, 10])]
print("자세별 보드 모서리 최소 여유(px, 영상 가장자리까지)")
worst = {}
for pid, q in rows:
    out = []
    for h, (c, zb) in HYP.items():
        mg, _ = margin(q, c, zb); out.append(round(mg)); worst[h] = min(worst.get(h, 1e9), mg)
    print(pid, q, "공칭/이동형/회전형 =", out)
print("가설별 최소:", {k: round(v) for k, v in worst.items()})
