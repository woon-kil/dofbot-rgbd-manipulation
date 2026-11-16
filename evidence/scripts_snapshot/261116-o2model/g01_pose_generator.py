#!/usr/bin/env python3
"""b-4 보정 자세 후보 생성(오프라인, URDF 순기구학). 하드웨어 접속·이동 없음. 모든 기하는 모델 값이다(UNVERIFIED).

가정: q = SDK - 90 deg (이전 프로젝트 kinematics.py), 베이스 +y = 사용자 쪽, DaBai 광축 = Arm4 링크 +Y(영상 오른쪽 +X, 아래 -Z, 이전 프로젝트 db8_model_projection.py),
보드 중심·크기는 인자(기본 y 230 mm, 72x60 mm), 평면 z=0(작업대).
필터: J3,J4 >= 5, J1 75~105, 링크·DaBai·TCP 최저점 >= 최소 간격, 보드 중심 깊이 0.15~0.30 m(컬러 보정, 이전 프로젝트 DB-1 근거), 보드 모서리가 영상 가장자리에서 80 px 이상 안쪽(모델-실측 어긋남 약 60 px 여유), 보드 시야각 <= 40 deg.
"""
import json, math, sys, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
CLEAR_MM = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0     # 최소 간격(첫 접근 60, 일반 40)
BOARD_Y_MM = float(sys.argv[2]) if len(sys.argv) > 2 else 230.0   # 보드 중심: 베이스에서 +y(사용자 쪽) mm
BOARD_W_MM = float(sys.argv[3]) if len(sys.argv) > 3 else 72.0    # 보드 좌우 mm
BOARD_H_MM = float(sys.argv[4]) if len(sys.argv) > 4 else 60.0    # 보드 앞뒤 mm
J34_MIN = int(sys.argv[5]) if len(sys.argv) > 5 else 5            # J3,J4 하한(SDK). 5는 이전 프로젝트의 미검증 후보값(J4 None 사고 후)이라 더 큰 값도 비교
N_SELECT = 20
DEPTH_MIN = 0.15      # 보정은 컬러만 쓴다. 이전 프로젝트에서 작업대 122~171 mm에서도 컬러 마커가 보였고(DB-1), 깊이 작동 거리 하한 0.15 m(Yahboom 표)와 같다
DEPTH_MAX = 0.30
K = (479.07, 478.98, 315.07, 237.41); W, H = 640, 480; MARGIN_PX = 80
BOARD_C = np.array([0.0, BOARD_Y_MM/1000, 0.003]); BOARD_HALF = (BOARD_W_MM/2000, BOARD_H_MM/2000)

def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])
def rot(axis, q):   # q: (N,) -> (N,3,3)
    x, y, z = axis; c, s = np.cos(q), np.sin(q); t = 1 - c; R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = c+x*x*t; R[:, 0, 1] = x*y*t-z*s; R[:, 0, 2] = x*z*t+y*s
    R[:, 1, 0] = y*x*t+z*s; R[:, 1, 1] = c+y*y*t; R[:, 1, 2] = y*z*t-x*s
    R[:, 2, 0] = z*x*t-y*s; R[:, 2, 1] = z*y*t+x*s; R[:, 2, 2] = c+z*z*t; return R
J = {j.get('name'): j for j in ET.parse(ROOT/'data/refs/dofbot-pro-DOFBOT_Pro-V24.urdf').getroot().findall('joint')}
def jo(n):
    o = J[n].find('origin'); return np.array([float(v) for v in o.get('xyz').split()]), rpy(*[float(v) for v in o.get('rpy').split()])
def fk(sdk):   # sdk: (N,5) -> dict of positions/rotations (N,...)
    q = np.radians(sdk - 90.0); n = len(q); Rm = np.broadcast_to(np.eye(3), (n, 3, 3)).copy(); p = np.zeros((n, 3)); out = {}
    for i, name in enumerate(('Arm1_Joint', 'Arm2_Joint', 'Arm3_Joint', 'Arm4_Joint')):
        xyz, R0 = jo(name); p = p + Rm @ xyz; Rm = Rm @ R0
        ax = [float(v) for v in J[name].find('axis').get('xyz').split()]; Rm = Rm @ rot(ax, q[:, i]); out[name] = (p.copy(), Rm.copy())
    xyz, R0 = jo('DaBai_DCW2_Joint'); out['dabai'] = (p + Rm @ xyz, Rm @ R0)
    xyz, R0 = jo('Arm5_Joint'); p5 = p + Rm @ xyz; R5 = Rm @ R0 @ rot([0, 1, 0], q[:, 4])
    xyz, _ = jo('Gripping_point_Joint'); out['tcp'] = (p5 + R5 @ xyz, R5); out['arm5'] = (p5, R5); return out

# 후보 격자
J1 = np.arange(75, 106, 5); J2 = np.arange(30, 151, 5); J3 = np.arange(J34_MIN, 176, 5); J4 = np.arange(J34_MIN, 176, 5); J5 = np.array([90])
g = np.array(np.meshgrid(J1, J2, J3, J4, J5, indexing='ij')).reshape(5, -1).T.astype(float)
f = fk(g); dp, dR = f['dabai']; tcp = f['tcp'][0]
# DaBai 본체 8모서리(링크 좌표: x 45, y 12.5, z 12.5 mm)로 최저점
box = np.array([[sx*0.045, sy*0.0125, sz*0.0125] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
zmin = np.min(dp[:, None, 2] + np.einsum('nij,kj->nki', dR, box)[:, :, 2], axis=1)
chain_z = np.minimum.reduce([f['Arm2_Joint'][0][:, 2], f['Arm3_Joint'][0][:, 2], f['Arm4_Joint'][0][:, 2], f['arm5'][0][:, 2], tcp[:, 2], zmin])
# 카메라 좌표: (xc, yc, zc) = (xl, -zl, yl)
def cam_points(Pw):   # Pw: (3,) 또는 (k,3) 세계점 -> (N,k,3)
    Pw = np.atleast_2d(Pw); rel = Pw[None, :, :] - dp[:, None, :]; loc = np.einsum('nji,nkj->nki', dR, rel)   # 링크 좌표
    return np.stack([loc[..., 0], -loc[..., 2], loc[..., 1]], -1)
corners = np.array([BOARD_C + np.array([sx*BOARD_HALF[0], sy*BOARD_HALF[1], 0]) for sx in (-1, 1) for sy in (-1, 1)])
pc = cam_points(corners); pcen = cam_points(BOARD_C)[:, 0, :]
zc = pc[..., 2]; ok_depth = (pcen[:, 2] >= DEPTH_MIN) & (pcen[:, 2] <= DEPTH_MAX) & np.all(zc > 0.05, axis=1)
u = K[0]*pc[..., 0]/np.where(zc > 0, zc, 1) + K[2]; v = K[1]*pc[..., 1]/np.where(zc > 0, zc, 1) + K[3]
ok_img = np.all((u >= MARGIN_PX) & (u <= W-MARGIN_PX) & (v >= MARGIN_PX) & (v <= H-MARGIN_PX), axis=1)
axis_w = np.einsum('nij,j->ni', dR, np.array([0, 1, 0]))      # 광축(세계)
view_ang = np.degrees(np.arccos(np.clip(-axis_w[:, 2], -1, 1)))   # 보드 법선(+z)과 광축(-z 쪽을 봄) 사이 각
ok_view = view_ang <= 40
ok_joint = (g[:, 2] >= J34_MIN) & (g[:, 3] >= J34_MIN) & (chain_z*1000 >= CLEAR_MM)
mask = ok_depth & ok_img & ok_view & ok_joint
def path_clearance_mm(target, steps=40):
    s = np.linspace(0, 1, steps)[:, None]; q = np.array([90.0]*5)[None, :]*(1-s) + target[None, :]*s
    ff = fk(q); dpp, dRR = ff['dabai']; bz = np.min(dpp[:, None, 2] + np.einsum('nij,kj->nki', dRR, box)[:, :, 2], axis=1)
    cz = np.minimum.reduce([ff['Arm2_Joint'][0][:, 2], ff['Arm3_Joint'][0][:, 2], ff['Arm4_Joint'][0][:, 2], ff['arm5'][0][:, 2], ff['tcp'][0][:, 2], bz])
    return float(cz.min()*1000)
cand = np.where(mask)[0]; pc_path = np.array([path_clearance_mm(g[i]) for i in cand])
keep = pc_path >= CLEAR_MM; print("경로(대기→자세 직선 보간) 최소 간격 >= %.0f mm 인 후보 %d / %d" % (CLEAR_MM, keep.sum(), len(cand)))
mask = np.zeros_like(mask); mask[cand[keep]] = True; path_min = dict(zip(cand.tolist(), pc_path.tolist()))
print("격자 %d, 후보(필터 통과) %d | 깊이 %d, 영상 %d, 시야각 %d, 관절·간격 %d" % (len(g), mask.sum(), ok_depth.sum(), (ok_depth & ok_img).sum(), (ok_depth & ok_img & ok_view).sum(), ok_joint.sum()))
idx = np.where(mask)[0]
if len(idx) == 0: sys.exit("후보 없음")
# 다양성 선택(최원점 샘플링): 카메라 위치(m), 광축, 영상 오른쪽 방향
right = np.einsum('nij,j->ni', dR, np.array([1, 0, 0]))
feat = np.concatenate([dp*4.0, axis_w, right], axis=1)[idx]
sel = [int(np.argmin(np.linalg.norm(feat-feat.mean(0), axis=1)))]
d = np.linalg.norm(feat-feat[sel[0]], axis=1)
for _ in range(N_SELECT-1):
    k = int(np.argmax(d)); sel.append(k); d = np.minimum(d, np.linalg.norm(feat-feat[k], axis=1))
S = idx[sel]
poses = []
for i in S:
    poses.append({"sdk_deg": [int(x) for x in g[i]], "dabai_pos_mm": [round(float(x)*1000, 1) for x in dp[i]], "board_depth_mm": round(float(pcen[i, 2])*1000), "view_angle_deg": round(float(view_ang[i]), 1),
                  "min_clearance_mm": round(float(chain_z[i])*1000, 1), "tcp_mm": [round(float(x)*1000, 1) for x in tcp[i]], "path_min_clearance_mm": round(path_min[int(i)], 1), "max_joint_change_deg": int(np.abs(g[i]-90).max()), "corner_px_min_margin": round(float(min(u[i].min(), v[i].min(), W-u[i].max(), H-v[i].max())), 1)})
# 다양성 지표: 상대 회전 벡터의 특이값 비
def rotvec(R):
    a = math.acos(max(-1, min(1, (np.trace(R)-1)/2)))
    return np.zeros(3) if a < 1e-9 else a/(2*math.sin(a))*np.array([R[2, 1]-R[1, 2], R[0, 2]-R[2, 0], R[1, 0]-R[0, 1]])
Rs = dR[S]; rels = np.array([rotvec(Rs[a].T @ Rs[b]) for a in range(len(S)) for b in range(a+1, len(S))])
sv = np.linalg.svd(rels, compute_uv=False)
print("선택 %d개 | 상대 회전 벡터 특이값 %s (비 s2/s1=%.2f, s3/s1=%.3f)" % (len(S), np.round(sv, 2), sv[1]/sv[0], sv[2]/sv[0]))
print("관절 범위(SDK) J1 %d~%d, J2 %d~%d, J3 %d~%d, J4 %d~%d" % tuple(x for k in range(1, 5) for x in (g[S, k-1].min(), g[S, k-1].max())))
print("DaBai-보드 깊이 %d~%d mm, 시야각 %.0f~%.0f deg, 최소 간격 %.0f~%.0f mm" % (min(p['board_depth_mm'] for p in poses), max(p['board_depth_mm'] for p in poses), min(p['view_angle_deg'] for p in poses), max(p['view_angle_deg'] for p in poses), min(p['min_clearance_mm'] for p in poses), max(p['min_clearance_mm'] for p in poses)))
# 파일럿 12 / holdout 8: 다양성 선택 순서에서 번갈아 holdout 지정(시드 고정, 이동 전 분리)
rng = np.random.default_rng(20261112); hold = set(rng.choice(len(S), 8, replace=False).tolist())
for i, p in enumerate(poses): p["id"] = i; p["split"] = "holdout" if i in hold else "pilot"
out = {"version": "v1-draft", "clear_mm": CLEAR_MM, "board": {"center_y_mm": BOARD_Y_MM, "w_mm": BOARD_W_MM, "h_mm": BOARD_H_MM}, "n_candidates": int(mask.sum()), "model_assumptions": "q=SDK-90, 베이스 +y 사용자 쪽, DaBai 광축 Arm4 +Y, 보드 중심 (0, BOARD_Y, 3) mm, 모델값 UNVERIFIED", "rotation_svd": [float(x) for x in sv], "poses": poses}
Path(ROOT/f'evidence/G01/b4_pose_candidates_y{int(BOARD_Y_MM)}_{int(BOARD_W_MM)}x{int(BOARD_H_MM)}_j34min{J34_MIN}_clear{int(CLEAR_MM)}.json').write_text(json.dumps(out, ensure_ascii=False, indent=1)+"\n", encoding='utf-8')
for p in poses[:6]: print(p["id"], p["split"], p["sdk_deg"], "깊이", p["board_depth_mm"], "각", p["view_angle_deg"], "간격", p["min_clearance_mm"], "TCP", p["tcp_mm"])
