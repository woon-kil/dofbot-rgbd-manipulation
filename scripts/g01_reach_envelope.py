#!/usr/bin/env python3
"""URDF 순기구학으로 TCP 도달 포락선(관절 한계 안)과 Gemini 마스크의 실제 크기·베이스 상대 거리를 계산한다.
실물 접속·이동 없음. 입력: data/refs/dofbot-pro-DOFBOT_Pro-V24.urdf, evidence/G01/raw/gemini/registered/depth_mm.npy
"""
import json, math, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]

def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])

def axis_rot(ax, q):
    ax = np.asarray(ax, float); ax = ax/np.linalg.norm(ax); x, y, z = ax; c, s = math.cos(q), math.sin(q); C = 1-c
    return np.array([[c+x*x*C, x*y*C-z*s, x*z*C+y*s], [y*x*C+z*s, c+y*y*C, y*z*C-x*s], [z*x*C-y*s, z*y*C+x*s, c+z*z*C]])

def load():
    root = ET.parse(ROOT/'data/refs/dofbot-pro-DOFBOT_Pro-V24.urdf').getroot()
    J = {j.get('name'): j for j in root.findall('joint')}
    def get(n):
        j = J[n]; o = j.find('origin'); xyz = [float(v) for v in o.get('xyz').split()]; rp = [float(v) for v in o.get('rpy').split()]
        a = j.find('axis'); l = j.find('limit')
        return {'xyz': np.array(xyz), 'R': rpy(*rp), 'axis': [float(v) for v in a.get('xyz').split()] if a is not None else None,
                'lim': (float(l.get('lower')), float(l.get('upper'))) if l is not None else None}
    return {n: get(n) for n in ('Arm1_Joint', 'Arm2_Joint', 'Arm3_Joint', 'Arm4_Joint', 'Arm5_Joint', 'Gripping_point_Joint')}

def fk(jt, q):
    """q = [q1..q5] -> TCP 위치(3), 접근 벡터(Arm5 +y, base 좌표), 중간 링크 위치들"""
    T = np.eye(4); pts = []
    for name, qi in zip(('Arm1_Joint', 'Arm2_Joint', 'Arm3_Joint', 'Arm4_Joint', 'Arm5_Joint'), q):
        j = jt[name]; To = np.eye(4); To[:3, :3] = j['R']; To[:3, 3] = j['xyz']
        Tq = np.eye(4); Tq[:3, :3] = axis_rot(j['axis'], qi)
        T = T @ To @ Tq; pts.append(T[:3, 3].copy())
    j = jt['Gripping_point_Joint']; To = np.eye(4); To[:3, 3] = j['xyz']; T = T @ To
    return T[:3, 3], T[:3, :3] @ np.array([0, 1, 0]), pts

def main():
    jt = load()
    tcp0, app0, _ = fk(jt, [0, 0, 0, 0, 0])
    print("직립(모든 관절 0) TCP:", np.round(tcp0*1000, 1), "mm, 접근 벡터:", np.round(app0, 3))
    # 평면 사슬 q2,q3,q4 (q1=0) 격자. 위치는 q1·q5와 독립이므로 평면 포락선이다.
    deg = np.deg2rad(np.arange(-90, 91, 1))
    L = {k: jt[k] for k in ('Arm2_Joint', 'Arm3_Joint', 'Arm4_Joint')}
    # 벡터화 순기구학: q1=q5=0
    Q2, Q3, Q4 = np.meshgrid(deg, deg, deg, indexing='ij')
    n = Q2.size; q2, q3, q4 = Q2.ravel(), Q3.ravel(), Q4.ravel()
    def rx(q):
        c, s = np.cos(q), np.sin(q); R = np.zeros((q.size, 3, 3)); R[:, 0, 0] = 1; R[:, 1, 1] = c; R[:, 1, 2] = -s; R[:, 2, 1] = s; R[:, 2, 2] = c; return R
    def mats(name, q):
        j = jt[name]; R = np.einsum('ij,njk->nik', j['R'], rx(q)); return R, j['xyz']
    # 베이스 → Arm1(q1=0) 고정 오프셋
    p = np.tile(jt['Arm1_Joint']['xyz'], (n, 1)); R = np.tile(np.eye(3), (n, 1, 1))
    elbows = []
    for name, q in (('Arm2_Joint', q2), ('Arm3_Joint', q3), ('Arm4_Joint', q4)):
        j = jt[name]; p = p + np.einsum('nij,j->ni', R, j['xyz']); R = np.einsum('nij,njk->nik', R, np.einsum('ij,njk->nik', j['R'], rx(q))); elbows.append(p.copy())
    j5 = jt['Arm5_Joint']; p = p + np.einsum('nij,j->ni', R, j5['xyz']); wrist = p.copy()
    g = jt['Gripping_point_Joint']; tcp = p + np.einsum('nij,j->ni', R, g['xyz']); app = R[:, :, 1]  # Arm5 y축 = 접근 방향 (q5는 y축 회전이라 불변)
    s = tcp[:, 1]; z = tcp[:, 2]   # 평면 좌표: s=전방(+y), z=높이 (q1=0)
    # 검증: 직립 TCP와 일치
    i0 = np.argmin(np.abs(q2)+np.abs(q3)+np.abs(q4)); print("격자 검증(0,0,0) TCP:", np.round(tcp[i0]*1000, 1), "mm")
    tilt = np.degrees(np.arccos(np.clip(-app[:, 2], -1, 1)))   # 접근 벡터와 아래(-z) 사이 각
    # 간이 지면 제약: 팔꿈치·손목·TCP가 지면(z=0) 위
    above = (tcp[:, 2] >= 0) & (wrist[:, 2] >= 0.01) & (elbows[1][:, 2] >= 0.01) & (elbows[2][:, 2] >= 0.01)
    out = {"upright_tcp_mm": [round(v*1000, 1) for v in tcp0], "n_grid": int(n), "joint_limits_rad": {k: v['lim'] for k, v in jt.items() if v['lim']}, "assumption": "베이스 링크 원점 z=0을 작업대면으로 가정(R2 미검증). 간이 지면 제약(팔꿈치·손목·TCP가 0 이상), 충돌 모델 아님."}
    print("전체 TCP 범위: s %.0f~%.0f mm, z %.0f~%.0f mm (지면 위만: s %.0f~%.0f, z %.0f~%.0f)" % (s.min()*1e3, s.max()*1e3, z.min()*1e3, z.max()*1e3, s[above].min()*1e3, s[above].max()*1e3, z[above].min()*1e3, z[above].max()*1e3))
    rows = []
    for tmax in (15, 30, 45, 180):
        for (z0, z1) in ((0.010, 0.030), (0.030, 0.060), (0.060, 0.100), (0.100, 0.200)):
            m = above & (tilt <= tmax) & (z >= z0) & (z < z1)
            if m.sum() == 0: rows.append((tmax, z0, z1, None, None, 0)); continue
            ss = s[m]; rows.append((tmax, z0, z1, ss.min(), ss.max(), int(m.sum())))
    print("접근 기울기 한도별, TCP 높이대별 도달 가능한 전방 거리 s (mm). s<0은 베이스 뒤쪽(Arm1 반대편)")
    print("tilt<=  z대(mm)      s_min   s_max   표본")
    tab = []
    for tmax, z0, z1, a, b, c in rows:
        print("%4d    %3d-%3d   %s %s %7d" % (tmax, z0*1e3, z1*1e3, "%7.0f" % (a*1e3) if a is not None else "   없음", "%7.0f" % (b*1e3) if b is not None else "   없음", c))
        tab.append({"tilt_max_deg": tmax, "z_range_mm": [round(z0*1e3), round(z1*1e3)], "s_min_mm": None if a is None else round(a*1e3), "s_max_mm": None if b is None else round(b*1e3), "n": c})
    out["envelope_table"] = tab
    # 팔 최대 전방 도달(지면 위, 접근 제약 없음)
    out["max_forward_reach_mm_any_tilt_above_ground"] = round(float(s[above].max())*1e3)
    out["max_reach_to_table_z_le_30mm_any_tilt_mm"] = round(float(s[above & (z <= 0.03)].max())*1e3)
    # ---- 마스크 실제 크기와 베이스 상대 위치 ----
    d = np.load(ROOT/'evidence/G01/raw/gemini/registered/depth_mm.npy').astype(float)/1000.0
    fx, fy, cx, cy = 610.51, 610.35, 641.79, 364.44
    x0, x1, y0, y1 = 550, 830, 190, 395
    vv, uu = np.mgrid[y0:y1, x0:x1]; dd = d[y0:y1, x0:x1]; ok = (dd > 0) & (dd < 3)
    P = np.stack([(uu[ok]-cx)*dd[ok]/fx, (vv[ok]-cy)*dd[ok]/fy, dd[ok]], 1)
    idx = np.random.default_rng(0).choice(len(P), min(5000, len(P)), replace=False); Ps = P[idx]
    c0 = Ps.mean(0); u, sv, vt = np.linalg.svd(Ps-c0, full_matrices=False); nrm = vt[2]
    if nrm[2] > 0: nrm = -nrm
    res = np.abs((P-c0)@nrm)
    print("평면 맞춤(마스크 영역): 표본 %d, 잔차 중앙값 %.2f mm, p95 %.2f mm, 광축-법선 각 %.1f deg" % (len(Ps), np.median(res)*1e3, np.percentile(res, 95)*1e3, math.degrees(math.acos(abs(nrm[2])))))
    def hit(u_, v_):
        ray = np.array([(u_-cx)/fx, (v_-cy)/fy, 1.0]); t = (c0@nrm)/(ray@nrm); return ray*t
    corners = {"좌상": hit(x0, y0), "우상": hit(x1, y0), "우하": hit(x1, y1), "좌하": hit(x0, y1)}
    e1 = corners["우상"]-corners["좌상"]; e2 = corners["좌하"]-corners["좌상"]
    print("마스크 실제 크기: 가로 %.0f mm, 세로 %.0f mm (평면 위 거리)" % (np.linalg.norm(e1)*1e3, np.linalg.norm(e2)*1e3))
    # 베이스 중심(영상에서 눈으로 추정한 픽셀, 불확실 ±20 px): 지면 평면 위 점으로 투영
    base = hit(690, 175)
    ex = (corners["우상"]-corners["좌상"]); ex /= np.linalg.norm(ex); ey = np.cross(nrm, ex)
    out["mask_size_mm"] = [round(float(np.linalg.norm(e1))*1e3), round(float(np.linalg.norm(e2))*1e3)]
    out["plane_fit"] = {"n": len(Ps), "residual_median_mm": round(float(np.median(res))*1e3, 2), "residual_p95_mm": round(float(np.percentile(res, 95))*1e3, 2)}
    rr = {}
    for k, c in corners.items():
        dv = c-base; h = np.linalg.norm(dv - (dv@nrm)*nrm); rr[k] = round(float(h)*1e3)
    # 베이스 불확실성 ±20px 반영: 베이스 픽셀을 흔들어 최대·최소
    spread = []
    for du in (-20, 0, 20):
        for dv_ in (-20, 0, 20):
            b = hit(690+du, 175+dv_); spread.append([np.linalg.norm((c-b)-((c-b)@nrm)*nrm)*1e3 for c in corners.values()])
    spread = np.array(spread); out["mask_corner_distance_from_base_mm"] = {"nominal": rr, "min_over_base_uncertainty": [round(float(v)) for v in spread.min(0)], "max_over_base_uncertainty": [round(float(v)) for v in spread.max(0)], "corner_order": list(corners.keys()), "base_pixel_assumed": [690, 175], "base_pixel_uncertainty_px": 20}
    # 마스크 모서리 중 가장 먼 점과 도달 한계 비교
    print("마스크 모서리의 베이스까지 수평 거리 (mm, 베이스 픽셀 추정 ±20px):", {k: (rr[k], int(spread.min(0)[i]), int(spread.max(0)[i])) for i, k in enumerate(corners)})
    Path(ROOT/'evidence/G01/reach_envelope_urdf.json').write_text(json.dumps(out, ensure_ascii=False, indent=2)+"\n", encoding='utf-8')
if __name__ == "__main__": main()
