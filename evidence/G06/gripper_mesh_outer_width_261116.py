#!/usr/bin/env python3
"""URDF 그리퍼 링크 메시(data/refs/dofbot_meshes)로 grip_joint별 바깥 폭·손끝 y 최대를 계산한다(오프라인, UNVERIFIED: grip↔J6 대응·캡(고무) 미포함).
실행: .venv/bin/python -I evidence/G06/gripper_mesh_outer_width_261116.py   (프로젝트 루트에서)
arm5 프레임: x = 열림 축, y = 손가락 축. mimic 부호는 URDF(rlink_joint2 +1, llink_joint1 +1, llink_joint2 −1, rlink_joint3 −1, llink_joint3 +1; grip_joint 축 −z)."""
import numpy as np, struct, re
def stl(p):
    b = open(p, 'rb').read()
    if b[:5] == b'solid' and b'facet' in b[:400]: return np.array(re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', b), dtype=float)
    n = struct.unpack('<I', b[80:84])[0]; a = np.frombuffer(b[84:84+n*50], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return a['v'].reshape(-1, 3).astype(float)
M = {k: stl('data/refs/dofbot_meshes/%s.STL' % k) for k in ('rlink1', 'rlink2', 'rlink3', 'llink1', 'llink2', 'llink3')}
def Rz(a): c, s = np.cos(a), np.sin(a); R = np.eye(4); R[:2, :2] = [[c, -s], [s, c]]; return R
def T(xyz, yaw, ang): m = Rz(yaw) @ Rz(ang); m[:3, 3] = xyz; return m
def pts(g):
    r1 = T([0.012625, 0.029275, 0.00475], 0.42401, -g); r2 = r1 @ T([0.027229, -0.012593, -0.0025], 1.1468, g)
    l1 = T([-0.012375, 0.029275, 0.0045], 2.7292, g); l2 = l1 @ T([0.027362, 0.012302, -0.0025], -1.1584, -g)
    r3 = T([0.0045, 0.047275, 0.0045], 0.42401, -g); l3 = T([-0.0045, 0.047275, 0.0045], 2.7292, g)
    return {k: (np.c_[M[k], np.ones(len(M[k]))] @ Tm.T)[:, :3] for k, Tm in (('rlink1', r1), ('rlink2', r2), ('rlink3', r3), ('llink1', l1), ('llink2', l2), ('llink3', l3))}
print('grip(rad) | 바깥 x 범위(mm) | 폭 | 손끝 y 최대 | 안쪽 간격(rlink2 min x − llink2 max x)')
for g in (0, -0.4, -0.7, -0.8, -1.0, -1.2, -1.4, -1.58, -1.6):
    o = pts(g); a = np.vstack(list(o.values())); r2, l2 = o['rlink2'], o['llink2']
    print('%6.2f | %.1f ~ %.1f | %.1f | %.1f | %.1f' % (g, a[:, 0].min()*1e3, a[:, 0].max()*1e3, (a[:, 0].max()-a[:, 0].min())*1e3, a[:, 1].max()*1e3, (r2[:, 0].min()-l2[:, 0].max())*1e3))
