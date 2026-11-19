#!/usr/bin/env python3
"""Run 2 접촉 구간 오프라인 간격(UNVERIFIED): 계획 세트 29의 하강·상승 이동을 0.5° 간격으로 보며 열린 손가락(링크 메시 + 캡 상자, 캡 중심 간격 관측 61.8 mm)과 Arm5 본체 메시가 블록 상자(30x60x30)와 종이에 얼마나 가까운지 계산한다. plan_check의 convex hull은 열린 두 손가락 사이를 채워 블록이 사이에 있으면 늘 관통으로 나오므로(모델 한계) 이 구간 판정에 쓰지 않는다.
블록 위치는 A50 자세에서 캡 중점 예측 x에서 옆 오프셋 d_x(프로젝트 x, mm)만큼 뺀 값으로 놓는다(d_x 후보 −3/+1.3/+3). 높이 변형: dz 0, −3 mm(손가락·본체를 3 mm 낮춤, J3 도달 오차 등 보수).
사용: .venv/bin/python -I evidence/G06/run2_contact_clearance_261117.py <plan.json>"""
import sys, json
sys.path.insert(0,'evidence/G06'); sys.path.insert(0,'scripts'); sys.path.insert(0,'src')
import numpy as np, importlib.util, struct, re
import gemini_fingertip_detect_r15 as D
spec = importlib.util.spec_from_file_location('pa','evidence/G06/p2k_analyze_261117.py'); PA = importlib.util.module_from_spec(spec); spec.loader.exec_module(PA)
def stl(p):
    b=open(p,'rb').read()
    if b[:5]==b'solid' and b'facet' in b[:400]: return np.array(re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)',b),dtype=float)
    n=struct.unpack('<I',b[80:84])[0]; a=np.frombuffer(b[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')])); return a['v'].reshape(-1,3).astype(float)
ARM5 = stl('data/refs/dofbot_meshes/Arm5_Link.STL') * 1e3
G = -0.79; CAPCX = 61.8
_PC = {}
def points(q):
    k = tuple(round(float(x),4) for x in q)
    if k in _PC: return _PC[k]
    r = _points(q); _PC[k] = r; return r
def _points(q):
    o = D.fk.fk(np.array([list(q)+[90.0]])); t = o['tcp'][0][0]*1000; R = o['tcp'][1][0]; ch = PA.__dict__['_Tm']
    chain = {}
    r1 = ch([0.012625,0.029275,0.00475],0.42401,-G); r2 = r1@ch([0.027229,-0.012593,-0.0025],1.1468,G)
    l1 = ch([-0.012375,0.029275,0.0045],2.7292,G); l2 = l1@ch([0.027362,0.012302,-0.0025],-1.1584,-G)
    r3 = ch([0.0045,0.047275,0.0045],0.42401,-G); l3 = ch([-0.0045,0.047275,0.0045],2.7292,G)
    chn = {'rlink1':r1,'rlink2':r2,'rlink3':r3,'llink1':l1,'llink2':l2,'llink3':l3}; pts=[]; tags=[]
    for side in 'rl':
        P = np.vstack([(np.c_[PA.M[k],np.ones(len(PA.M[k]))]@chn[k].T)[:,:3] for k in chn if k[0]==side]); tip = P[np.argmax(P[:,1])]
        pts.append(P); tags += ['링크']*len(P)
        cx = CAPCX/2 if side=='r' else -CAPCX/2
        C = np.array([[cx+x, tip[1]+y, tip[2]+z] for x in np.linspace(-5,5,5) for y in np.linspace(-25,4,13) for z in np.linspace(-3,3,3)]); pts.append(C); tags += ['캡']*len(C)
    pts.append(ARM5); tags += ['Arm5']*len(ARM5)
    A = np.vstack(pts); W = t + (R @ np.c_[A[:,0], A[:,1]-90, A[:,2]].T).T
    return W, np.array(tags)
def clear(W, tags, c, half):
    d = np.abs(W-c)-half; out = np.maximum(d,0); dist = np.linalg.norm(out,axis=1); inside = (d<0).all(axis=1); v = np.where(inside, d.max(axis=1), dist); i = int(np.argmin(v)); return float(v[i]), tags[i], float(W[:,2].min())
plan = json.load(open(sys.argv[1])); start = [plan['start'][str(i)] for i in (1,2,3,4)]
cmds = [(m['joint'], m['target'], m.get('pose_id')) for m in plan['moves']]
# 상태: 계획 이동을 순서대로 적용, J1은 도착 원시가 명령보다 0.4° 모자라게 선다고 가정해 +... (감소 방향이므로 +0.4)
cur = list(map(float,start)); states = []; labels = []
for (j,t,pid) in cmds:
    tt = float(t) + (0.4 if j == 1 and t < cur[0] else 0.0)
    for f in np.linspace(0,1,max(2,int(np.ceil(abs(tt-cur[j-1])/0.5))+1)):
        q = list(cur); q[j-1] = cur[j-1] + (tt-cur[j-1])*f; states.append(q); labels.append((j,t,pid))
    cur[j-1] = tt
# A50 자세(2901 도착) 이후 상태만 평가
_first_pid = int(list(plan['poses'])[0]); first = next(i for i,l in enumerate(labels) if l[2] == _first_pid and l[0] == 2)
ret = [ (r['after_move'], r['state'], r['route']) for r in plan['return_routes']]
# 복귀 경로도 추가(마지막 정상 상태 이후 route)
last_route = plan['return_routes'][-1]
cur2 = list(map(float, plan['return_routes'][-1]['state']))
import os as _os
half = np.array([15.0 + float(_os.environ.get('RUN2_BOX_XPAD_MM', '0')), 30.0, 15.0])   # RUN2_BOX_XPAD_MM: 블록 yaw 불확실(±2° 해상도, 손가락 y 어긋남 ≤ 8 mm → 약 0.3 mm)을 덮는 x 방향 상자 확장(기본 0)
import sys as _sys; _sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', '..', 'scripts')); import g01_lift_marker_ref as _LMR
_J1A = _LMR.j1_raw_for_plan(plan)   # 2026-11-19: 세트 번호가 아니라 계획의 접촉 자세 J1 명령으로 정한다(J1 85 계획 → 85.4, J1 86 계획 → 86.4, 세트 31 → 86.4, 그 밖 → 87.4). 이전에는 세트 33이면 항상 86.4라 J1 85 계획이 3 mm 어긋나 계산됐다
xa = PA.caps_world([_J1A,81.0,16.28,19.31], 100)[0]   # A50 자세 J6 100 캡 중점 x(프로젝트)
# 중단 복귀 경로(열린 손가락 상태로 하강 중 중단): 상태 J1 ≤ 88(J1 보정 뒤)인 복귀 경로를 모두 평가
abort_states = []
for r in plan['return_routes']:
    st0 = list(map(float, r['state']))
    if st0[0] > 88.5: continue
    curr = list(st0); 
    for m in r['route']:
        j, t = m['joint'], float(m['target'])
        for f in np.linspace(0,1,max(2,int(np.ceil(abs(t-curr[j-1])/0.5))+1)):
            q = list(curr); q[j-1] = curr[j-1] + (t-curr[j-1])*f; abort_states.append((q, ('복귀', r['after_move'], j, t)))
        curr[j-1] = t
print('※ 오프라인 UNVERIFIED, 단위 mm. 간격 = 손가락(링크+캡)·Arm5 본체와 블록 상자 사이 최소 거리(음수 = 관통).')
import os
GATE = float(os.environ.get('RUN2_GATE_MM', '2.0')); overall = 1e9
print('※ 최저 점 높이 열은 FK 월드 z다(종이 위 높이 = FK z + 11 mm). 2026-11-17 정정: 이전 출력은 이 열을 "종이 위"로 잘못 표기했다.')
print('d_x 후보 / dz | 정상 경로(하강·상승) 최소 간격 (부위) | 중단 복귀 경로 최소 간격 | 최저 점 높이 FK z, 명령 각도 기준(종이 위 = +11, 원시 각도라면 약 −5)')
for dx in tuple(float(v) for v in _os.environ.get('RUN2_DX_LIST', '-3.0,1.3,3.0').split(',')):   # RUN2_DX_LIST: d_x 후보(mm), 기본 −3, +1.3, +3(2026-11-17 환경변수 추가)
    xb = xa - dx; c = np.array([-xb, 189.0, -11.0 + 15.0])   # 블록 중심 FK z = TABLE_Z(−11) + 15 (종이 = FK z + 11). 2026-11-17 정정: 이전 15.0은 블록을 11 mm 높게 둠     # FK 월드 x = −프로젝트 x
    for dz in (0.0, -3.0, -6.0):   # dz −6: Run 2 경향(원시가 명령보다 약 5 mm 낮음)을 덮는 변형(2026-11-17 추가)
        best = (1e9, None, None, None); minz = 1e9
        for q, l in zip(states[first:], labels[first:]):
            W, tg = points(q); W = W + np.array([0,0,dz]); v, part, zmin = clear(W, tg, c, half)
            minz = min(minz, zmin)
            if v < best[0]: best = (v, part, l, q)
        ab = (1e9, None)
        for q, l in abort_states:
            W, tg = points(q); W = W + np.array([0,0,dz]); v, part, zmin = clear(W, tg, c, half); minz = min(minz, zmin)
            if v < ab[0]: ab = (v, (part, l))
        overall = min(overall, best[0], ab[0])
        print('d_x %+.1f / dz %+.0f | %.1f (%s, 이동 J%d→%s 자세 %s) | %.1f (%s %s) | %.1f' % (dx, dz, best[0], best[1], best[2][0], best[2][1], best[2][2], ab[0], ab[1][0], ab[1][1], minz))

print('전체 최소 간격 %.1f mm, 게이트-접촉 %.1f mm → %s' % (overall, GATE, '통과' if overall >= GATE else '미달'))
sys.exit(0 if overall >= GATE else 1)
