#!/usr/bin/env python3
"""첫 접촉 시험 선행조건 c(오프라인, UNVERIFIED): J6 열림에서 열린 손가락(링크 메시 + 고무캡 근사 상자)과 블록(30x60x30 mm, 열림 축 x폭 30)의 최소 간격을 하강 높이·옆 오프셋별로 계산한다.
가정: grip↔J6 대응(P2g 안쪽 간격, 100 → −0.79, 더 열린 경우 −0.62), 캡 = 손끝 메시 점 뒤로 25 mm·앞 4 mm·x 폭 10·z 두께 6 상자(UNVERIFIED), 블록 중심 FK 월드 (−1.4, 189) mm(프로젝트 x는 FK x의 반대), 종이 높이 0.
실행: .venv/bin/python -I evidence/G06/open_finger_block_clearance_261117.py"""
import sys; sys.path.insert(0,'evidence/G06'); sys.path.insert(0,'scripts'); sys.path.insert(0,'src')
import numpy as np, struct, re
import gemini_fingertip_detect_r15 as D
def stl(p):
    b=open(p,'rb').read()
    if b[:5]==b'solid' and b'facet' in b[:400]: return np.array(re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)',b),dtype=float)
    n=struct.unpack('<I',b[80:84])[0]; a=np.frombuffer(b[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')])); return a['v'].reshape(-1,3).astype(float)
M={k:stl('data/refs/dofbot_meshes/%s.STL'%k)*1e3 for k in ('rlink1','rlink2','rlink3','llink1','llink2','llink3')}
def Rz(a): c,s=np.cos(a),np.sin(a); R=np.eye(4); R[:2,:2]=[[c,-s],[s,c]]; return R
def Tm(xyz,yaw,ang): m=Rz(yaw)@Rz(ang); m[:3,3]=np.array(xyz)*1e3; return m
def chain(g):
    r1=Tm([0.012625,0.029275,0.00475],0.42401,-g); r2=r1@Tm([0.027229,-0.012593,-0.0025],1.1468,g)
    l1=Tm([-0.012375,0.029275,0.0045],2.7292,g); l2=l1@Tm([0.027362,0.012302,-0.0025],-1.1584,-g)
    r3=Tm([0.0045,0.047275,0.0045],0.42401,-g); l3=Tm([-0.0045,0.047275,0.0045],2.7292,g)
    return {'rlink1':r1,'rlink2':r2,'rlink3':r3,'llink1':l1,'llink2':l2,'llink3':l3}
def finger_points(q,g,cap_cx=None,tag=False):
    o=D.fk.fk(np.array([list(q)+[90.0]])); t=o['tcp'][0][0]*1000; R=o['tcp'][1][0]; ch=chain(g); pts=[]; tips={}
    for side in 'rl':
        P=np.vstack([(np.c_[M[k],np.ones(len(M[k]))]@ch[k].T)[:,:3] for k in ch if k[0]==side]); tip=P[np.argmax(P[:,1])]; tips[side]=tip
        pts.append(P)
        # 캡 상자(손끝 메시 점 기준 뒤 25·앞 4, x 폭 10, z 두께 6)를 점으로 샘플
        xs=np.linspace(-5,5,5); ys=np.linspace(-25,4,13); zs=np.linspace(-3,3,3)
        cx=tip[0] if cap_cx is None else (cap_cx/2 if side=='r' else -cap_cx/2)   # 관측 보정: 캡 중심 간격(mm)을 직접 지정(P2k 관측 J6 100 약 61.8 mm)
        pts.append(np.array([[cx+x,tip[1]+y,tip[2]+z] for x in xs for y in ys for z in zs]))
    A=np.vstack(pts); W=t+(R@np.c_[A[:,0],A[:,1]-90,A[:,2]].T).T
    nlink=sum(len(x) for x in pts[0::2]); part=np.array(['링크']*len(A)); 
    off=0
    for k,x in enumerate(pts):
        part[off:off+len(x)]='링크' if k%2==0 else '캡'; off+=len(x)
    global PART; PART=part
    return W,t,R
def box_clear(W,c,half):
    d=np.abs(W-c)-half; out=np.maximum(d,0); dist=np.linalg.norm(out,axis=1); inside=(d<0).all(axis=1)
    v=np.where(inside,d.max(axis=1),dist); global LIM; LIM=PART[int(np.argmin(v))]
    return v.min()
half=np.array([15.0,30.0,15.0]); CAPCX=float(sys.argv[1]) if len(sys.argv)>1 else None   # 인자: 캡 중심 간격 mm(관측 보정, 없으면 메시 끝 점 기준)

J2=np.arange(40,96,1.0); J3=np.arange(0,101,2.0); J4=np.arange(10,101,2.0)
G=np.array(np.meshgrid(J2,J3,J4,indexing='ij')).reshape(3,-1).T; Q=np.c_[np.full(len(G),90.0),G,np.full(len(G),90.0)]
o=D.fk.fk(Q); t=o['tcp'][0]*1000; R=o['tcp'][1]; a=R[:,:,1]; a=np.where(a[:,2:3]<0,a,-a); cap=t+a*24.4; tilt=np.degrees(np.arccos(np.clip(-a[:,2],0,1)))
print('※ 오프라인 UNVERIFIED. 단위 mm. 간격 = 열린 손가락(링크+캡 근사)과 블록 상자 사이 최소 거리(음수 = 관통).')
print('캡 끝(오므림 모델) 높이 | 자세(J2,J3,J4,tilt) | J6 100: 옆 오프셋 −8/−4/0/+4/+8 mm의 블록 간격 | 최소 높이(손가락 점, 종이 위) | J6 더 열림(−0.62) 간격 −8/0/+8')
for level in (50,40,30,25,20,15):
    ok=(np.hypot(cap[:,0]-(-1.4),cap[:,1]-189)<=2)&(np.abs(cap[:,2]-level)<=1.5)&(tilt<=30)&(G[:,0]>=52)&(G[:,1]>=10)&(G[:,2]>=10)   # 러너 관절 창(J2 하한은 세트별, 여기서는 52 이상, J3·J4 [10,105])
    idx=np.where(ok)[0]
    if len(idx)==0: print(level,'후보 없음'); continue
    i=sorted(idx,key=lambda k:(abs(tilt[k]-27),abs(cap[k,2]-level)))[0]; q=Q[i,:4]
    row=[]
    for g,offs in ((-0.79,(-8,-4,0,4,8)),(-0.62,(-8,0,8))):
        W,tt,RR=finger_points(q,g,CAPCX if g==-0.79 else None); res=[]
        for dx in offs: res.append((round(float(box_clear(W,np.array([-1.4+dx,189.0,15.0]),half)),1),LIM))
        row.append((res,round(float(W[:,2].min()),1)))
    print('%d | J2 %.0f J3 %.0f J4 %.0f tilt %.0f | %s | %.1f | %s'%(level,q[1],q[2],q[3],tilt[i],row[0][0],row[0][1],row[1][0]))
