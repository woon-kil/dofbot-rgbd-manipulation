#!/usr/bin/env python3
"""오프라인(하드웨어 없음, UNVERIFIED): D1-A 자세 2202 원시 각도에서 J6 178/140/100의 두 캡 끝을 Gemini 영상 화소로 투영한다.
가정: J6→grip 대응은 P2g 안쪽 간격과 메시 안쪽 간격 대응(178→-1.5, 140→-1.1, 100→-0.79), 캡 끝 = 메시 손끝 + 4 mm 손가락 축(R14~R16 실측 +24.4 − 메시 +20.3에서 추정).
실행: .venv/bin/python -I evidence/G06/gemini_j6_visibility_check_261117.py (프로젝트 루트). 출력 요약은 docs/gates/G07-P2j-… §13."""
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
q=[109.5,70.9,14.0,30.2]; o=D.fk.fk(np.array([q+[90.0]])); t=o['tcp'][0][0]*1000; R=o['tcp'][1][0]
for j6,g in ((178,-1.5),(140,-1.1),(100,-0.79)):
    ch=chain(g); res=[]
    for s in 'rl':
        pts=np.vstack([(np.c_[M[k],np.ones(len(M[k]))]@ch[k].T)[:,:3] for k in ch if k[0]==s]); p=pts[np.argmax(pts[:,1])].copy(); p[1]+=4.0
        w=t+R@np.array([p[0],p[1]-90,0]); res.append((w,D.P(-w[0],w[1],(w[2]+11)/1000)))
    print('J6 %d: 캡 끝 월드 z %.1f/%.1f mm, 화소 (%.0f,%.0f)/(%.0f,%.0f), 두 캡 화소 간격 %.0f'%(j6,res[0][0][2],res[1][0][2],*res[0][1],*res[1][1],abs(res[0][1][0]-res[1][1][0])))
