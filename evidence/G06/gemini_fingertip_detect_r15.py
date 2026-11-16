"""Gemini 사진에서 손가락 아래끝 두 점의 중간을 자동 검출하고 'FK TCP − 22.5 mm' 예측 픽셀과 비교한다(사전 고정 규칙, 2026-11-16, D2 시험 전).
규칙(사진을 보기 전 고정, 사후 조정 금지):
 - ROI = 예측 픽셀(오프셋 22.5 mm 투영) 기준 x ±60 px, y ±60 px (고정 ROI 아님).
 - 어두운 화소: 회색조 < 70. 8-연결 성분 중 면적 ≥ 300 px인 것을 면적 순으로 최대 2개.
 - 한 성분당 아래끝 = 성분 최하단 y, x = 최하단 4행의 x 평균. 두 성분이면 중간점, 하나뿐이거나 면적 비 ≥ 1.6이면 '병합: 제외'.
 - 원시 각도가 None인 관절은 readback(정수)+0.5로 두고 FK 범위(정수, 정수+1)를 병기한다.
 - 환산: 예측 위치에서 평면 모델로 구한 국소 px/mm(옆, 앞뒤)를 쓴다(블록 근처 1.32 / 1.07).
사용: python3 -I gemini_fingertip_detect_r15.py [--post-hoc] <결과.json> <사진 접두사 data/g04/d1_261116-185300_> [출력.json]"""
import sys, json
sys.path.insert(0,'scripts'); sys.path.insert(0,'src')
import cv2, numpy as np
import g01_e0g_analyze as E
from kinematics.urdf_fk import UrdfFK
fx,fy,cx,cy=E.K; n=np.array(E.N_UP); d0=E.N_DOT_C0
def on_plane(u,v,h): r=np.array([(u-cx)/fx,(v-cy)/fy,1.0]); return r*((h+d0)/(r@n))
def px(X): return np.array([cx+fx*X[0]/X[2], cy+fy*X[1]/X[2]])
up=np.array([0,-1.0,0]); yd=up-(up@n)*n; yd/=np.linalg.norm(yd); xd=np.cross(n,yd); xd=xd if xd[0]>0 else -xd; xd/=np.linalg.norm(xd)
Xb=on_plane(697.5,268.0,0.030)-0.030*n; BX,BY=1.4,189.0
def P(x,y,h=0.0): return px(Xb+((x-BX)/1000)*xd+((BY-y)/1000)*yd+h*n)
fk=UrdfFK()
def predict(q, off=22.5):
    o=fk.fk(np.array([list(q)+[90.0]])); t=o['tcp'][0][0]*1000; R=o['tcp'][1][0]; ax=R[:,1]; ax=ax if ax[2]<0 else -ax
    b=t+ax*(off/abs(ax[2])); hh=(b[2]+11)/1000.0
    sx=float(np.linalg.norm(P(-b[0]+1.0,b[1],hh)-P(-b[0],b[1],hh))); sy=float(np.linalg.norm(P(-b[0],b[1]-1.0,hh)-P(-b[0],b[1],hh)))   # 이 위치의 px/mm(옆, 앞뒤)
    return P(-b[0],b[1],hh), (sx,sy)
def detect(img_path, pred):
    g=cv2.cvtColor(cv2.imread(img_path),cv2.COLOR_BGR2GRAY); H,W=g.shape
    x0=int(max(0,pred[0]-60)); x1=int(min(W,pred[0]+60)); y0=int(max(0,pred[1]-60)); y1=int(min(H,pred[1]+60))
    m=(g[y0:y1,x0:x1]<70).astype(np.uint8); nc,lab,st,_=cv2.connectedComponentsWithStats(m,8)
    comps=sorted([(int(st[i,cv2.CC_STAT_AREA]),i) for i in range(1,nc) if st[i,cv2.CC_STAT_AREA]>=300],reverse=True)[:2]
    pts=[]
    for a,i in comps:
        ys,xs=np.nonzero(lab==i); yb=ys.max(); pts.append((x0+float(xs[ys>=yb-4].mean()), y0+float(yb), a))
    if len(pts)<2: return None,'병합 또는 미검출: 제외',pts
    if max(pts[0][2],pts[1][2])/min(pts[0][2],pts[1][2])>=1.6: return None,'병합: 제외(면적 비 %.1f)'%(max(pts[0][2],pts[1][2])/min(pts[0][2],pts[1][2])),pts
    return ((pts[0][0]+pts[1][0])/2,(pts[0][1]+pts[1][1])/2),'ok',pts
if __name__=='__main__':
    POST_HOC='--post-hoc' in sys.argv; sys.argv=[x for x in sys.argv if x!='--post-hoc']   # 사전 등록하지 않은 방법·수정 규칙으로 기존 사진을 볼 때 지정: 출력에 '참고용, 결정 근거 아님'을 붙인다
    if POST_HOC: print('※ 사후 분석: 참고용, 결정 근거 아님(사전 등록 규칙이 아님)')
    res=json.load(open(sys.argv[1])); pre=sys.argv[2]; rows=[]
    for p in res['poses']:
        pid=p['pose_id']; r=p['capture']['raw_pre']; q=[r[k]['deg'] for k in '1234']; none4=False
        if any(v is None for v in q):
            none4=True; rb=p['readback']; q=[(v if v is not None else (rb[k]+0.5 if k=='1' else rb[k]-0.5)) for v,k in zip(q,'1234')]   # 정수 회독: J1은 내림(실제 ∈ [정수, 정수+1) → +0.5), J2~J4는 올림(실제 ∈ (정수−1, 정수] → −0.5)
        pred,(sx,sy)=predict(q)
        mid,why,pts=detect(f'{pre}{pid}.png',pred)
        row={'pose_id':pid,'q_raw':[round(v,1) for v in q],'q_none_filled':none4,'pred_px':[round(float(pred[0]),1),round(float(pred[1]),1)],'status':why}
        if mid is not None: row['mid_px']=[round(mid[0],1),round(mid[1],1)]; row['diff_px']=[round(mid[0]-pred[0],1),round(mid[1]-pred[1],1)]; row['local_px_per_mm']=[round(sx,2),round(sy,2)]; row['diff_mm']=[round((mid[0]-pred[0])/sx,1),round((mid[1]-pred[1])/sy,1)]
        row['label']='참고용, 결정 근거 아님(사후)' if POST_HOC else '사전 등록 규칙'; rows.append(row); print({k:(float(v) if hasattr(v,'item') else v) for k,v in row.items()})
    ok=[r for r in rows if 'diff_px' in r]
    if ok:
        a=np.array([r['diff_px'] for r in ok]); print('평균 diff px',a.mean(0).round(1).tolist(),'표준편차',a.std(0).round(1).tolist(),'n',len(ok))
    if len(sys.argv)>3: json.dump(rows,open(sys.argv[3],'w'),ensure_ascii=False,indent=1)
