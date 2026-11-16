"""Gemini 손가락 끝 검출기 v2 — 배경 차분 방식(D3 시험용, 사진을 찍기 전 사전 등록, 2026-11-16).
v1(`gemini_fingertip_detect_r15.py`)은 어두운 화소 문턱이라 흰 종이/나무 경계 위 손가락을 놓쳤다(2302 제외). v2는 같은 세션의 기준 사진(팔이 K1 직립, 손가락 영역이 비어 있음)과의 차분으로 팔을 분리한다.
규칙(사진을 보기 전 고정, 사후 조정 금지):
 - 기준 사진: 실행 직전 같은 장면(눈금자·사람 없음, 팔 K1)에서 Gemini로 찍은 1장.
 - 차분: |회색조(사진) − 회색조(기준)| > 45, 3x3 열기(open) 1회.
 - ROI: 예측 픽셀(오프셋 22.5 mm 투영) 기준 x ±60, y ±60 px.
 - ROI 안 차분 성분 중 면적 ≥ 800 px인 것의 합집합에서, 최하단 y_max를 구하고 y ≥ y_max − 14 px인 화소만 모아 x로 2-means 군집(초기값 최소·최대 x). 군집 간 중심 거리 < 8 px이거나 한 군집 화소가 < 15개면 '병합: 제외'.
 - 각 군집의 아래끝 = 군집 최하단 y, x = 최하단 4행 x 평균. 중간점 = 두 아래끝의 평균.
 - 사진 유효: 촬영 시간 ≤ hold_s − 2 s(D1 초안 부록 G).
 - 사전 등록 높이식은 D1 초안 부록 B, 오프셋 비교는 예측 픽셀(FK 원시 각도, TCP − 22.5 mm 손가락 축 방향)과의 차이(px → 국소 px/mm).
사용: python3 -I gemini_fingertip_detect_v2.py [--post-hoc] <결과.json> <사진 접두사> <기준 사진> [출력.json]"""
import sys, json
sys.path.insert(0,'evidence/G06'); sys.path.insert(0,'scripts'); sys.path.insert(0,'src')
import cv2, numpy as np
import gemini_fingertip_detect_r15 as D1
def detect_v2(img_path, ref_path, pred):
    g=cv2.cvtColor(cv2.imread(img_path),cv2.COLOR_BGR2GRAY).astype(np.int16); r=cv2.cvtColor(cv2.imread(ref_path),cv2.COLOR_BGR2GRAY).astype(np.int16)
    H,W=g.shape; x0=int(max(0,pred[0]-60)); x1=int(min(W,pred[0]+60)); y0=int(max(0,pred[1]-60)); y1=int(min(H,pred[1]+60))
    m=(np.abs(g[y0:y1,x0:x1]-r[y0:y1,x0:x1])>45).astype(np.uint8); m=cv2.morphologyEx(m,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    nc,lab,st,_=cv2.connectedComponentsWithStats(m,8); keep=np.zeros_like(m)
    for i in range(1,nc):
        if st[i,cv2.CC_STAT_AREA]>=800: keep[lab==i]=1
    ys,xs=np.nonzero(keep)
    if len(ys)==0: return None,'미검출: 제외'
    ymax=ys.max(); sel=ys>=ymax-14; yy=ys[sel]; xx=xs[sel].astype(float)
    c=np.array([xx.min(),xx.max()])
    for _ in range(20):
        a=np.abs(xx[:,None]-c[None,:]).argmin(1)
        if (a==0).sum()==0 or (a==1).sum()==0: break
        c=np.array([xx[a==0].mean(),xx[a==1].mean()])
    a=np.abs(xx[:,None]-c[None,:]).argmin(1)
    if abs(c[0]-c[1])<8 or (a==0).sum()<15 or (a==1).sum()<15: return None,'병합: 제외'
    pts=[]
    for k in (0,1):
        yk=yy[a==k]; xk=xx[a==k]; yb=yk.max(); pts.append((x0+float(xk[yk>=yb-4].mean()), y0+float(yb)))
    return ((pts[0][0]+pts[1][0])/2,(pts[0][1]+pts[1][1])/2),'ok'
if __name__=='__main__':
    POST='--post-hoc' in sys.argv; sys.argv=[x for x in sys.argv if x!='--post-hoc']
    if POST: print('※ 사후 분석: 참고용, 결정 근거 아님(구현 점검용, 사전 등록 규칙이 아닌 자료에 적용)')
    res=json.load(open(sys.argv[1])); pre=sys.argv[2]; ref=sys.argv[3]; rows=[]
    for p in res['poses']:
        pid=p['pose_id']; r=p['capture']['raw_pre']; q=[r[k]['deg'] for k in '1234']; none=False
        if any(v is None for v in q): none=True; rb=p['readback']; q=[(v if v is not None else rb[k]+0.5) for v,k in zip(q,'1234')]
        pred,(sx,sy)=D1.predict(q); mid,why=detect_v2(f'{pre}{pid}.png',ref,pred)
        row={'pose_id':pid,'q_none_filled':none,'pred_px':[round(float(pred[0]),1),round(float(pred[1]),1)],'status':why,'label':'참고용, 결정 근거 아님(사후)' if POST else '사전 등록 규칙 v2'}
        if mid is not None: row['mid_px']=[round(mid[0],1),round(mid[1],1)]; row['diff_px']=[round(mid[0]-pred[0],1),round(mid[1]-pred[1],1)]; row['diff_mm']=[round((mid[0]-pred[0])/sx,1),round((mid[1]-pred[1])/sy,1)]
        rows.append(row); print(row)
    ok=[r for r in rows if 'diff_px' in r]
    if ok: a=np.array([r['diff_px'] for r in ok]); print('평균 diff px',a.mean(0).round(1).tolist(),'표준편차',a.std(0).round(1).tolist(),'n',len(ok))
    if len(sys.argv)>4: json.dump(rows,open(sys.argv[4],'w'),ensure_ascii=False,indent=1)
