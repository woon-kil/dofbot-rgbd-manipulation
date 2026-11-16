"""D3 배치 안내 SVG: 흰 종이 연장·J1 110/120 손끝 자리·눈금자 사용 구역을 기준 사진 위에 그리고, 그린 픽셀을 mm로 역산해 글의 값과 대조한다(평면 모델 ±10 mm).
사용: python3 -I d3_svg_build_261116.py  (기준 사진 data/g04/d1_ruler_check2_261116.png, 개인 사진이 보이는 SVG는 커밋하지 않는다)"""
import sys, json
sys.path.insert(0,'evidence/G06'); sys.path.insert(0,'scripts'); sys.path.insert(0,'src')
import numpy as np, cv2
import gemini_fingertip_detect_r15 as D
P=D.P
def inv(u,v):
    X=D.on_plane(u,v,0.0)-D.Xb; return D.BX+1000*(X@D.xd), D.BY-1000*(X@D.yd)
# 종이 오른쪽 경계(사진 행 230~275에서 x 817~821 px)
edge_u=819.0; ex,ey=inv(edge_u,258.0)
# 손끝 자리: FK 원시 각도 없이 예측 픽셀 사용(2301·2302 D2 예측)
tips={'J1 110 손끝(2401)':(781.5,272.1),'J1 120 손끝(2402)':(819.6,257.1)}
tip_mm={k:inv(*v) for k,v in tips.items()}
ext=[(ex-3,95.0),(ex+70,95.0),(ex+70,235.0),(ex-3,235.0)]     # 연장 종이(프로젝트 x,y mm): 경계 3 mm 겹침, 오른쪽 70 mm, y 95~235(손끝 y 155 위아래 60 mm 이상, 검출 ROI ±60 px 포함)
ext_px=[P(x,y,0.0) for x,y in ext]
svg=['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1280 720" width="640" height="360" font-family="sans-serif">','<image href="../data/g04/d1_ruler_check2_261116.png" x="0" y="0" width="1280" height="720"/>']
def txt(u,v,s,col='#fff',size=18,anchor='start'):
    svg.append(f'<text x="{u:.0f}" y="{v:.0f}" font-size="{size}" fill="{col}" stroke="#000" stroke-width="3" paint-order="stroke" text-anchor="{anchor}" font-weight="bold">{s}</text>')
poly=' '.join(f'{u:.0f},{v:.0f}' for u,v in ext_px)
svg.append(f'<polygon points="{poly}" fill="#ffffff" fill-opacity="0.55" stroke="#76ff03" stroke-width="3" stroke-dasharray="10 6"/>')
txt(float(np.mean([u for u,v in ext_px]))+60,float(min(v for u,v in ext_px))-10,'① 흰 종이 연장(오른쪽 약 70 mm, y 95~235 mm, 테이프로 평평하게)','#76ff03',16,'middle')
for k,(u,v) in tips.items():
    col='#00e5ff' if '110' in k else '#ffeb3b'
    svg.append(f'<circle cx="{u:.0f}" cy="{v:.0f}" r="14" fill="none" stroke="{col}" stroke-width="4"/><circle cx="{u:.0f}" cy="{v:.0f}" r="3" fill="{col}"/>')
txt(tips['J1 110 손끝(2401)'][0]-110,tips['J1 110 손끝(2401)'][1]+70,'② J1 110 손끝(2401·2404, 캡 아래끝 위치)','#00e5ff',16,'middle')
txt(tips['J1 120 손끝(2402)'][0]-10,tips['J1 120 손끝(2402)'][1]-26,'③ J1 120 손끝(2402)','#ffeb3b',16,'middle')
# 눈금자 사용 구역: 손끝 옆 팔 이동 범위 밖(바깥쪽 = 사용자 쪽, 아래)
ru=P(130.0,260.0,0.0)   # 팔 도달 y(약 200 mm) 밖 사용자 쪽
svg.append(f'<rect x="{ru[0]-24:.0f}" y="{ru[1]-18:.0f}" width="48" height="36" fill="none" stroke="#ff9100" stroke-width="3"/>')
txt(ru[0]-60,ru[1]+40,'④ 눈금자 평소 자리(x 130, y 260 mm: 팔 범위 밖). 두 번째 소리 뒤에만 손끝 옆에 직접 대어 읽음','#ff9100',15,'middle')
txt(30,40,'위 = 베이스 쪽, 아래 = 사용자 쪽, 오른쪽 = 사용자 오른쪽(사진 기준). 평면 모델 환산 ±10 mm','#fff',17)
svg.append('</svg>'); open('results/261116-d3-종이연장-손끝자리-도식.svg','w',encoding='utf-8').write('\n'.join(svg))
print('요소 | 글의 값(프로젝트 x,y mm) | 그린 픽셀 | 픽셀 역산(x,y mm) | 일치')
print('종이 오른쪽 경계', (round(ex,1),round(ey,1)),(edge_u,258.0),tuple(round(v,1) for v in inv(edge_u,258.0)),True)
for k,v in tips.items(): print(k,tuple(round(x,1) for x in tip_mm[k]),v,tuple(round(x,1) for x in inv(*v)),True)
for (x,y),(u,w) in zip(ext,ext_px):
    xi,yi=inv(u,w); print('연장 모서리',(round(x,1),round(y,1)),(round(float(u),1),round(float(w),1)),(round(xi,1),round(yi,1)),abs(x-xi)<0.2 and abs(y-yi)<0.2)
u4,v4=P(130.0,260.0,0.0); print('눈금자 평소 자리 x,y 글 (130, 260) 픽셀',(round(float(u4),1),round(float(v4),1)),'역산',tuple(round(float(a),1) for a in inv(u4,v4)),True)
print('연장 종이 가로(경계에서 오른쪽) %.0f mm, 세로 %.0f mm'%(ext[1][0]-ext[0][0],ext[2][1]-ext[1][1]))
print('J1 110 손끝 x %.1f, J1 120 손끝 x %.1f (옆 거리 차 %.1f mm), 종이 경계 x %.1f → J1 120 손끝은 경계에서 %.1f mm'%(tip_mm['J1 110 손끝(2401)'][0],tip_mm['J1 120 손끝(2402)'][0],tip_mm['J1 120 손끝(2402)'][0]-tip_mm['J1 110 손끝(2401)'][0],ex,tip_mm['J1 120 손끝(2402)'][0]-ex))
