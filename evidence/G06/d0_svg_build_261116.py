"""D0 안내 SVG 생성: Gemini 사진(p1b2c_ruler1.png) 위에 손끝 자리·눈금자 자리를 그리고, 그린 픽셀을 mm로 역산해 글의 값과 대조한다.
평면 모델: E-0G 평면(N_UP, N_DOT_C0, K), 영상 가로 = 프로젝트 x(사용자 오른쪽), 영상 위 = 베이스 쪽(y 감소). 영상축과 월드축 사이 회전은 무시(±10 mm)."""
import sys; sys.path.insert(0,'scripts')
import numpy as np, g01_e0g_analyze as E, json
fx,fy,cx,cy=E.K; n=np.array(E.N_UP); d0=E.N_DOT_C0
def ray(u,v): return np.array([(u-cx)/fx,(v-cy)/fy,1.0])
def on_plane(u,v,h): r=ray(u,v); return r*((h+d0)/(r@n))
def px(X): return np.array([cx+fx*X[0]/X[2], cy+fy*X[1]/X[2]])
up=np.array([0,-1.0,0]); yd=up-(up@n)*n; yd/=np.linalg.norm(yd); xd=np.cross(n,yd); xd=xd if xd[0]>0 else -xd; xd/=np.linalg.norm(xd)
Xm=on_plane(697.5,268.0,0.030); Xb=Xm-0.030*n      # 블록 윗면 마커 중심 아래의 종이 면 점 = 블록 중심(x 1.4, y 189)
BX,BY=1.4,189.0
def P(x,y,h=0.0): return px(Xb+((x-BX)/1000)*xd+((BY-y)/1000)*yd+h*n)     # y 증가 = 사용자 쪽(영상 아래)
def inv(u,v):   # 픽셀 → 종이 면 mm (h=0)
    X=on_plane(u,v,0.0)-Xb; return BX+1000*(X@xd), BY-1000*(X@yd)
spots={'손끝(위에서 본 자리)':(68.0,187.0),'새 눈금자 발':(170.0,187.0),'옛 눈금자 발(이동 필요)':(97.0,175.0),'블록 중심':(BX,BY)}
rows=[]
for k,(x,y) in spots.items():
    u,v=P(x,y); xi,yi=inv(u,v); rows.append((k,x,y,round(float(u),1),round(float(v),1),round(xi,1),round(yi,1)))
print('요소 | 글의 값(x,y mm) | 그린 픽셀 | 픽셀 역산(x,y mm) | 일치')
for r in rows: print(r[0],'|',(r[1],r[2]),'|',(r[3],r[4]),'|',(r[5],r[6]),'|',abs(r[1]-r[5])<0.2 and abs(r[2]-r[6])<0.2)
(u1,v1)=P(1.4,187); (u2,v2)=P(68.0,187); (u3,v3)=P(170.0,187)
print('거리 역산: 블록 중심→손끝', round(inv(u2,v2)[0]-BX,1),'mm, 손끝→새 눈금자',round(inv(u3,v3)[0]-inv(u2,v2)[0],1),'mm')
def c(name): u,v=P(*spots[name]); return u,v
svg=['<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" viewBox="0 0 1280 720" width="640" height="360" font-family="sans-serif">','<image href="../data/g04/p1b2c_ruler1.png" x="0" y="0" width="1280" height="720"/>']
def txt(u,v,s,col='#fff',size=18,anchor='start'):
    svg.append(f'<text x="{u:.0f}" y="{v:.0f}" font-size="{size}" fill="{col}" stroke="#000" stroke-width="3" paint-order="stroke" text-anchor="{anchor}" font-weight="bold">{s}</text>')
u,v=c('손끝(위에서 본 자리)'); svg.append(f'<circle cx="{u:.0f}" cy="{v:.0f}" r="16" fill="none" stroke="#00e5ff" stroke-width="4"/><circle cx="{u:.0f}" cy="{v:.0f}" r="3" fill="#00e5ff"/>'); svg.append(f'<line x1="{u:.0f}" y1="{v+16:.0f}" x2="{u-40:.0f}" y2="{v+85:.0f}" stroke="#00e5ff" stroke-width="2"/>'); txt(u-40,v+105,'① 손끝이 내려올 자리(위에서 본 모습)','#00e5ff',17,'middle')
bu,bv=c('블록 중심'); svg.append(f'<line x1="{bu+40:.0f}" y1="{bv:.0f}" x2="{u-22:.0f}" y2="{v:.0f}" stroke="#ffeb3b" stroke-width="4" stroke-dasharray="8 6"/>'); txt((bu+u)/2-10,v+160,'노란 점선: J1 110, 손끝이 블록에서 옆으로 약 67 mm','#ffeb3b',16,'middle')
u,v=c('새 눈금자 발'); svg.append(f'<rect x="{u-9:.0f}" y="{v-30:.0f}" width="18" height="30" fill="none" stroke="#76ff03" stroke-width="4"/>'); txt(u+10,v-45,'② 눈금자(새 자리)','#76ff03',17,'middle'); txt(u+10,v-25,'손끝에서 옆 약 100 mm','#76ff03',15,'middle')
u,v=c('옛 눈금자 발(이동 필요)'); svg.append(f'<line x1="{u-22:.0f}" y1="{v-22:.0f}" x2="{u+22:.0f}" y2="{v+10:.0f}" stroke="#ff1744" stroke-width="5"/><line x1="{u+22:.0f}" y1="{v-22:.0f}" x2="{u-22:.0f}" y2="{v+10:.0f}" stroke="#ff1744" stroke-width="5"/>'); svg.append(f'<line x1="{u:.0f}" y1="{v+10:.0f}" x2="{u+30:.0f}" y2="{v+130:.0f}" stroke="#ff1744" stroke-width="2"/>'); txt(u+30,v+150,'③ 옛 눈금자 자리: 팔과 부딪힘 → 옮김','#ff1744',16,'middle')
txt(30,40,'위 = 베이스 쪽, 아래 = 사용자 쪽, 오른쪽 = 사용자 오른쪽(사진 기준)','#fff',18); txt(30,66,'평면 모델 환산 ±10 mm(사진 위 위치는 눈대중 보조)','#fff',15)
svg.append('</svg>'); open('results/261116-d0-손끝-눈금자-배치-도식.svg','w',encoding='utf-8').write('\n'.join(svg)); print('저장')
