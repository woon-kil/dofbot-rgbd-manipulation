"""D3 진행 판정용 선형 높이 모델(맥, UrdfFK): 캡 아래끝(종이 위 mm) = FK TCP z(판 기준) + 11 − 22.5. 기준 q_ref 근처 편미분 a2·a3·a4(mm/°)를 구해 계획 JSON에 넣는다.
출력: evidence/G06/d3_height_model.json (생성기가 세트 24 계획에 height_check로 복사)"""
import sys, json
sys.path.insert(0,'src')
import numpy as np
from kinematics.urdf_fk import UrdfFK
fk=UrdfFK()
def h(j2,j3,j4): return float(fk.fk(np.array([[110.0,j2,j3,j4,90.0]]))['tcp'][0][0][2]*1000+11-22.5)
qref=(54.9,28.5,29.1)                       # 설계 최악 J2 54.9, 실측 J3 28.5, J4 명령 30 → 원시 ≈ 29.1
h_ref=h(*qref); d=0.5
a=[ (h(qref[0]+d,qref[1],qref[2])-h(qref[0]-d,qref[1],qref[2]))/(2*d),
    (h(qref[0],qref[1]+d,qref[2])-h(qref[0],qref[1]-d,qref[2]))/(2*d),
    (h(qref[0],qref[1],qref[2]+d)-h(qref[0],qref[1],qref[2]-d))/(2*d)]
lin=lambda j2,j3,j4: h_ref+a[0]*(j2-qref[0])+a[1]*(j3-qref[1])+a[2]*(j4-qref[2])
print('h_ref %.2f, 계수 a2 %.3f a3 %.3f a4 %.3f (mm/°)'%(h_ref,*a))
print('선형 대 정확(FK) 모서리 비교:')
worst=0
for q in ((54.9,28.0,28.0),(54.5,28.0,28.0),(55.1,28.5,29.1),(54.9,29.5,30.1),(53.9,28.0,28.0)):
    e=h(*q); l=lin(*q); worst=max(worst,abs(e-l)); print(q,'정확 %.2f 선형 %.2f 차 %.2f'%(e,l,l-e))
print('최대 선형 오차 %.2f mm'%worst)
json.dump({"q_ref":list(qref),"h_ref":round(h_ref,3),"a":[round(x,4) for x in a],"note":"cap(종이 위 mm)=h_ref+a2(J2-q2)+a3(J3-q3)+a4(J4-q4), J1 110, 오프셋 22.5 mm(수직)","max_linear_err_mm":round(worst,3)},open('evidence/G06/d3_height_model.json','w'),indent=1,ensure_ascii=False)
