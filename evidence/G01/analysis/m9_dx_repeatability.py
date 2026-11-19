#!/usr/bin/env python3
"""M9 참고용(사후 분석, 결정 근거 아님): 정렬 보고 3건의 d_x를 보고 로그의 마커 원값(0.25 px)과 캡 좌표로 재계산하고 마커 항·캡 항으로 분해한다. 읽기 전용, 하드웨어 없음.
사용: .venv/bin/python -I evidence/G01/analysis/m9_dx_repeatability.py"""
import sys, importlib.util
sys.path.insert(0,'evidence/G06'); sys.path.insert(0,'scripts'); sys.path.insert(0,'src')
spec=importlib.util.spec_from_file_location('pa','evidence/G06/p2k_analyze_261117.py'); PA=importlib.util.module_from_spec(spec); spec.loader.exec_module(PA)
import g01_lift_marker_ref as LMR
Q=[LMR.load()[2],81.0,16.28,19.31]; zc=PA.caps_world(Q,100)[2]
def dx(mk,c1,c2):
    mid=((c1[0]+c2[0])/2,(c1[1]+c2[1])/2); xb,_=PA.inv_P(mk[0],mk[1],0.030); xc,_=PA.inv_P(mid[0],mid[1],(zc+11)/1000.0); return xc-xb
# 마커 = 로그 '기록: P-A 기준 마커' 원값(0.25 px), 캡 = 보고의 '두 캡'(0.1 px 반올림), 마지막 = 보고 d_x
runs={'261118-211829':((673.25,267.00),(636.5,216.0),(721.6,226.0),2.55),'261118-222500':((672.75,268.00),(636.3,217.0),(721.0,227.0),2.63),'261119-094728':((673.00,268.00),(636.7,216.0),(722.2,226.0),3.06)}
for k,(m,a,b,rep) in runs.items(): print(k,'재계산 %+.2f 보고 %+.2f 차 %+.2f'%(dx(m,a,b),rep,dx(m,a,b)-rep))
m0,a0,b0,_=runs['261118-211829']; base=dx(m0,a0,b0)
print('마커 x 1 px = %+.3f mm (0.25 px 계단 = %+.3f mm), 캡 중점 x 1 px = %+.3f mm, J1 1° = %.2f mm'%(dx((m0[0]+1,m0[1]),a0,b0)-base,(dx((m0[0]+.25,m0[1]),a0,b0)-base),dx(m0,(a0[0]+1,a0[1]),(b0[0]+1,b0[1]))-base,PA.dx_per_J1(Q,100)))
for k in list(runs)[1:]:
    m,a,b,_=runs[k]; print(k,'1회차 대비 총 %+.2f = 마커 %+.2f + 캡 %+.2f'%(dx(m,a,b)-base,dx(m,a0,b0)-base,dx(m0,a,b)-base))
m,a,b,_=runs['261119-094728']; print('3회차 마커를 1회차 자리(673.25,267.00)로 두면 d_x = %+.2f'%dx(m0,a,b))
