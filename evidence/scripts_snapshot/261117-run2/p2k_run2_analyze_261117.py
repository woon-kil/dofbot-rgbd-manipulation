#!/usr/bin/env python3
"""Run 2(첫 접촉) 분석(촬영·실행 전 사전 등록, 2026-11-17). 사용: .venv/bin/python -I evidence/G06/p2k_run2_analyze_261117.py <결과.json> <사진 접두사 data/g04/d1_<TS>_> <기준 사진 png>
성공 정의(모두 충족해야 '접촉 확인 성공', 하나라도 어긋나면 실패로 센다, 규칙 수정 없음):
 S1 정렬: 결과의 run2.align_confirmed == True (사람 y)이고 Mac 정렬 보고가 '정렬 OK'였다(보고는 화면 출력이라 사용자 확인으로 갈음, 분석기는 29010 사진으로 같은 계산을 다시 한다).
 S2 하강: ended가 completed이거나 하강 중단이 없고(정렬 N 중단 제외), 29020·29030·29040 정지 중 J2~J4 처짐(drift) 없음.
 S3 접촉: P2h 정체 규칙(명령 − 회독 ≥ 3° 이고 같은 값 연속 2회)이 닫기 계단(120,130,135,140,145,150) 중 한 번 충족(close_steps에 stall). 150까지 정체가 없으면 '접촉 확인 실패(블록 없음 곡선 150에서 148 = −2° 와 같거나 물림 깊이 얕음)'.
 S4 블록 위치·기울어짐: 열린 뒤 사진(29041)의 마커 id 7 중심이 기준 사진 대비 ≤ 2 px 이고 변 길이 차 ≤ 1 px. 마커 미검출이면 판정 불가(실패로 센다).
 S5 열기: run2.release_readback이 열림 100 ±3.
 참고(판정 아님): 각 정지 사진의 캡 중점·d_x·캡별 옆 여유(p2k_analyze 규칙), 정체 명령·회독 값, 닫기 단계별 회독.
 기록해 두는 한계: 물림 깊이는 약 7 mm(메시 예측, 관측은 더 얕을 수 있음)라 정체가 안 생길 수 있다 — 그 경우도 실패로 기록하고 해석(깊이 대 접촉)은 이 시험으로 분리하지 않는다."""
import sys, json, os
sys.path.insert(0,'evidence/G06'); sys.path.insert(0,'scripts'); sys.path.insert(0,'src')
import cv2, numpy as np, importlib.util
spec = importlib.util.spec_from_file_location('pa','evidence/G06/p2k_analyze_261117.py'); PA = importlib.util.module_from_spec(spec); spec.loader.exec_module(PA)
res = json.load(open(sys.argv[1])); pre = sys.argv[2]; ref = sys.argv[3]
pose = res['poses']; r2 = {}; drift = []
for p in pose:
    r2.update(p.get('run2', {})); drift += p.get('drift', [])
rows = {}
rows['S1'] = bool(r2.get('align_confirmed'))
rows['S2'] = (res['ended'] == 'completed' or (r2.get('align_confirmed') and not drift)) and not drift
steps = r2.get('close_steps', [])
stall = next((s for s in steps if s.get('stall')), None); rows['S3'] = stall is not None
mref = PA.E.detect(cv2.imread(ref)).get(7); mpost = PA.E.detect(cv2.imread(pre + '29041.png')).get(7) if os.path.exists(pre + '29041.png') else None
if mref and mpost:
    sh = float(np.hypot(mpost['center'][0]-mref['center'][0], mpost['center'][1]-mref['center'][1])); sd = abs(mpost['side_px']-mref['side_px']); rows['S4'] = (sh <= 2.0 and sd <= 1.0); s4txt = '마커 이동 %.2f px, 변 길이 차 %.2f px' % (sh, sd)
else: rows['S4'] = False; s4txt = '마커 미검출(판정 불가=실패)'
rr = r2.get('release_readback'); rows['S5'] = rr is not None and abs(rr-100) <= 3
print('Run 2 분석 (사전 등록 규칙)')
print('S1 정렬 확인(사람 y):', rows['S1']); print('S2 하강(처짐·중단 없음):', rows['S2'], '| drift', drift)
print('S3 정체(P2h 규칙):', rows['S3'], '| 정체 명령/회독', (stall['target'], stall['final']) if stall else None, '| 닫기 단계', [(s['target'], s['final']) for s in steps])
print('S4 블록 위치·기울어짐:', rows['S4'], '|', s4txt); print('S5 열기 회독:', rows['S5'], '|', rr)
print('종합:', '접촉 확인 성공' if all(rows.values()) else '접촉 확인 실패 또는 불완전(%s)' % [k for k, v in rows.items() if not v])
print('참고(판정 아님): 정지 사진별 캡 검출')
q0 = None
for p in pose:
    pr = p.get('capture', {}).get('raw_pre') or {}
    try: q0 = [pr[k]['deg'] for k in '1234']
    except Exception: pass
for tag in ('29010', '29020', '29030', '29040', '29041'):
    f = pre + tag + '.png'
    if not os.path.exists(f) or q0 is None: print(' ', tag, '사진 없음'); continue
    cp = PA.detect_caps(f, ref, (697.0, 235.0))
    print(' ', tag, '캡', [[round(v, 1) for v in c] for c in cp] if cp else '미검출')
