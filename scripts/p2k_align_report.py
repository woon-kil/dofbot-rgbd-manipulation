#!/usr/bin/env python3
"""Run 2 정렬 보고(Mac, 실행 중 50 mm 열린 상태 사진 1장): d_x, 캡별 옆 여유, 마커 관문을 사전 등록 규칙(p2k_analyze_261117.py)으로 계산해 화면에 출력한다. 판단은 사람이 y/N로 한다.
사용: .venv/bin/python -I scripts/p2k_align_report.py <점 사진 png> <기준 사진(블록 있음, 팔 K1) png>
정렬 OK = |d_x| ≤ 3 mm 그리고 두 캡 옆 여유 ≥ 6 mm 그리고 기준 사진 마커가 Run 1 기준 (678.2, 265.8)의 1 px 이내. q(높이 평면용)는 Run 1 원시 J2~J4(81.0, 16.28, 19.31)와 J1 87.4(명령 87 예상)로 고정한다."""
import sys, importlib.util, os
sys.path.insert(0, 'evidence/G06'); sys.path.insert(0, 'scripts'); sys.path.insert(0, 'src')
import cv2, numpy as np
spec = importlib.util.spec_from_file_location('pa', 'evidence/G06/p2k_analyze_261117.py'); PA = importlib.util.module_from_spec(spec); spec.loader.exec_module(PA)
img, ref = sys.argv[1], sys.argv[2]; Q = [87.4, 81.0, 16.28, 19.31]; RUN1_MARKER = (678.2, 265.8)
if len(sys.argv) > 3 and sys.argv[3] == '33':   # 세트 33: 재등록 1회 한정 기준 파일(정확히 1개)만 쓴다. 없으면 Run 1 위치·J1 87.4로 되돌아가지 않고 NG
    import g01_lift_marker_ref as LMR
    try: RUN1_MARKER = LMR.load()[:2]; Q[0] = LMR.j1_assumption()   # 마커 좌표는 마커 기준 파일 그대로, J1 원시 가정만 J1 가정 파일(2026-11-19 승인, 플랜 v2 §4)
    except LMR.MarkerRefError as e: print('\n' + '=' * 60 + '\n[세트 33 정렬 보고] ' + str(e) + '\n결과: 정렬 NG — 러너 질문에 N (하강 없이 복귀)\n' + '=' * 60); sys.exit(0)
if len(sys.argv) > 3 and sys.argv[3] == '31' and os.path.exists('evidence/G06/p2k_marker_ref.txt'): _rf = [l for l in open('evidence/G06/p2k_marker_ref.txt', encoding='utf-8') if not l.startswith('#')]; RUN1_MARKER = tuple(float(v) for v in _rf[0].split()[:2]); Q[0] = float(_rf[1].split()[0])   # 세트 31: 사전 등록 파일(Run 3 복귀 뒤 위치)
out = ['', '=' * 60, '[Run 2 정렬 보고] (사전 등록 규칙, J6 100 열린 상태 50 mm 사진)']
mk = PA.E.detect(cv2.imread(ref)).get(7)
if mk is None: out.append('기준 사진에서 마커 id 7 미검출 → 정렬 NG')
else:
    dmk = float(np.hypot(mk['center'][0] - RUN1_MARKER[0], mk['center'][1] - RUN1_MARKER[1])); out.append('기준 사진 마커 (%.1f, %.1f), 기준 (%.2f, %.2f)과 차 %.2f px (관문 ≤ 1.0)' % (mk['center'][0], mk['center'][1], RUN1_MARKER[0], RUN1_MARKER[1], dmk))
c = PA.caps_px(Q, 100); mid_pred = ((c[0][0] + c[1][0]) / 2, (c[0][1] + c[1][1]) / 2)
cp = PA.detect_caps(img, ref, mid_pred) if os.path.exists(img) else None
ok = False
if cp is None: out.append('두 캡 미검출 → 정렬 NG')
else:
    mid = ((cp[0][0] + cp[1][0]) / 2, (cp[0][1] + cp[1][1]) / 2); d = PA.dx_mm(img, Q, 100, mid, ref)
    if d is None: out.append('d_x 계산 불가 → 정렬 NG')
    else:
        zc = PA.caps_world(Q, 100)[2]; marg = []
        for cpx in sorted(cp): xi, yi = PA.inv_P(cpx[0], cpx[1], (zc + 11) / 1000.0); marg.append(round(abs(xi - d['x_block']) - 15 - 5, 2))
        dj1 = -d['d_x_mm'] / PA.dx_per_J1(Q, 100)
        out.append('두 캡 (%.1f,%.1f) (%.1f,%.1f), 점 사진 마커 %s' % (cp[0][0], cp[0][1], cp[1][0], cp[1][1], d['marker_point_photo_px']))
        out.append('d_x = %+.2f mm (허용 ±3), 캡별 옆 여유 [왼쪽 %.2f, 오른쪽 %.2f] mm (허용 ≥ 6), 참고 J1 보정량 %+.2f°' % (d['d_x_mm'], marg[0], marg[1], dj1))
        ok = (abs(d['d_x_mm']) <= 3.0 and min(marg) >= 6.0 and mk is not None and dmk <= 1.0)
out.append('결과: ' + ('정렬 OK — 조건이 모두 맞으면 러너 질문에 y' if ok else '정렬 NG — 러너 질문에 N (하강 없이 복귀)')); out.append('=' * 60)
print('\n'.join(out))
