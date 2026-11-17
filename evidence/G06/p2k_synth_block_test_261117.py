#!/usr/bin/env python3
"""합성 시험(촬영 전, 참고용): R-P2k2 사진의 캡·손가락 화소(기준 사진 차분 > 45, 팽창)를 블록이 있는 d3ref(K1 + 블록 + 마커 id 7)에 세트 28 예측 위치로 옮겨 붙이고, 사전 등록 분석기(--block)를 끝까지 돌려 캡 두 개 검출·마커 id 7 검출·d_x 산출을 확인한다. 그림자는 재현하지 않는다.
실행: .venv/bin/python -I evidence/G06/p2k_synth_block_test_261117.py"""
import sys, json, copy
sys.path.insert(0,'evidence/G06'); sys.path.insert(0,'scripts'); sys.path.insert(0,'src')
import cv2, numpy as np, importlib.util, subprocess
spec = importlib.util.spec_from_file_location('pa','evidence/G06/p2k_analyze_261117.py'); PA = importlib.util.module_from_spec(spec); spec.loader.exec_module(PA)
ref = cv2.imread('data/g04/p2kref_261117-093753.png'); blk = cv2.imread('data/g04/d3ref_261116-224203.png')
res = json.load(open('evidence/G01/raw/g01_5b4_s4225_result-real-261117-093753.json')); pts = res['poses'][0]['j6_points']
q28 = [90.0, 81.0, 18.0, 20.0]; fake = copy.deepcopy(res); fake['poses'][0]['pose_id'] = 2504
for i, pt in enumerate(pts):
    j6 = 178 if pt['label'] in ('start', '4') else pt['j6_target']; qA = [pt['raw_pre_arm'][k]['deg'] for k in '1234']
    cA = PA.caps_px(qA, j6); c28 = PA.caps_px(q28, j6); dx = (c28[0][0]+c28[1][0])/2-(cA[0][0]+cA[1][0])/2; dy = (c28[0][1]+c28[1][1])/2-(cA[0][1]+cA[1][1])/2
    im = cv2.imread('data/g04/d1_261117-093753_2501%d.png' % i); mask = (cv2.cvtColor(np.abs(im.astype(int)-ref.astype(int)).astype(np.uint8), cv2.COLOR_BGR2GRAY) > 45).astype(np.uint8)
    mask = cv2.dilate(mask, np.ones((5, 5), np.uint8)); M = np.float32([[1, 0, dx], [0, 1, dy]])
    mv = cv2.warpAffine(im, M, (im.shape[1], im.shape[0])); mm = cv2.warpAffine(mask, M, (im.shape[1], im.shape[0]))
    out = blk.copy(); out[mm > 0] = mv[mm > 0]; cv2.imwrite('data/g04/d1_synth_2504%d.png' % i, out)
    open('evidence/G06/d1_photo_synth_2504%d.log' % i, 'w').write('photo_start_epoch=1000\nphoto_end_epoch=1036\n')
    for k, kk in zip('1234', q28): fake['poses'][0]['j6_points'][i]['raw_pre_arm'][k] = {'raw': 0, 'deg': kk}
    print('점 %s: 이동 (%.1f, %.1f) px' % (pt['label'], dx, dy))
json.dump(fake, open('/tmp/p2k_synth_result.json', 'w'), ensure_ascii=False)
r = subprocess.run(['.venv/bin/python', '-I', 'evidence/G06/p2k_analyze_261117.py', '/tmp/p2k_synth_result.json', 'data/g04/d1_synth_', 'data/g04/d3ref_261116-224203.png', '--block', '/tmp/p2k_synth_an.json'], capture_output=True, text=True)
print(r.stdout[-3500:], r.stderr[-400:])
