#!/usr/bin/env python3
"""마커 yaw 검출 노이즈 확인(합성, 정답 각도 알고 있음, 하드웨어 없음). 사용: .venv/bin/python -I evidence/G06/marker_subpix_synthetic_261117.py
마커 한 변 약 29 px(Gemini 사진의 블록 마커 크기), 가우시안 잡음 σ=2, 회전 θ ∈ {0, 1, 2, 3, −2}°, 부분 픽셀 이동 3종. 정수(NONE) 대 서브픽셀(SUBPIX) yaw 오차."""
import sys; sys.path.insert(0, 'evidence/G06')
import cv2, numpy as np, marker_subpix_261117 as M
rng = np.random.default_rng(0)
def synth(theta, dx, dy, side=29.0):
    big = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 7, 200)
    mk = cv2.copyMakeBorder(big, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255)
    rot = cv2.warpAffine(mk, cv2.getRotationMatrix2D((200, 200), theta, 1.0), (400, 400), flags=cv2.INTER_CUBIC, borderValue=255)
    sm = cv2.resize(rot, None, fx=side / 200.0, fy=side / 200.0, interpolation=cv2.INTER_AREA)
    sh = cv2.warpAffine(sm, np.float32([[1, 0, 60 + dx], [0, 1, 45 + dy]]), (160, 120), flags=cv2.INTER_LINEAR, borderValue=180)
    out = np.clip(sh + rng.normal(0, 2.0, sh.shape), 0, 255).astype(np.uint8)
    return cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
res = {'정수': [], '서브픽셀': []}
for th in (0.0, 1.0, 2.0, 3.0, -2.0):
    for sub in (0.0, 0.3, 0.6):
        im = synth(th, sub, sub * 0.5)
        for name, mode in (('정수', cv2.aruco.CORNER_REFINE_NONE), ('서브픽셀', cv2.aruco.CORNER_REFINE_SUBPIX)):
            r = M.detect(im, mode=mode)
            if r is not None: res[name].append(r['yaw_deg'] - (-th))
for k, v in res.items(): print('%s: n=%d 평균 %+.2f° 표준편차 %.2f° 최대 |%.2f|°' % (k, len(v), np.mean(v), np.std(v), np.max(np.abs(v))))
