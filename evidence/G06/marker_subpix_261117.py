#!/usr/bin/env python3
"""마커 id 7 서브픽셀 검출기(사전 등록 제안, 2026-11-17, 승인 전). 하드웨어·카메라 명령 없음. 사진 파일만 읽는다.
기존 `scripts/g01_e0g_analyze.detect`는 정수 픽셀 꼭짓점(cornerRefinementMethod 기본 = NONE)이다. 이 검출기는 cornerRefinementMethod = CORNER_REFINE_SUBPIX(1)만 바꾸고
나머지 DetectorParameters는 OpenCV 4.14.0 기본값(cornerRefinementWinSize 5, cornerRefinementMaxIterations 30, cornerRefinementMinAccuracy 0.1)을 그대로 쓴다.
출력: center(px), side_px, yaw_deg(윗·아랫변 각도와 좌·우변 각도−90의 평균), corners. Run 3·4를 이 검출기로 다시 판정하지 않는다(참고용)."""
import cv2, numpy as np

def _detector(mode):
    p = cv2.aruco.DetectorParameters(); p.cornerRefinementMethod = mode
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), p)

def detect(img, marker_id=7, mode=cv2.aruco.CORNER_REFINE_SUBPIX):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    c, ids, _ = _detector(mode).detectMarkers(g)
    if ids is None: return None
    for cc, i in zip(c, ids.ravel()):
        if int(i) == marker_id:
            q = cc.reshape(4, 2).astype(float)
            ang = lambda a, b: np.degrees(np.arctan2(b[1] - a[1], b[0] - a[0]))
            top = (ang(q[0], q[1]) + ang(q[3], q[2])) / 2; side = ((ang(q[0], q[3]) + ang(q[1], q[2])) / 2) - 90
            return {'center': q.mean(0).tolist(), 'side_px': float(np.mean([np.linalg.norm(q[k] - q[(k + 1) % 4]) for k in range(4)])),
                    'yaw_deg': float((top + side) / 2), 'corners': q.tolist()}
    return None
