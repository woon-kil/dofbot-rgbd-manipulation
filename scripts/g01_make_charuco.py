#!/usr/bin/env python3
"""ChArUco 보정 보드 인쇄용 이미지 생성(7x5, 사각형 16 mm, 마커 12 mm, DICT_6X6_250, 300 dpi)과 검출 검증."""
import cv2, numpy as np
MM = 300/25.4                      # 300 dpi에서 1 mm의 픽셀 수
SQ_MM, MK_MM, NX, NY, MARGIN_MM = 16.0, 12.0, 7, 5, 4.0
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
board = cv2.aruco.CharucoBoard((NX, NY), SQ_MM/1000.0, MK_MM/1000.0, dic)
w = int(round(NX*SQ_MM*MM)); h = int(round(NY*SQ_MM*MM)); m = int(round(MARGIN_MM*MM))
img = board.generateImage((w+2*m, h+2*m), marginSize=m, borderBits=1)
cv2.imwrite("results/charuco_7x5_sq16mm_dict6x6_300dpi.png", img)
print("이미지 크기 px:", img.shape[::-1], "= %.1f x %.1f mm (보드 %.0f x %.0f mm + 여백 %.0f mm)" % (img.shape[1]/MM, img.shape[0]/MM, NX*SQ_MM, NY*SQ_MM, MARGIN_MM))
det = cv2.aruco.CharucoDetector(board)
cc, ci, mc, mi = det.detectBoard(img)
print("검증: 마커 %d개, ChArUco 모서리 %d개 검출 (이론: 마커 %d, 내부 모서리 %d)" % (0 if mi is None else len(mi), 0 if ci is None else len(ci), (NX*NY)//2, (NX-1)*(NY-1)))
# 기존 마커(4x4 id0, AprilTag)와 사전이 달라 혼동되지 않는지
for nm in ("DICT_4X4_250", "DICT_APRILTAG_36h11"):
    dd = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, nm)), cv2.aruco.DetectorParameters())
    _, ids, _ = dd.detectMarkers(img); print("  %s로 이 보드를 검출하면: %s" % (nm, "없음(혼동 없음)" if ids is None else list(ids.ravel())))
