#!/usr/bin/env python3
"""ChArUco 보드를 A4(210x297 mm) 용지에 정확한 실제 크기로 배치한 인쇄용 PDF/PNG를 만든다. 300 dpi, 100 mm 스케일 바 포함."""
import cv2, numpy as np
from PIL import Image, ImageDraw, ImageFont
MM = 300/25.4
import sys
NX, NY, SQ, MK, ID0 = (int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]), int(sys.argv[5])) if len(sys.argv) > 5 else (7, 5, 16.0, 12.0, 0)
TAG = "charuco_%dx%d_sq%gmm_4x4_id%d" % (NX, NY, SQ, ID0) if len(sys.argv) > 5 else "charuco_a4_100pct"
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50 if len(sys.argv) > 5 else cv2.aruco.DICT_6X6_250)
ids = np.arange(ID0, ID0 + (NX*NY)//2, dtype=np.int32) if len(sys.argv) > 5 else None
board = cv2.aruco.CharucoBoard((NX, NY), SQ/1000.0, MK/1000.0, dic, ids) if ids is not None else cv2.aruco.CharucoBoard((NX, NY), SQ/1000.0, MK/1000.0, dic)
bw, bh = int(round(NX*SQ*MM)), int(round(NY*SQ*MM)); m = int(round(4*MM))
bimg = board.generateImage((bw+2*m, bh+2*m), marginSize=m, borderBits=1)
W, H = int(round(210*MM)), int(round(297*MM))
page = Image.new("L", (W, H), 255); d = ImageDraw.Draw(page)
ox, oy = int(round(45*MM)), int(round(60*MM))          # 보드(여백 포함) 왼쪽 위 위치: 45, 60 mm
page.paste(Image.fromarray(bimg), (ox, oy))
f = ImageFont.load_default(size=36) if hasattr(ImageFont, "load_default") else None
def mm(x): return int(round(x*MM))
d.text((mm(20), mm(20)), "ChArUco %dx%d, square %g mm, marker %g mm, %s" % (NX, NY, SQ, MK, "DICT_4X4_50 ids %d-%d" % (ID0, ID0+(NX*NY)//2-1) if ids is not None else "DICT_6X6_250"), fill=0, font=f)
d.text((mm(20), mm(30)), "Print at 100% (Actual size, NOT fit to page). Verify the bar below = 100 mm.", fill=0, font=f)
# 100 mm 스케일 바(양끝 눈금)
by = mm(180); x0 = mm(55); x1 = x0 + mm(100)
d.line((x0, by, x1, by), fill=0, width=6); d.line((x0, by-30, x0, by+30), fill=0, width=6); d.line((x1, by-30, x1, by+30), fill=0, width=6)
d.text((x0, by+40), "100 mm", fill=0, font=f)
# 16 mm 기준 정사각형(채움)
d.rectangle((mm(55), mm(200), mm(55)+mm(16), mm(200)+mm(16)), fill=0); d.text((mm(78), mm(200)), "16 mm square", fill=0, font=f)
page.save("results/%s.png" % TAG, dpi=(300, 300))
page.save("results/%s.pdf" % TAG, resolution=300.0)
print("A4 페이지 px:", page.size, "= %.1f x %.1f mm" % (page.size[0]/MM, page.size[1]/MM))
print("보드(여백 포함) 크기: %.1f x %.1f mm, 위치 (%.0f, %.0f) mm" % (bimg.shape[1]/MM, bimg.shape[0]/MM, ox/MM, oy/MM))
chk = Image.open("results/%s.png" % TAG); print("PNG dpi 메타데이터:", chk.info.get("dpi"))
# 검증: 페이지에서 보드를 다시 검출
det = cv2.aruco.CharucoDetector(board); arr = np.array(page)
cc, ci, mc, mi = det.detectBoard(arr); print("페이지에서 재검출: 마커 %d, 모서리 %d (이론 %d, %d)" % (0 if mi is None else len(mi), 0 if ci is None else len(ci), (NX*NY)//2, (NX-1)*(NY-1)))
print("보드 물리 크기: %g x %g mm (사각형 %g, 마커 %g, 마커/사각형 %.2f)" % (NX*SQ, NY*SQ, SQ, MK, MK/SQ))
