#!/usr/bin/env python3
"""Gemini 정합 컬러에서 ArUco/AprilTag 마커를 검출해 사전·ID·픽셀 크기·물리 크기(평면 위 거리)를 구한다. 영상 분석만(오프라인)."""
import json, sys
import cv2, numpy as np
img = cv2.imread("evidence/G01/raw/gemini/registered/color.png")
d = np.load("evidence/G01/raw/gemini/registered/depth_mm.npy").astype(float)/1000.0
fx, fy, cx, cy = 610.51, 610.35, 641.79, 364.44
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
names = ["DICT_4X4_50","DICT_4X4_100","DICT_4X4_250","DICT_4X4_1000","DICT_5X5_50","DICT_5X5_100","DICT_5X5_250","DICT_5X5_1000","DICT_6X6_50","DICT_6X6_100","DICT_6X6_250","DICT_6X6_1000","DICT_7X7_50","DICT_7X7_100","DICT_7X7_250","DICT_7X7_1000","DICT_ARUCO_ORIGINAL","DICT_APRILTAG_16h5","DICT_APRILTAG_25h9","DICT_APRILTAG_36h10","DICT_APRILTAG_36h11"]
# 작업대 평면(마스크 영역에서 맞춤)으로 4모서리를 3D로 환산
x0,x1,y0,y1=550,830,190,395
vv,uu=np.mgrid[y0:y1,x0:x1]; dd=d[y0:y1,x0:x1]; ok=(dd>0)&(dd<3)
P=np.stack([(uu[ok]-cx)*dd[ok]/fx,(vv[ok]-cy)*dd[ok]/fy,dd[ok]],1); Ps=P[np.random.default_rng(0).choice(len(P),5000,replace=False)]
c0=Ps.mean(0); _,_,vt=np.linalg.svd(Ps-c0,full_matrices=False); n=vt[2]; n=-n if n[2]>0 else n
def hit(u,v):
    ray=np.array([(u-cx)/fx,(v-cy)/fy,1.0]); return ray*((c0@n)/(ray@n))
found=[]
for nm in names:
    dic=cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,nm)); det=cv2.aruco.ArucoDetector(dic,cv2.aruco.DetectorParameters())
    corners,ids,_=det.detectMarkers(gray)
    if ids is None: continue
    for c,i in zip(corners,ids.ravel()):
        pts=c.reshape(4,2); side_px=np.mean([np.linalg.norm(pts[k]-pts[(k+1)%4]) for k in range(4)])
        P3=[hit(*p) for p in pts]; side_mm=np.mean([np.linalg.norm(P3[k]-P3[(k+1)%4]) for k in range(4)])*1000
        cen=pts.mean(0)
        found.append({"dict":nm,"id":int(i),"center_px":[round(float(cen[0])),round(float(cen[1]))],"side_px":round(float(side_px),1),"side_mm_on_table_plane":round(float(side_mm),1)})
print(json.dumps(found,ensure_ascii=False,indent=1))
