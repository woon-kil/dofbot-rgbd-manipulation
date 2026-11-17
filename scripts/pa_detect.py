#!/usr/bin/env python3
"""P-A: 마커 id 7 검출(사전 등록 정수 검출기: AprilTag 36h11, 기본 DetectorParameters = 코너 정제 없음). Mac에서 실행.
사용: python3 scripts/pa_detect.py <사진.png>   → 한 줄 JSON(found, center, side_px, yaw_deg)
구현은 S4' §2 분석 도구 evidence/G06/marker_subpix_261117.py의 detect(mode=CORNER_REFINE_NONE)를 그대로 가져다 쓴다(코드를 다시 쓰지 않는다)."""
import json, sys
from pathlib import Path
import cv2
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "evidence" / "G06"))
import marker_subpix_261117 as MS


def detect(img):
    r = MS.detect(img, 7, cv2.aruco.CORNER_REFINE_NONE)
    if r is None: return {"found": False}
    return {"found": True, "center": r["center"], "side_px": r["side_px"], "yaw_deg": r["yaw_deg"]}


if __name__ == "__main__":
    if len(sys.argv) != 2: sys.exit("사용: python3 scripts/pa_detect.py <사진.png>")
    im = cv2.imread(sys.argv[1])
    if im is None: sys.exit("사진을 읽지 못했다: " + sys.argv[1])
    print(json.dumps(detect(im), ensure_ascii=False))
