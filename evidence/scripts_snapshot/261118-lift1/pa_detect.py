#!/usr/bin/env python3
"""P-A: 마커 id 7 검출(사전 등록 정수 검출기: AprilTag 36h11, 기본 DetectorParameters = 코너 정제 없음). Mac에서 실행.
사용: python3 scripts/pa_detect.py <사진.png> [<meta.json> <요청id>]   → 한 줄 JSON(found, center, side_px, yaw_deg). meta·요청id를 주면 같은 프로세스에서 크기(720×1280×3)·ok·요청 id 대조를 먼저 하고 실패하면 종료 코드 4(프로세스 수를 줄여 M1 지연을 줄이기 위함)
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
    if len(sys.argv) not in (2, 4): sys.exit("사용: python3 scripts/pa_detect.py <사진.png> [<meta.json> <요청id>]")
    im = cv2.imread(sys.argv[1])
    if im is None: sys.exit("사진을 읽지 못했다: " + sys.argv[1])
    if len(sys.argv) == 4:
        if im.shape != (720, 1280, 3): print("사진 크기가 720x1280이 아니다", file=sys.stderr); sys.exit(4)
        m = json.load(open(sys.argv[2]))
        if not (m.get("ok") and m.get("request_id") == sys.argv[3]): print("요청 id 불일치 또는 ok 아님: %r" % (m.get("request_id"),), file=sys.stderr); sys.exit(4)
    print(json.dumps(detect(im), ensure_ascii=False))
