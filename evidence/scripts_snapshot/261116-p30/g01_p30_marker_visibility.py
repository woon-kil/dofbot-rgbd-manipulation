#!/usr/bin/env python3
"""G07 P3-0 블록 윗면 마커 가시성 확인 도구. **로봇 명령·관절 읽기 코드가 없다**(서보 어댑터 import 없음).

모드:
  --analyze DIR        DIR의 h{mm:02d}_r{n}.png 컬러 영상에서 AprilTag 36h11 id 7을 찾아 판정(오프라인).
  --fake OUT           예측 투영으로 가짜 영상을 만들어 OUT에 저장하고 판정(도구 시험용, 하드웨어 없음).
  --real OUT           JETSON_HOST ssh로 DaBai 드라이버를 띄우고 h마다 사람이 홀더를 맞춘 뒤 촬영(R10 승인 뒤에만).
  --predict            예측 표만 출력.
판정 기준(승인 문서 §3): Q1 h=0 검출 여부, Q2 재현 2장 모두 검출되는 최소 h, 예측 일치(행·열 ±20 px).
"""
import argparse, json, os, subprocess, sys, time
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CAM = json.loads((ROOT / "evidence/G02/dabai_color_camera_info.json").read_text(encoding="utf-8"))
K = np.array(CAM["K"], dtype=float).reshape(3, 3) if len(np.array(CAM["K"]).shape) == 1 else np.array(CAM["K"], dtype=float)
FX, FY, CX, CY = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
W, H_IMG = 640, 480
TCP = (10.8, 52.8, 95.9)          # 카메라→TCP mm (S4-13, UNVERIFIED)
D_GRASP = 10.0                    # 파지 깊이 d mm (가정)
MARK = 25.0                       # 마커 한 변 mm
H_LIST = [0, 10, 20, 30, 43, 50, 60]
REPS = 2
TOL_PX = 20.0
TAG_ID = 7


def predict(h):
    """h mm에서 마커 중심·4모서리 픽셀 예측(축 평행·정렬 가정, UNVERIFIED)."""
    z = TCP[2] - D_GRASP + h
    c3 = np.array([[TCP[0] + sx * MARK / 2, TCP[1] + sy * MARK / 2, z] for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
    uv = np.stack([CX + FX * c3[:, 0] / z, CY + FY * c3[:, 1] / z], 1)
    centre = (CX + FX * TCP[0] / z, CY + FY * TCP[1] / z)
    inside = bool(((uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H_IMG)).all())
    margin = float(min(uv[:, 0].min(), W - uv[:, 0].max(), uv[:, 1].min(), H_IMG - uv[:, 1].max()))
    return {"h": h, "z_mm": round(z, 1), "centre_px": [round(centre[0]), round(centre[1])], "corners_px": uv.round(1).tolist(),
            "all_corners_inside": inside, "min_edge_margin_px": round(margin, 1)}


def detect(png):
    import cv2
    img = cv2.imread(str(png))
    if img is None:
        return {"file": Path(png).name, "error": "읽기 실패"}
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(gray)
    res = {"file": Path(png).name, "detected": False, "image_mean_brightness": round(float(gray.mean()), 1)}
    if ids is not None:
        for c, i in zip(corners, ids.ravel()):
            if int(i) == TAG_ID:
                pts = c.reshape(4, 2)
                res.update(detected=True, centre_px=[round(float(pts[:, 0].mean()), 1), round(float(pts[:, 1].mean()), 1)],
                           side_px=round(float(np.mean([np.linalg.norm(pts[k] - pts[(k + 1) % 4]) for k in range(4)])), 1))
    return res


def analyze(d):
    d = Path(d)
    rows = []
    for h in H_LIST:
        pred = predict(h)
        dets = [detect(d / f"h{h:02d}_r{r}.png") for r in range(1, REPS + 1) if (d / f"h{h:02d}_r{r}.png").exists()]
        n_det = sum(1 for x in dets if x.get("detected"))
        match = None
        if n_det:
            devs = [(abs(x["centre_px"][0] - pred["centre_px"][0]), abs(x["centre_px"][1] - pred["centre_px"][1])) for x in dets if x.get("detected")]
            match = all(a <= TOL_PX and b <= TOL_PX for a, b in devs)
        rows.append({"h": h, "n_images": len(dets), "n_detected": n_det, "pred": pred, "detections": dets, "centre_matches_pred": match})
    full = [r["h"] for r in rows if r["n_images"] >= REPS and r["n_detected"] == r["n_images"]]
    q1 = next((r for r in rows if r["h"] == 0), None)
    q1_text = "촬영 없음" if not q1 or not q1["n_images"] else ("반증(h=0에서 검출됨, 기하·가정 재검토)" if q1["n_detected"] else "예측과 일치(h=0에서 검출 안 됨)")
    missing = [r["h"] for r in rows if r["n_images"] < REPS]
    return {"robot_commands": 0, "tag": f"AprilTag 36h11 id {TAG_ID}", "rows": rows, "Q1": q1_text,
            "Q2_min_h_all_detected": min(full) if full else None, "missing_images_h": missing,
            "caveat": "검출기는 4모서리가 모두 영상 안에 있고 가장자리 여유(가짜 영상 시험에서 약 12 px에서 검출 실패)가 있어야 검출한다. 일부만 보이는 경우는 사진을 눈으로 확인한다."}


def fake_images(out):
    import cv2
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    tag = cv2.aruco.generateImageMarker(dic, TAG_ID, 200)
    tag = cv2.copyMakeBorder(tag, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=255)
    src = np.float32([[20, 20], [219, 20], [219, 219], [20, 219]])
    for h in H_LIST:
        uv = np.float32(predict(h)["corners_px"])
        M = cv2.getPerspectiveTransform(src, uv)
        img = cv2.warpPerspective(tag, M, (W, H_IMG), flags=cv2.INTER_LINEAR, borderValue=128)
        for r in range(1, REPS + 1):
            cv2.imwrite(str(out / f"h{h:02d}_r{r}.png"), cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
    return out


def real(out):
    host = os.environ.get("JETSON_HOST")
    if not host:
        sys.exit("JETSON_HOST 없음")
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    ssh = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", host]
    src = "source /opt/ros/humble/setup.bash; source ~/dofbot_pro_ws/install/setup.bash"
    def run(cmd):
        return subprocess.run([*ssh, cmd], capture_output=True, text=True, timeout=90)
    if run("pgrep -f '[_]_ns:=/dabai'").stdout.strip():
        sys.exit("이미 드라이버가 떠 있다")
    subprocess.run(["scp", "-q", "-o", "BatchMode=yes", str(ROOT / "scripts/g01_dabai_probe.py"), f"{host}:/tmp/g01_dabai_probe.py"], check=True)
    run(f"{src}; nohup ros2 launch orbbec_camera dabai_dcw2.launch.py camera_name:=dabai enable_point_cloud:=false enable_colored_point_cloud:=false enable_ir:=false > /tmp/g01_p30_drv.log 2>&1 & echo $! > /tmp/g01_p30.pid; sleep 12")
    try:
        if not run("pgrep -f '[_]_ns:=/dabai'").stdout.strip():
            sys.exit("드라이버 기동 실패")
        for h in H_LIST:
            for r in range(1, REPS + 1):
                if input(f"[h={h} mm, {r}/{REPS}] 홀더를 정해진 위치에 맞추고 손을 뗀 뒤 Enter(촬영), q 중단: ").strip().lower() == "q":
                    return out
                run(f"{src}; python3 /tmp/g01_dabai_probe.py /tmp/g01_p30_view 3 dabai > /dev/null")
                subprocess.run(["scp", "-q", "-o", "BatchMode=yes", f"{host}:/tmp/g01_p30_view/color.png", str(out / f"h{h:02d}_r{r}.png")], check=True)
                print("  저장", f"h{h:02d}_r{r}.png")
    finally:
        run("kill -INT $(cat /tmp/g01_p30.pid 2>/dev/null) 2>/dev/null; sleep 2; pkill -INT -f '[_]_ns:=/dabai' 2>/dev/null; sleep 2")
        print("드라이버 종료:", run("pgrep -fa '[_]_ns:=/dabai' || echo 드라이버 없음").stdout.strip())
    return out


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--analyze"); g.add_argument("--fake"); g.add_argument("--real"); g.add_argument("--predict", action="store_true")
    ap.add_argument("--result")
    a = ap.parse_args()
    if a.predict:
        for h in H_LIST:
            p = predict(h); print(h, p["z_mm"], p["centre_px"], "전체가시" if p["all_corners_inside"] else "가려짐/밖")
        return
    d = fake_images(a.fake) if a.fake else (real(a.real) if a.real else Path(a.analyze))
    res = analyze(d)
    if a.result:
        Path(a.result).write_text(json.dumps(res, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: res[k] for k in ("Q1", "Q2_min_h_all_detected", "missing_images_h")}, ensure_ascii=False))


if __name__ == "__main__":
    main()
