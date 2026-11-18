#!/usr/bin/env python3
"""G07 E-0G Gemini 들어올림 이동량 분석(오프라인). 로봇·카메라 명령 없음. 사진 파일만 읽는다.

사용: g01_e0g_analyze.py --dir data/g04 --prefix e0g --heights 0,30,60 [--reps 2] [--out 결과.json]
파일명: <dir>/<prefix>_h<높이mm>_r<번호>.png (Gemini 컬러 1280x720). 높이 0이 반드시 있어야 한다(예측 기준).
판정(승인 문서 §2, 사전 등록): 모델 일치 = 예측 방향으로 투영한 마커 중심 이동이 예측의 ±25 % 안, 재현성 = 같은 높이 두 장 중심 차 ≤ 1 px,
구분력 = 재현 차 × 3 / 기울기, Q3(사용자 변경 2026-11-16) = 같은 촬영 구간에서 첫 사진(h0_r1) 대비 로봇 베이스 상부 영역 위상 상관 이동 ≤ 2 px·응답 ≥ 0.3 (id 1 대 11-15 기준은 참고만), 노랑 덩어리 지표 = 아래 가장자리 행.
"""
import argparse, json
from pathlib import Path
import numpy as np, cv2

K = (610.51, 610.35, 641.79, 364.44)               # Gemini 컬러(g01_marker_detect.py)
N_UP = np.array([0.0226, -0.4655, -0.8847])         # 작업대 법선(카메라 쪽 = 위), G01 깊이 평면 맞춤
N_DOT_C0 = -0.4036                                  # n·c0 (카메라→평면 부호 있는 거리 m)
BLOCK_H = 0.030                                     # 노랑 블록 높이 m
MARK = 0.025
BASE_BOX = (480, 0, 900, 100)                       # 로봇 베이스 상부(고정 구조) 영역 x0,y0,x1,y1
REF_ID1 = (699.0, 292.2)                            # p1_block_check4에서 검출한 보드 마커 id 1 중심
TOL = 0.25; REPEAT_PX = 1.0; Q3_TOL_PX = 2.0; BASE_RESP_MIN = 0.3         # 사전 등록: 모델 ±25 %, 재현 ≤ 1 px, Q3 id 1 위치 차 ≤ 2 px, 노랑 지표 = 아래 가장자리 행


def detect(img):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), cv2.aruco.DetectorParameters())
    c, ids, _ = det.detectMarkers(g)
    out = {}
    if ids is not None:
        for cc, i in zip(c, ids.ravel()):
            p = cc.reshape(4, 2)
            out[int(i)] = {"center": p.mean(0).tolist(), "side_px": float(np.mean([np.linalg.norm(p[k] - p[(k + 1) % 4]) for k in range(4)]))}
    return out


def yellow_blob(img, near):
    """마커 근처(±90 px)의 노란색 최대 연결 덩어리 중심·아래 가장자리. 임계: 블록 H 28~30 S 208~215, 테이블 H 23~25 S 142~167(p1_block_check4 실측)."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = ((hsv[..., 0] >= 27) & (hsv[..., 0] <= 35) & (hsv[..., 1] >= 185) & (hsv[..., 2] >= 150)).astype(np.uint8)
    x, y = int(near[0]), int(near[1]); roi = np.zeros_like(m); roi[max(0, y - 90):y + 90, max(0, x - 90):x + 90] = 1
    m = m * roi
    n, lab, st, cen = cv2.connectedComponentsWithStats(m)
    if n <= 1:
        return None
    i = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    return {"center": cen[i].tolist(), "bottom_y": float(st[i, cv2.CC_STAT_TOP] + st[i, cv2.CC_STAT_HEIGHT]), "area": int(st[i, cv2.CC_STAT_AREA])}


def base_shift(ref, img):
    """고정 구조(로봇 베이스 상부) 영역의 위상 상관 이동량(px)과 응답. Q3 변경(사용자 결정): 같은 촬영 구간 안에서 기준 사진 대비 ≤ 2 px."""
    x0, y0, x1, y1 = BASE_BOX
    a = cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY).astype(np.float32)[y0:y1, x0:x1]; b = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)[y0:y1, x0:x1]
    (dx, dy), resp = cv2.phaseCorrelate(a, b, cv2.createHanningWindow((x1 - x0, y1 - y0), cv2.CV_32F))
    return float(dx), float(dy), float(resp)


def predict_px(center0, lift_m):
    fx, fy, cx, cy = K
    ray = np.array([(center0[0] - cx) / fx, (center0[1] - cy) / fy, 1.0])
    t = (BLOCK_H + N_DOT_C0) / (ray @ N_UP)
    Y = ray * t + N_UP * lift_m
    return np.array([cx + fx * Y[0] / Y[2], cy + fy * Y[1] / Y[2]])


def analyze(d, prefix, heights, reps):
    d = Path(d); rows = {}; ref_img = None; base = []
    for h in heights:
        shots = []
        for r in range(1, reps + 1):
            f = d / f"{prefix}_h{h}_r{r}.png"
            im = cv2.imread(str(f)) if f.exists() else None
            if im is None:
                shots.append({"file": f.name, "missing": True}); continue
            det = detect(im); m7 = det.get(7)
            if ref_img is None:
                ref_img = im
            else:
                dx, dy, resp = base_shift(ref_img, im); base.append({"file": f.name, "dx": dx, "dy": dy, "response": resp})
            shots.append({"file": f.name, "id7": m7, "id1": det.get(1), "blob": yellow_blob(im, m7["center"]) if m7 else None})
        rows[h] = shots
    ok0 = [s for s in rows.get(0, []) if s.get("id7")]
    if not ok0:
        return {"error": "높이 0 사진에서 마커 id 7 검출 실패", "rows": rows}
    c0 = np.mean([s["id7"]["center"] for s in ok0], 0)
    bb0 = np.mean([s["blob"]["bottom_y"] for s in ok0 if s.get("blob")]) if any(s.get("blob") for s in ok0) else None
    res = {"robot_commands": 0, "center0_px": c0.tolist(), "heights": []}
    for h in heights:
        ok = [s for s in rows[h] if s.get("id7")]
        row = {"h_mm": h, "n_detected": len(ok), "n_shots": len(rows[h])}
        if ok:
            cs = np.array([s["id7"]["center"] for s in ok]); mean = cs.mean(0)
            row["center_mean_px"] = mean.tolist()
            row["shift_y_px"] = float(c0[1] - mean[1]); row["shift_total_px"] = float(np.linalg.norm(mean - c0))
            row["side_px_mean"] = float(np.mean([s["id7"]["side_px"] for s in ok]))
            row["repeat_diff_px"] = float(np.linalg.norm(cs[0] - cs[1])) if len(cs) >= 2 else None
            pred = predict_px(c0, h / 1000.0); row["pred_px"] = pred.tolist()
            row["pred_shift_total_px"] = float(np.linalg.norm(pred - c0))
            if h > 0:
                u = (pred - c0) / np.linalg.norm(pred - c0)                       # 예측 방향 단위벡터
                row["shift_along_pred_px"] = float((mean - c0) @ u)               # 재적층 가로 오차를 빼려고 예측 방향으로 투영
                row["ratio_to_pred"] = row["shift_along_pred_px"] / row["pred_shift_total_px"]
                row["model_ok"] = abs(row["ratio_to_pred"] - 1) <= TOL
            bb = [s["blob"]["bottom_y"] for s in ok if s.get("blob")]
            if bb and bb0 is not None:
                row["blob_bottom_shift_px"] = float(bb0 - np.mean(bb))             # 양수 = 위로 이동
            row["id1_visible"] = any(s.get("id1") for s in ok)
        res["heights"].append(row)
    id1 = [s["id1"]["center"] for s in ok0 if s.get("id1")]
    res["Q3_id1"] = {"measured_px": np.mean(id1, 0).tolist() if id1 else None, "ref_px": list(REF_ID1),
                     "diff_px": float(np.linalg.norm(np.mean(id1, 0) - np.array(REF_ID1))) if id1 else None}
    res["Q3_id1_vs_1115_info"] = {"ok_by_old_rule": bool(id1) and res["Q3_id1"]["diff_px"] <= Q3_TOL_PX}   # 참고(사전 기준, 변경 전 규칙)
    res["Q3_base_shift"] = base
    res["Q3_ok"] = bool(base) and all(abs(b["dx"]) <= Q3_TOL_PX and abs(b["dy"]) <= Q3_TOL_PX and b["response"] >= BASE_RESP_MIN for b in base)
    reps_ = [r["repeat_diff_px"] for r in res["heights"] if r.get("repeat_diff_px") is not None]
    res["repeat_max_px"] = max(reps_) if reps_ else None
    res["repeat_ok"] = bool(reps_) and max(reps_) <= REPEAT_PX
    pos = [r for r in res["heights"] if r["h_mm"] > 0 and r.get("shift_along_pred_px")]
    if pos and reps_:
        slope = float(np.mean([r["shift_along_pred_px"] / r["h_mm"] for r in pos]))
        res["slope_px_per_mm"] = slope
        res["min_discriminable_mm"] = 3 * max(reps_) / slope if slope else None
    res["model_ok_all"] = all(r.get("model_ok") for r in res["heights"] if r["h_mm"] > 0)
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="data/g04"); ap.add_argument("--prefix", default="e0g")
    ap.add_argument("--heights", default="0,30,60"); ap.add_argument("--reps", type=int, default=2); ap.add_argument("--out")
    a = ap.parse_args()
    r = analyze(a.dir, a.prefix, [int(x) for x in a.heights.split(",")], a.reps)
    if a.out:
        Path(a.out).write_text(json.dumps(r, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in r.items() if k != "rows"}, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
