"""③ P1 촬영(자세 A·B·B2, 각 30프레임)의 M1·M2·M3 분석(오프라인, 이동 없음).
사용: python3 -I src/planning/p1_m1_analyze.py <촬영 폴더(g01_s42_view 상위)> <실행 결과 json> [out.json]
M1: AprilTag 36h11 id 7 검출률, PnP(IPPE_SQUARE, 마커 25.0 mm) 재투영 오차, tvec 프레임 간 표준편차. 왜곡은 무시(observe.py와 같은 K).
M2: PnP 마커 중심의 월드 높이(FK+hand-eye 카메라 자세, 읽은 관절값 사용)와 실측 윗면 높이(종이 위 30 mm)의 차.
M3: B→B2 카메라 프레임 마커 변위(PnP) 대 FK 예측."""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from kinematics.urdf_fk import UrdfFK                  # noqa: E402
from planning.dry_run import BEST, TRUTH, TABLE_Z      # noqa: E402
from planning.observe import camera_pose, K            # noqa: E402

MARKER = 0.025
DIST = None   # --dist 이면 evidence/G02/dabai_color_camera_info.json의 D(rational_polynomial 8계수)를 쓴다
CENTER = (0.0014, 0.189 - 0.0005)
TOP = TABLE_Z + 0.030
CAM = np.array([[K[0], 0, K[2]], [0, K[1], K[3]], [0, 0, 1.0]])
OBJ = np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]], float) * MARKER / 2   # cv2 aruco 모서리 순서(좌상, 우상, 우하, 좌하), 마커 평면 x 오른쪽 y 위


def analyze_pose(folder, pid):
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), cv2.aruco.DetectorParameters())
    files = sorted(Path(folder, "p%s" % pid).glob("exp_25*.png")); rows = []
    for f in files:
        im = cv2.imread(str(f), cv2.IMREAD_GRAYSCALE); c, ids, _ = det.detectMarkers(im)
        if ids is None or 7 not in ids.ravel():
            rows.append(None); continue
        pts = c[list(ids.ravel()).index(7)].reshape(4, 2).astype(np.float64)
        ok, rv, tv, err = cv2.solvePnPGeneric(OBJ, pts, CAM, DIST, flags=cv2.SOLVEPNP_IPPE_SQUARE)[:4]
        i = int(np.argmin(err)); proj, _ = cv2.projectPoints(OBJ, rv[i], tv[i], CAM, DIST)
        rows.append({"file": f.name, "t": tv[i].ravel(), "reproj_px": float(np.linalg.norm(proj.reshape(4, 2) - pts, axis=1).mean()), "side_px": float(np.mean([np.linalg.norm(pts[(k + 1) % 4] - pts[k]) for k in range(4)])), "corners": pts, "ambig_err": [float(e) for e in err.ravel()]})
    return files, rows


def main():
    global DIST
    if "--dist" in sys.argv:
        sys.argv.remove("--dist"); DIST = np.array(json.load(open(ROOT / "evidence/G02/dabai_color_camera_info.json"))["D"], float)
    folder = Path(sys.argv[1]); res = json.load(open(sys.argv[2])); out = Path(sys.argv[3]) if len(sys.argv) > 3 else ROOT / ("evidence/G06/p1_m1_analysis_dist.json" if DIST is not None else "evidence/G06/p1_m1_analysis.json")
    fk = UrdfFK(); poses = {"A": "1201", "B": "1202", "B2": "1203"}; summary = {}; means = {}
    rb = {str(p["pose_id"]): p["readback"] for p in res["poses"]}
    for name, pid in poses.items():
        files, rows = analyze_pose(folder, pid); good = [r for r in rows if r]
        q = [rb[pid][str(k)] for k in (1, 2, 3, 4)]
        pred = {}
        for nm, t in TRUTH.items():
            cp, Rc = camera_pose(fk, q, t)
            pred[nm] = (Rc.T @ (np.array([CENTER[0], CENTER[1], TOP]) - cp))
        cp, Rc = camera_pose(fk, q, BEST)
        s = {"readback_q": q, "n_frames": len(files), "n_detected": len(good), "detect_rate": round(len(good) / max(1, len(files)), 3)}
        if good:
            T = np.array([r["t"] for r in good]); tm = T.mean(axis=0); means[name] = tm
            wz = (cp + Rc @ tm)[2]
            s.update({"tvec_mean_mm": np.round(tm * 1000, 2).tolist(), "tvec_std_mm": np.round(T.std(axis=0) * 1000, 3).tolist(), "depth_mm": round(float(tm[2]) * 1000, 1),
                      "reproj_px_mean": round(float(np.mean([r["reproj_px"] for r in good])), 3), "reproj_px_max": round(float(np.max([r["reproj_px"] for r in good])), 3), "side_px_mean": round(float(np.mean([r["side_px"] for r in good])), 2),
                      "fk_predicted_tvec_mm_best": np.round(pred["best"] * 1000, 2).tolist(), "pnp_minus_fk_mm_best": np.round((tm - pred["best"]) * 1000, 2).tolist(),
                      "fk_pred_depth_range_mm_over_models": [round(float(min(p[2] for p in pred.values())) * 1000, 1), round(float(max(p[2] for p in pred.values())) * 1000, 1)],
                      "M2_marker_center_world_z_mm": round(float(wz) * 1000, 2), "M2_expected_z_mm": round(TOP * 1000, 2), "M2_residual_mm": round(float(wz - TOP) * 1000, 2),
                      "min_corner_margin_px": round(float(min(np.min(r["corners"][:, 0]), np.min(r["corners"][:, 1]), np.min(640 - r["corners"][:, 0]), np.min(480 - r["corners"][:, 1])) for r in good) if False else float(min(min(np.min(r["corners"][:, 0]), np.min(r["corners"][:, 1]), np.min(640 - r["corners"][:, 0]), np.min(480 - r["corners"][:, 1])) for r in good)), 1),
                      "ippe_ambiguity_ratio_median": round(float(np.median([max(r["ambig_err"]) / max(1e-9, min(r["ambig_err"])) for r in good])), 2)})
        summary[name] = s
    if "B" in means and "B2" in means:
        qB, qB2 = [rb["1202"][str(k)] for k in (1, 2, 3, 4)], [rb["1203"][str(k)] for k in (1, 2, 3, 4)]
        predm = {}
        for nm, t in TRUTH.items():
            cpB, RB = camera_pose(fk, qB, t); cpB2, RB2 = camera_pose(fk, qB2, t); M = np.array([CENTER[0], CENTER[1], TOP])
            predm[nm] = (RB2.T @ (M - cpB2)) - (RB.T @ (M - cpB))
        d = means["B2"] - means["B"]
        summary["M3"] = {"pnp_shift_mm": np.round(d * 1000, 2).tolist(), "fk_pred_shift_mm_best": np.round(predm["best"] * 1000, 2).tolist(), "diff_mm_best": np.round((d - predm["best"]) * 1000, 2).tolist(),
                         "fk_pred_shift_range_mm": {k: [round(float(min(p[k] for p in predm.values())) * 1000, 2), round(float(max(p[k] for p in predm.values())) * 1000, 2)] for k in range(3)}}
    out.write_text(json.dumps(summary, ensure_ascii=False, indent=1), encoding="utf-8"); print(json.dumps(summary, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
