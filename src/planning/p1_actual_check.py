"""③ P1 자세 목록을 실제 블록 실측 위치로 다시 평가한다(오프라인, 이동 명령 없음).
실측(2026-11-15 Gemini): 블록 중심 (x +1.4, y 189) mm ±2, 회전 약 1.5°, 마커 중심은 윗면 중심보다 y −0.5 mm.
간섭 박스는 실측 위치에서 반폭 +3 mm(위치 불확도 ±2 + 회전 1.5°의 모서리 이동 약 0.8)로 키워 보수적으로 본다.
마커 4모서리 가시성은 마커 중심을 x ±2, y ±2 mm 흔들고 회전 ±3°를 준 9개 조합에서 모두 통과하는지 본다."""
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from kinematics.urdf_fk import UrdfFK                                   # noqa: E402
from planning.collision_3a import Box, load_link_hull                   # noqa: E402
from planning.dry_run import BEST, TABLE_Z, HULL_KEYS, TRUTH                   # noqa: E402
from planning.observe import PoseGrid                                   # noqa: E402
from planning import p1_poses as P                                      # noqa: E402

CENTER = (0.0014, 0.189)
MARKER_OFFSET_Y = -0.0005
INFLATE = 0.003


def main():
    src = json.load(open(ROOT / "evidence/G06/p1_pose_list_r190.json"))
    fk = UrdfFK(); hulls = {k: load_link_hull(v) for k, v in HULL_KEYS.items()}
    half = np.array([P.BLOCK[0] / 2, P.BLOCK[1] / 2, P.BLOCK[2] / 2])
    box = Box([CENTER[0], CENTER[1], TABLE_Z + P.BLOCK[2] / 2], half + np.array([INFLATE, INFLATE, 0.0]))
    out = {"block_center_mm": [CENTER[0] * 1000, CENTER[1] * 1000], "box_inflate_mm": INFLATE * 1000, "pose_list_sha256_source": src["pose_list_sha256"], "segments": [], "poses": {}}
    poses = {k: v["sdk"] for k, v in src["poses"].items()}
    for s in src["segments"]:
        w, mx = P.path_worst(fk, hulls, poses[s["from"]], poses[s["to"]], box)
        out["segments"].append({"from": s["from"], "to": s["to"], "max_joint_delta_deg": round(mx, 1), "path_worst_clearance_mm": w, "was_mm": s["path_worst_clearance_mm"]})
    grid = PoseGrid(fk, BEST, j234_min=P.J234_MIN)
    top = TABLE_Z + P.BLOCK[2]; h = P.MARKER / 2
    for name in ("A", "B", "B2"):
        q = np.array(poses[name], float)
        i = int(np.argmin(np.linalg.norm(grid.sdk - q, axis=1)))
        assert np.abs(grid.sdk[i] - q)[1:].max() < 1e-6, (name, grid.sdk[i], q)
        res = []
        for dx in (-0.002, 0.0, 0.002):
            for dy in (-0.002, 0.0, 0.002):
                for rot in (-3.0, 0.0, 3.0):
                    c = np.array([CENTER[0] + dx, CENTER[1] + MARKER_OFFSET_Y + dy, top])
                    r = np.radians(rot); R = np.array([[np.cos(r), -np.sin(r)], [np.sin(r), np.cos(r)]])
                    ok = True; mn = 1e9
                    for sx in (-1, 1):
                        for sy in (-1, 1):
                            d = R @ np.array([sx * h, sy * h]); C = c + np.array([d[0], d[1], 0.0])
                            o, _, _, _ = grid.visible_mask(C, 0.0, 40)
                            ok &= bool(o[i])
                    v = grid.cp[i] - c
                    inc = float(np.degrees(np.arccos(v[2] / np.linalg.norm(v))))
                    res.append((ok, inc))
        out["poses"][name] = {"corner_visible_all_%d_combos" % len(res): all(r[0] for r in res), "n_fail": sum(not r[0] for r in res), "incidence_deg_max": round(max(r[1] for r in res), 1)}
    # 분할안: HOME→A(88°)를 이미 평가한 선형 경로 위의 중간 자세 W 하나로 둘로 나눈다(경로 기하는 같다).
    W = [88.0, 99.0, 54.0, 51.0]
    split = []
    for a, b, qa, qb in (("HOME", "W", poses["HOME"], W), ("W", "A", W, poses["A"]), ("A", "W", poses["A"], W), ("W", "HOME", W, poses["HOME"])):
        w, mx = P.path_worst(fk, hulls, qa, qb, box)
        split.append({"from": a, "to": b, "max_joint_delta_deg": round(mx, 1), "path_worst_clearance_mm": w})
    order = [("HOME", "W"), ("W", "A"), ("A", "B"), ("B", "B2"), ("B2", "B"), ("B", "A"), ("A", "W"), ("W", "HOME")]
    canon = json.dumps({"poses": {**poses, "W": W}, "segments": order}, sort_keys=True).encode()   # p1_poses.build와 같은 정규화
    out["split_variant_W"] = {"W": W, "segments": split, "pose_list_split_sha256": hashlib.sha256(canon).hexdigest()}
    # M3 예측: B→B2에서 마커 중심의 카메라 프레임 변위(FK 예측)를 모델 변형별로 계산
    top_ = TABLE_Z + P.BLOCK[2]; M = np.array([CENTER[0], CENTER[1] + MARKER_OFFSET_Y, top_])
    pred = {}
    for nm, t in TRUTH.items():
        vec = {}
        for pn in ("B", "B2"):
            f = fk.fk(np.array([poses[pn] + [90.0]]), gains=t.gains, tool_offset=t.tool_offset())
            pc = f["dabai"][0][0] + np.array([0.0, 0.0, t.dz]); Rc = f["dabai"][1][0]
            vec[pn] = Rc.T @ (M - pc)
        pred[nm] = np.round((vec["B2"] - vec["B"]) * 1000, 2).tolist()
    out["m3_predicted_camera_frame_marker_shift_mm"] = pred
    (ROOT / "evidence/G06/p1_actual_check.json").write_text(json.dumps(out, ensure_ascii=False, indent=1, default=float), encoding="utf-8")
    print(json.dumps(out, ensure_ascii=False, indent=1, default=float))


if __name__ == "__main__":
    main()
