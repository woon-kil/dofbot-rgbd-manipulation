"""③ 큰 이동 M3: 회독 양자화(±0.5°)가 FK 예측 마커 변위를 mm로 얼마나 흔드는지(오프라인, 이동 명령 없음).
이 시험이 ‘판별 불가’가 되지 않으려면 예측 쪽 양자화 폭이 기대 효과(FK 오차 크기)보다 작아야 한다. 방법(P1-b2 문서 §3의 ‘J2 5~7° → 예측 범위’와 같음):
S·E 자세의 J1~J4 회독 8개에 각각 ±0.5°를 주어(2^8 = 256 조합) 카메라 프레임 예측 변위의 범위와, 공칭 예측에서 벗어나는 최대 벡터 크기를 구하고, 모델 변형 5종(최선 + 비관적 4)에서 반복한다.
입력: evidence/G06/p1_bigm3_poses_s42.json, 출력: evidence/G06/p1_bigm3_quant.json."""
import itertools
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from kinematics.urdf_fk import UrdfFK                                   # noqa: E402
from planning.dry_run import TABLE_Z, TRUTH                             # noqa: E402
from planning.observe import camera_pose                                # noqa: E402
from planning import p1_poses as P                                      # noqa: E402
from planning import p1_b2_poses as B                                   # noqa: E402

HALF = 0.5   # 회독 정수 반올림 폭(도)


def marker_world():
    return np.array([B.CENTER[0], B.CENTER[1] + B.MARKER_OFFSET_Y, TABLE_Z + P.BLOCK[2]])


def cam_point(fk, q4, model, M):
    cp, Rc = camera_pose(fk, q4, model)
    return Rc.T @ (M - cp)


def pair_range(fk, qs, qe, model, M):
    """공칭 변위 d0(카메라 프레임, E−S)와, S·E의 J1~J4 각 ±0.5° 256 조합에서의 변위 범위·공칭 대비 최대 벡터 편차."""
    s0 = cam_point(fk, qs, model, M); d0 = cam_point(fk, qe, model, M) - s0
    ds = []
    for signs in itertools.product((-HALF, HALF), repeat=8):
        a = [qs[k] + signs[k] for k in range(4)]; b = [qe[k] + signs[4 + k] for k in range(4)]
        ds.append(cam_point(fk, b, model, M) - cam_point(fk, a, model, M))
    ds = np.array(ds)
    return d0, ds.min(0), ds.max(0), float(np.linalg.norm(ds - d0, axis=1).max())


def mc_dev(fk, qs, qe, model, M, n=4000, seed=0):
    """반올림 오차가 관절마다 독립 균등(±0.5°)일 때 공칭 대비 예측 변위 벡터 편차의 RMS·95 백분위(mm). 경계값(최악 조합)보다 전형적인 크기."""
    rng = np.random.default_rng(seed); d0 = cam_point(fk, qe, model, M) - cam_point(fk, qs, model, M); v = []
    for _ in range(n):
        e = rng.uniform(-HALF, HALF, 8)
        v.append(np.linalg.norm(cam_point(fk, [qe[k] + e[4 + k] for k in range(4)], model, M) - cam_point(fk, [qs[k] + e[k] for k in range(4)], model, M) - d0))
    v = np.array(v) * 1000
    return float(np.sqrt((v ** 2).mean())), float(np.percentile(v, 95))


def main():
    fk = UrdfFK(); M = marker_world()
    poses_path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "evidence/G06/p1_bigm3_poses_s42.json"   # 인자: [자세 json] [출력 json]
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else ROOT / "evidence/G06/p1_bigm3_quant.json"
    poses = json.load(open(poses_path, encoding="utf-8"))["poses"]
    S = next(p for p in poses if p["name"] == "S")
    out = {"note": "회독 ±0.5° 양자화가 FK 예측 마커 변위(카메라 프레임 mm)에 주는 흔들림. 오프라인 모델, UNVERIFIED.", "pairs": {}}
    for e in poses:
        if e["name"] == "S":
            continue
        qs, qe = S["sdk_deg"][:4], e["sdk_deg"][:4]; rec = {"tip_move_mm": e["tip_move_mm"], "variants": {}}
        for name, model in TRUTH.items():
            d0, lo, hi, dev = pair_range(fk, qs, qe, model, M)
            rec["variants"][name] = {"nominal_mm": np.round(d0 * 1000, 2).tolist(), "min_mm": np.round(lo * 1000, 2).tolist(), "max_mm": np.round(hi * 1000, 2).tolist(), "max_vector_dev_mm": round(dev * 1000, 2)}
        nom = np.array([v["nominal_mm"] for v in rec["variants"].values()])
        rec["quant_max_vector_dev_mm"] = max(v["max_vector_dev_mm"] for v in rec["variants"].values())      # 회독 양자화만
        rec["model_variant_spread_mm"] = round(float(np.linalg.norm(nom - nom[0], axis=1).max()), 2)           # 모델 변형 간 공칭 차(양자화 제외)
        rms, p95 = mc_dev(fk, qs, qe, TRUTH["best"], M); rec["mc_rms_mm"] = round(rms, 2); rec["mc_p95_mm"] = round(p95, 2)   # 최선 모델, 균등 반올림
        rec["quant_dev_over_tip_move"] = round(rec["quant_max_vector_dev_mm"] / e["tip_move_mm"], 3)
        out["pairs"][e["name"]] = rec
    out_path.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    for n, r in out["pairs"].items():
        b = r["variants"]["best"]
        print("%s tip %.1f mm: 공칭(최선) %s, 범위 x[%s,%s] y[%s,%s] z[%s,%s], 양자화 최대 벡터 편차 %.2f mm (이동량의 %.1f %%), 균등 반올림 RMS %.2f·95%% %.2f mm, 모델 변형 간 공칭 차 %.2f mm"
              % (n, r["tip_move_mm"], b["nominal_mm"], b["min_mm"][0], b["max_mm"][0], b["min_mm"][1], b["max_mm"][1], b["min_mm"][2], b["max_mm"][2], r["quant_max_vector_dev_mm"], r["quant_dev_over_tip_move"] * 100, r["mc_rms_mm"], r["mc_p95_mm"], r["model_variant_spread_mm"]))


if __name__ == "__main__":
    main()
