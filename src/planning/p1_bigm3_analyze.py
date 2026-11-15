"""③ 큰 이동 M3 `--real` 촬영의 이동량-오차 분석(오프라인, 이동 명령 없음).
규칙은 데이터를 보기 전에 `docs/gates/G07-큰이동-M3-승인-요청-문서.md` §3에 고정되어 있고 이 스크립트는 그대로 구현한다:
  쌍 = (S_k 촬영, 곧바로 이어진 E_k 촬영) 세 쌍(50·70·100 mm급). 오차 = |PnP 변위 − FK 예측 변위|(카메라 프레임 벡터 크기, 모델 변형 5종 범위 병기), 비율 = 오차 / tip 이동량.
  FK 예측 3종: (a) 정수 회독, (a′) 촬영 전·후 원시 서보 값 환산 평균, (b) 계획의 명령값. 축별 오차도 기록.
  M3-b = S 방문 쌍마다 (PnP 차 − 원시 값 FK 예측 차)의 크기 최대(잔차). 근거리 M3에서 데이터를 보기 전에 재정의: S₁은 쓰기 없이 접기 뒤 처진 J2로 촬영되어 PnP 차 자체가 실제 움직임을 포함한다. ‘판별 불가’: 오차가 합성 폭 이하. (a) √(양자화 95 % 폭² + M3-b²), (b) M3-b만(예측 쪽 반올림 없음).
  **사전 등록에 없는 부분(출력에 표시):** (a′)의 폭 — 규칙에 없어 양자화 95 % 폭을 원시 SD/반올림 SD(0.289°)로 비례 환산해 쓴다(원시 SD는 출발 전 점검 표본 평균 0.038°). 왜곡 보정 민감도(`--dist`)와 S 방문 간 예측 차는 참고 기록이다.
  (b)의 첫 S 방문: S₁의 J2는 쓰기 명령 없이 시작 회독값(92)이 ‘명령값’으로 기록되어 실제와 다를 수 있다(원시 90.25°). 명령값 기준은 S₁ 쌍에서 신뢰가 낮다.
사용: python3 -I src/planning/p1_bigm3_analyze.py <촬영 폴더(g01_s42_view 상위)> <실행 결과 json> [--dist] [out.json]"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from kinematics.urdf_fk import UrdfFK                      # noqa: E402
from planning.dry_run import TRUTH                         # noqa: E402
from planning import p1_m1_analyze as M1                   # noqa: E402
from planning.p1_bigm3_quant import cam_point, marker_world   # noqa: E402

RAW_SD_RATIO = 0.038 / 0.289   # 원시 SD(출발 전 점검 11:24 표본 평균 0.035·0.040·0.041·0.037) / 정수 반올림 SD
PAIRS = [(0, 1), (2, 3), (4, 5)]   # 실행 순서: S₁ E S₂ E S₃ E


def q4(d):
    return [float(d[str(k)]) for k in (1, 2, 3, 4)]


def raw_q(cap):
    """촬영 전·후 원시 환산 각도의 평균(J1~J4)."""
    return [float(np.mean([cap["raw_pre"][str(k)]["deg"], cap["raw_post"][str(k)]["deg"]])) for k in (1, 2, 3, 4)]


def main():
    argv = list(sys.argv[1:]); dist = "--dist" in argv
    opts = {}
    for flag in ("--poses", "--quant", "--ids", "--names", "--pairs"):   # 근거리 M3 등 다른 세트: --poses 자세json --quant 양자화json --ids 1501,1502,1501,1503,1501,1504 --names E5,E10,E15
        if flag in argv:
            i = argv.index(flag); opts[flag] = argv[i + 1]; del argv[i:i + 2]
    if dist:
        argv.remove("--dist"); M1.DIST = np.array(json.load(open(ROOT / "evidence/G02/dabai_color_camera_info.json"))["D"], float)
    folder = Path(argv[0]); res = json.load(open(argv[1], encoding="utf-8"))
    out_path = Path(argv[2]) if len(argv) > 2 else ROOT / ("evidence/G06/p1_bigm3_analysis_dist.json" if dist else "evidence/G06/p1_bigm3_analysis.json")
    fk = UrdfFK(); M = marker_world(); poses = res["poses"]
    ids = [p["pose_id"] for p in poses]; want = [int(x) for x in opts["--ids"].split(",")] if "--ids" in opts else [1401, 1402, 1401, 1403, 1401, 1404]; assert ids == want, ids
    names = opts["--names"].split(",") if "--names" in opts else ["E50", "E70", "E100"]
    quant = json.load(open(opts.get("--quant", ROOT / "evidence/G06/p1_bigm3_quant.json"), encoding="utf-8"))["pairs"]
    tips = {p["name"]: p["tip_move_mm"] for p in json.load(open(opts.get("--poses", ROOT / "evidence/G06/p1_bigm3_poses_s42.json"), encoding="utf-8"))["poses"] if "tip_move_mm" in p}
    # 촬영별 PnP 평균
    vis = []
    for p in poses:
        tag = p["capture"]["view_tag"]; files, rows = M1.analyze_pose(folder, tag); good = [r for r in rows if r]
        T = np.array([r["t"] for r in good]); vis.append({"tag": tag, "pose_id": p["pose_id"], "n_frames": len(files), "n_detected": len(good), "tvec_mean_mm": np.round(T.mean(0) * 1000, 3).tolist(), "tvec_std_mm": np.round(T.std(0) * 1000, 3).tolist(),
                                                           "reproj_px_mean": round(float(np.mean([r["reproj_px"] for r in good])), 3), "reproj_px_max": round(float(np.max([r["reproj_px"] for r in good])), 3), "_t": T.mean(0)})
    # M3-b(근거리 M3에서 데이터 전에 재정의): S 방문 쌍마다 (PnP 차 − 원시 값 FK 예측 차)의 크기 최대. S₁은 쓰기 명령 없이 접기 뒤 처진 J2(큰 이동 M3 실측 90.25° 대 S₂ 91.39°)로 촬영되어 PnP 차 자체는 실제 움직임을 포함하므로 예측 차를 뺀 잔차를 바닥으로 쓴다.
    sidx = [k for k, pid in enumerate(ids) if pid == ids[0]]; S = [vis[k]["_t"] for k in sidx]; qraw = [raw_q(poses[k]["capture"]) for k in sidx]
    m3b_pure = max(float(np.linalg.norm(S[a] - S[b])) * 1000 for a in range(len(S)) for b in range(a + 1, len(S)))   # 사전 정의 이전 방식(참고): PnP 위치 최대 쌍별 차
    resid = {}
    for a in range(len(S)):
        for b in range(a + 1, len(S)):
            pr = (cam_point(fk, qraw[b], TRUTH["best"], M) - cam_point(fk, qraw[a], TRUTH["best"], M)) * 1000
            resid["S%d-S%d" % (a + 1, b + 1)] = {"pnp_diff_mm": round(float(np.linalg.norm((S[b] - S[a]) * 1000)), 3), "raw_pred_diff_mm": round(float(np.linalg.norm(pr)), 3), "residual_mm": round(float(np.linalg.norm((S[b] - S[a]) * 1000 - pr)), 3)}
    m3b = max(v["residual_mm"] for v in resid.values())
    out = {"distortion": "dist(rational)" if dist else "none(P1과 같음)", "visits": [{k: v for k, v in x.items() if k != "_t"} for x in vis], "M3b_residual_mm": round(m3b, 3), "M3b_S_visit_pairs": resid, "M3b_pure_pnp_max_pairwise_mm_reference": round(m3b_pure, 3),
           
           "raw_sd_ratio": round(RAW_SD_RATIO, 3), "pairs": {}}
    pair_defs = []   # (i, j, 출력 이름, 이동량·양자화 키)
    if "--pairs" in opts:   # 예: 1-2:E5,2-3:E10,4-5:E15,0-1:E5_ref  (이름이 _ref로 끝나면 참고 쌍: 기준 판정 대상이 아니라는 표시)
        for tok in opts["--pairs"].split(","):
            ij, nm = tok.split(":"); i_, j_ = ij.split("-"); pair_defs.append((int(i_), int(j_), nm, nm[:-4] if nm.endswith("_ref") else nm))
    else:
        pair_defs = [(i_, j_, nm, nm) for (i_, j_), nm in zip(PAIRS, names)]
    for i, j, pname, name in pair_defs:
        s, e = poses[i], poses[j]; pnp = (vis[j]["_t"] - vis[i]["_t"]) * 1000
        srcs = {"a_int_readback": (q4(s["readback"]), q4(e["readback"])), "a2_raw": (raw_q(s["capture"]), raw_q(e["capture"])), "b_commanded": (q4(s["commanded"]), q4(e["commanded"]))}
        q95 = quant[name]["mc_p95_mm"]
        thr = {"a_int_readback": float(np.hypot(q95, m3b)), "a2_raw": float(np.hypot(q95 * RAW_SD_RATIO, m3b)), "b_commanded": float(m3b)}
        rec = {"reference_only": pname.endswith("_ref"), "from_to_visits": [i, j], "tip_move_mm": tips[name], "pnp_shift_mm": np.round(pnp, 2).tolist(), "quant_p95_mm_int": q95, "predictions": {}}
        for k, (qs, qe) in srcs.items():
            per = {}
            for vn, model in TRUTH.items():
                pred = (cam_point(fk, qe, model, M) - cam_point(fk, qs, model, M)) * 1000
                per[vn] = {"pred_mm": np.round(pred, 2).tolist(), "err_vec_mm": np.round(pnp - pred, 2).tolist(), "err_mm": round(float(np.linalg.norm(pnp - pred)), 2)}
            eb = per["best"]["err_mm"]; errs = [v["err_mm"] for v in per.values()]
            rec["predictions"][k] = {"q_S": [round(x, 3) for x in qs], "q_E": [round(x, 3) for x in qe], "best": per["best"], "err_range_over_models_mm": [min(errs), max(errs)], "ratio_to_tip_move": round(eb / tips[name], 3),
                                     "indistinguishable_threshold_mm": round(thr[k], 2), "indistinguishable": bool(eb <= thr[k]), "variants": per}
        out["pairs"][pname] = rec
    out_path.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    print("왜곡:", out["distortion"], "| 검출", [(v["tag"], "%d/%d" % (v["n_detected"], v["n_frames"])) for v in out["visits"]])
    print("M3-b(S 방문 쌍별 PnP 차 − 원시 FK 예측 차의 잔차 최대): %.3f mm  (참고: PnP 차 자체 최대 %.3f mm) %s" % (m3b, m3b_pure, resid))
    for n, r in out["pairs"].items():
        print("%s tip %.1f mm, PnP 변위 %s" % (n, r["tip_move_mm"], r["pnp_shift_mm"]))
        for k, v in r["predictions"].items():
            print("   %-15s 예측 %s 오차(최선) %.2f mm (모델 범위 %s) 비율 %.3f 기준 %.2f mm → %s" % (k, v["best"]["pred_mm"], v["best"]["err_mm"], v["err_range_over_models_mm"], v["ratio_to_tip_move"], v["indistinguishable_threshold_mm"], "판별 불가" if v["indistinguishable"] else "판별 가능"))


if __name__ == "__main__":
    main()
