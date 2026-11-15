"""③ T1-C 크기 구간·이력 확인 분석(오프라인, 이동 명령 없음).
**사전 등록(데이터를 보기 전에 고정, docs/gates/G07-소각응답T1C-크기구간-승인-요청-문서.md §1·§3, E2 축소안):**
 입력: 세트 19 실행 결과 json(role "mix", 관절당 20이동: 크기 8·12·16·20·30° 각 위2·아래2, 섞인 순서·종점). 행·모델·잔차 정의는 T1-A 분석(`p1_t1_model_analyze.py`, sha 앞 16자 7375eabe9c0bbe27)을 그대로 불러 쓴다(수정하지 않는다).
 예측 모델은 **다시 맞추지 않는다**: T1-B 분석(`evidence/G06/p1_t1b_real_analysis_261115.json`)의 선택 모델·매개변수를 아래 T1B_PARAMS에 고정(J2 M3, J3 M1).
 크기 s = |명령 − 직전 명령|(계획 크기). 구간: 작은 {8, 12}, 큰 {16, 20, 30}(E2 축소안으로 문서의 ‘중간’ 구간은 큰 구간에 합침).
 C1: 큰 이동(s ≥ 16, 관절당 12이동)의 잔차(예측 정지 − 실제 정지) RMS ≤ 0.25° 그리고 max ≤ 0.6° → ‘큰 이동 전이 성립(G1 수준)’; RMS ≤ 0.40° → ‘부분’; 그 밖 → ‘전이 실패(이력 또는 모델 불일치)’.
 C2: (구간, 방향)별 셀에서 정지 오차(실제 정지 − 명령)의 평균을 뺀 표본 SD(n < 2면 표시 안 함). 가설 H2: 아래 이동에서 SD(작은) ≥ 2 × SD(큰) → ‘크기 전환 지지’, 아니면 ‘크기 전환 미지지’. 판정은 참고(셀 n 4~6).
 C3: 큰 이동에서 직전 방향이 같은/반대인 이동의 잔차 RMS를 보고(판정 없음).
 보조(판정 아님): 작은 이동(8·12°)에 T1-B 모델을 적용한 잔차 RMS.
 해석 범위: ‘6b 판정 보류: J4·J1 미측정, 자세 의존 미해결(J2는 90 이상 구간만 시험)’ — 이 결과만으로 6b 달성 여부를 판정하지 않는다.
사용: python3 -I src/planning/p1_t1c_analyze.py <결과 json> [out.json]"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "planning"))
import p1_t1_model_analyze as A   # noqa: E402

T1B_PARAMS = {
    # T1-B(왕복 쌍 설계)는 ‘위, 직전 위’와 ‘아래, 직전 아래’ 칸을 식별하지 못해 0.0으로 남겼다. 데이터 전 정정(문서 §6-c): 그 칸은 같은 방향의 식별된 계수를 쓴다(이력 무관 가정, C3가 이 가정을 시험한다).
    2: {"model": "M3", "up": [1.8859367533485132, 1.8859367533485132, -0.02906568174052299], "down": [0.7382681186464963, 0.7382681186464963, 0.0020461804499007892]},
    3: {"model": "M1", "d_up": 1.56, "o_down": 0.6433749999999998},
}
SMALL = (8, 12)
LARGE = (16, 20, 30)


def rows_with_size(res):
    rows = A.rows_from_result(res); size = {}
    for s in res["steps"]:
        j = s["joint"]
        if s.get("phase") == "T": size[s["n"]] = abs(float(s["target"]) - float(s["cmd_before"][str(j)]))
    for r in rows: r["size"] = int(round(size.get(r["n"], float("nan")))) if r["n"] in size else None
    return [r for r in rows if r["role"] == "mix"]


def cell_sd(rows):
    """(구간, 방향) 셀의 정지 오차(실제 − 명령) 평균을 뺀 SD."""
    out = {}
    for grp, sizes in (("small", SMALL), ("large", LARGE)):
        for d in ("up", "down"):
            E = [r["stop"] - r["cmd"] for r in rows if r["size"] in sizes and A._dirclass(r) == d]
            out["%s_%s" % (grp, d)] = {"n": len(E), "mean": round(float(np.mean(E)), 3) if E else None, "sd": round(float(np.std(E, ddof=1)), 3) if len(E) > 1 else None}
    return out


def analyze(res):
    rows = rows_with_size(res); out = {"n_rows": len(rows), "joints": {}}
    for j in sorted({r["joint"] for r in rows}):
        rj = [r for r in rows if r["joint"] == j]; P = T1B_PARAMS[j]
        big = [r for r in rj if r["size"] in LARGE]; small = [r for r in rj if r["size"] in SMALL]
        info = {"n": len(rj), "n_large": len(big), "n_small": len(small)}
        st = A.residual_stats(P, big, j) if big else None; info["c1_large"] = st
        if st: info["c1_verdict"] = "큰 이동 전이 성립(G1 수준)" if (st["rms"] <= 0.25 and st["max_abs"] <= 0.6) else ("부분(RMS ≤ 0.40°)" if st["rms"] <= 0.40 else "전이 실패(이력 또는 모델 불일치)")
        cs = cell_sd(rj); info["c2_cells"] = cs
        sd_s, sd_l = cs["small_down"]["sd"], cs["large_down"]["sd"]
        info["c2_ratio_down_small_over_large"] = round(sd_s / sd_l, 2) if (sd_s is not None and sd_l) else None
        info["c2_verdict"] = None if info["c2_ratio_down_small_over_large"] is None else ("크기 전환 지지(SD 비 ≥ 2)" if info["c2_ratio_down_small_over_large"] >= 2 else "크기 전환 미지지")
        c3 = {}
        for same in (True, False):
            R = [r for r in big if r["prev_dir"] != 0 and ((r["dreq"] > 0) == (r["prev_dir"] > 0)) == same]
            c3["same_dir" if same else "opposite_dir"] = A.residual_stats(P, R, j) if R else None
        info["c3_history_large"] = c3
        info["aux_small_transfer"] = A.residual_stats(P, small, j) if small else None
        out["joints"][str(j)] = info
    out["interpretation_scope"] = "6b 판정 보류: J4·J1 미측정, 자세 의존 미해결(J2는 90 이상 구간만 시험) — 이 결과만으로 6b 달성 여부를 판정하지 않는다"
    return out


def main():
    res = json.load(open(sys.argv[1], encoding="utf-8")); out = analyze(res)
    if len(sys.argv) > 2: Path(sys.argv[2]).write_text(json.dumps(out, ensure_ascii=False, indent=1, default=float), encoding="utf-8")
    for j, i in out["joints"].items():
        print("J%s: 이동 %d (큰 %d, 작은 %d)" % (j, i["n"], i["n_large"], i["n_small"]))
        if i.get("c1_large"): print("    C1 큰 이동 전이(T1-B 모델 그대로): %s → %s" % ({k: i["c1_large"][k] for k in ("n", "rms", "max_abs", "mean", "rms_upper90")}, i["c1_verdict"]))
        print("    C2 셀(정지 오차 평균 제거 SD): %s | 아래 이동 SD 비(작은/큰) %s → %s" % (i["c2_cells"], i["c2_ratio_down_small_over_large"], i["c2_verdict"]))
        print("    C3 큰 이동 이력별 잔차: %s" % {k: (None if v is None else {a: v[a] for a in ("n", "rms", "mean")}) for k, v in i["c3_history_large"].items()})
        if i.get("aux_small_transfer"): print("    보조 작은 이동(8·12°)에 T1-B 모델 적용: %s" % {a: i["aux_small_transfer"][a] for a in ("n", "rms", "max_abs", "mean")})
    print(out["interpretation_scope"])


if __name__ == "__main__":
    main()
