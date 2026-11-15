"""③ 소각 응답 시험(세트 16) 분석(오프라인, 이동 명령 없음).
**사전 등록(데이터를 보기 전에 고정, docs/gates/G07-소각응답-승인-요청-문서.md §3과 같음):**
 입력: 실행 결과 json의 steps. 탐침 이동(phase "T")마다 직전 이동 뒤 원시 값(`raw_after.b`, 2초 뒤)을 ‘실제 위치’로, 이번 이동 뒤 원시 값 두 시점(a: 이동 직후, b: 2초 뒤)을 쓴다.
 주 변수: Δreq = 명령 − 직전 실제 위치(부호 있음, 방향은 Δreq의 부호), Δact = 이번 b − 직전 b, 정지 오차 e = b − 명령, 모자람 = Δreq − Δact, creep = b − a.
 ‘움직임 없음’: |Δreq| ≥ 1° 이고 |Δact| < 0.3°(원시 SD 0.04°의 약 7배).
 방향별 지표: 정지 오차 e의 평균·SD(Δreq > 0.5 위, < −0.5 아래), 크기 구간(|Δreq| 0.5~1.5, 1.5~2.5, 2.5~3.5, ≥ 3.5)별 실현 비율 Δact/Δreq와 ‘움직임 없음’ 수, 같은 (관절, 명령, 직전 명령) 반복 쌍의 |e₁ − e₂|, creep 중앙값.
 가설(관절별로 판정): H1 부하 오프셋 — e_up ≤ −0.5° 그리고 e_down ≤ −0.5° 그리고 |e_up − e_down| < 0.5°; H2 방향별 정지 대역 — e_down − e_up ≥ 0.5°; H3 쓰기 누락 — 위·아래 모두에서 |Δreq| ≥ 2°인 이동의 ‘움직임 없음’ 비율 ≥ 25 %. 위 순서로 처음 맞는 것(H3은 H1·H2와 독립으로도 표시), 아무것도 아니면 ‘정상(편차 없음)’ 또는 ‘미분류’.
 최소 유효 폭(방향별): 크기 구간 중 실현 비율 중앙값이 처음 ≥ 0.5인 구간(없으면 None). 공통 성분 = (e_up + e_down)/2(양방향 공통으로 명령보다 아래에 멈추는 성분, 보정 전략용), 대역 반폭 = (e_down − e_up)/2. 두 시점의 실제 시각: a = 쓰기 후 약 3.3~3.8 s(watch 창 1.5×1500+1000 ms 뒤 원시 5회 평균), b = 쓰기 후 약 5.8~6.3 s(a 뒤 2 s 대기 후 5회 평균). 판정 임계값은 위와 같고 바뀌지 않는다.
 참고: 모자람의 tip 환산(FK, 명령 상태 대 실제 원시 상태).
사용: python3 -I src/planning/p1_step_response_analyze.py <결과 json> [out.json]"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

NO_MOVE_REQ = 1.0
NO_MOVE_ACT = 0.3
BINS = [(0.5, 1.5), (1.5, 2.5), (2.5, 3.5), (3.5, 99.0)]


def _deg(raw_after, key, j):
    return raw_after[key][str(j)]["deg"]


def analyze(res):
    steps = res["steps"]; rows = []; prev = None   # prev: 직전 이동 뒤 raw_after
    for s in steps:
        ra = s.get("raw_after")
        if ra is None:
            prev = None; continue
        if s.get("phase") == "T" and prev is not None:
            j = s["joint"]; cmd = float(s["target"]); p0 = _deg(prev, "b", j); a = _deg(ra, "a", j); b = _deg(ra, "b", j)
            if None not in (p0, a, b):
                dreq = cmd - p0; dact = b - p0
                rows.append({"n": s["n"], "joint": j, "cmd": cmd, "cmd_before": float(s["cmd_before"][str(j)]), "prev_actual": round(p0, 3), "raw_a": round(a, 3), "raw_b": round(b, 3),
                             "dreq": round(dreq, 3), "dact": round(dact, 3), "stop_err": round(b - cmd, 3), "shortfall": round(dreq - dact, 3), "creep": round(b - a, 3),
                             "dir": "up" if dreq > 0.5 else ("down" if dreq < -0.5 else "none"), "no_move": bool(abs(dreq) >= NO_MOVE_REQ and abs(dact) < NO_MOVE_ACT),
                             "fraction": None if abs(dreq) < 0.5 else round(dact / dreq, 3)})
        prev = ra
    out = {"n_probe_moves": len(rows), "moves": rows, "joints": {}}
    for j in sorted({r["joint"] for r in rows}):
        R = [r for r in rows if r["joint"] == j]; info = {"n": len(R)}
        for d in ("up", "down"):
            E = [r["stop_err"] for r in R if r["dir"] == d]
            info[d] = {"n": len(E), "stop_err_mean": round(float(np.mean(E)), 3) if E else None, "stop_err_sd": round(float(np.std(E)), 3) if E else None,
                       "shortfall_median": round(float(np.median([r["shortfall"] for r in R if r["dir"] == d])), 3) if E else None}
            bins = []
            for lo, hi in BINS:
                B = [r for r in R if r["dir"] == d and lo <= abs(r["dreq"]) < hi]
                bins.append({"abs_dreq_range": [lo, hi], "n": len(B), "n_no_move": sum(1 for r in B if r["no_move"]), "fraction_median": round(float(np.median([r["fraction"] for r in B])), 3) if B else None})
            info[d]["bins"] = bins
        # 반복 쌍(같은 관절·명령·직전 명령): |e1 − e2|
        seen = {}
        for r in R: seen.setdefault((r["cmd"], r["cmd_before"]), []).append(r["stop_err"])
        info["repeat_abs_diff"] = [round(abs(v[0] - v[1]), 3) for v in seen.values() if len(v) >= 2]
        info["creep_median_abs"] = round(float(np.median([abs(r["creep"]) for r in R])), 3)
        eu, ed = info["up"]["stop_err_mean"], info["down"]["stop_err_mean"]
        big = [r for r in R if abs(r["dreq"]) >= 2.0]
        frac = lambda d: (sum(1 for r in big if r["dir"] == d and r["no_move"]) / max(1, sum(1 for r in big if r["dir"] == d)))
        h1 = eu is not None and ed is not None and eu <= -0.5 and ed <= -0.5 and abs(eu - ed) < 0.5
        h2 = eu is not None and ed is not None and (ed - eu) >= 0.5
        h3 = frac("up") >= 0.25 and frac("down") >= 0.25
        for d in ("up", "down"):   # 최소 유효 폭: 실현 비율 중앙값이 처음 ≥ 0.5인 크기 구간
            hit = [tuple(b["abs_dreq_range"]) for b in info[d]["bins"] if b["n"] > 0 and b["fraction_median"] is not None and b["fraction_median"] >= 0.5]
            info[d]["min_effective_width_bin"] = hit[0] if hit else None
        info["common_component"] = None if eu is None or ed is None else round((eu + ed) / 2, 3)
        info["band_halfwidth"] = None if eu is None or ed is None else round((ed - eu) / 2, 3)
        info["hypotheses"] = {"H1_load_offset": bool(h1), "H2_direction_dependent_band": bool(h2), "H3_write_miss": bool(h3), "no_move_fraction_ge2deg": {"up": round(frac("up"), 3), "down": round(frac("down"), 3)}}
        info["verdict"] = "H2 방향별 정지 대역" if h2 else ("H1 부하 오프셋" if h1 else ("H3 쓰기 누락" if h3 else ("정상(편차 없음)" if (eu is not None and ed is not None and abs(eu) < 0.5 and abs(ed) < 0.5) else "미분류")))
        out["joints"][str(j)] = info
    return out


def main():
    res = json.load(open(sys.argv[1], encoding="utf-8")); out = analyze(res)
    if len(sys.argv) > 2:
        Path(sys.argv[2]).write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    print("탐침 이동 %d개" % out["n_probe_moves"])
    for j, info in out["joints"].items():
        print("J%s: 위 e=%s±%s(n=%d, 모자람 중앙값 %s) 아래 e=%s±%s(n=%d, 모자람 중앙값 %s) | 공통 성분 %s, 대역 반폭 %s | 최소 유효 폭 구간 위 %s 아래 %s | 반복 쌍 |e₁−e₂| %s | creep 중앙값 %s | → %s %s" % (j, info["up"]["stop_err_mean"], info["up"]["stop_err_sd"], info["up"]["n"], info["up"]["shortfall_median"],
              info["down"]["stop_err_mean"], info["down"]["stop_err_sd"], info["down"]["n"], info["down"]["shortfall_median"], info["common_component"], info["band_halfwidth"], info["up"]["min_effective_width_bin"], info["down"]["min_effective_width_bin"], info["repeat_abs_diff"], info["creep_median_abs"], info["verdict"], info["hypotheses"]["no_move_fraction_ge2deg"]))
        for d in ("up", "down"):
            print("    %s 크기 구간별 [n, 움직임 없음, 실현 비율 중앙값]: %s" % (d, [(tuple(b["abs_dreq_range"]), b["n"], b["n_no_move"], b["fraction_median"]) for b in info[d]["bins"]]))


if __name__ == "__main__":
    main()
