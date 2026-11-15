"""③ T1-B 큰 이동 응답 분석(오프라인, 이동 명령 없음).
**사전 등록(데이터를 보기 전에 고정, docs/gates/G07-소각응답T1B-큰이동-승인-요청-문서.md §1·§3):**
 입력: 세트 18 실행 결과 json. 행·모델(M0~M3)·적합·G1/G2/G3 판정·RMS 계산은 T1-A 분석(`p1_t1_model_analyze.py`, sha 앞 16자 7375eabe9c0bbe27)을 그대로 불러 쓴다(수정하지 않는다).
 달라지는 점: ① 이동 묶음 = **왕복 한 쌍(위 b→b+s, 아래 b+s→b) 2이동**(T1-A는 4이동), 모델 선택은 쌍 단위 leave-one-out. ② 크기: 보정용 s = 15·25·35·45(각 2번), 홀드아웃 s = 20·30·40(각 2번), 관절 J2(b=90)·J3(b=22), 이동 시간 max(1500 ms, 60 ms/°×s)(문서 §2-4-1).
 Q1(전이): T1-A 보정값(고정, 아래 T1A_PARAMS; `evidence/G06/p1_t1_real_analysis_261115.json`의 선택 모델 M1 매개변수)을 **다시 맞추지 않고** 홀드아웃 이동의 정지 위치를 예측한 잔차 RMS ≤ 0.40° → ‘전이 성립’, 그 밖 → ‘전이 실패(큰 이동에서 모델이 바뀐다)’. 보정용 이동에도 같은 통계를 참고로 보고한다.
 Q2(재적합): 큰 이동 보정용 행으로 M0~M3 적합, 쌍 단위 LOO RMS 최소 모델 선택, 홀드아웃 RMS·max·90 % 상한으로 T1-A와 같은 판정(G1 RMS ≤ 0.25°·max ≤ 0.6°·상한 ≤ 0.40° / G2 RMS ≤ 0.40° / G3).
 Q3(잔차 SD): 선택 모델의 홀드아웃 잔차 SD를 T1-A 홀드아웃 SD(J2 0.34°, J3 0.226°)와 나눈 비를 보고한다(판정 없음, 6b 한도 계산의 입력).
 보조(탐색, 판정 아님): 크기 구간별 전이 잔차(평균·RMS), 방향별(위/아래) 전이 잔차.
 해석 범위: ‘6b 판정 보류: J4·J1 미측정, 자세 의존 미해결(J2는 90 이상 구간만 시험)’ — 이 결과만으로 6b 달성 여부를 판정하지 않는다.
사용: python3 -I src/planning/p1_t1b_analyze.py <결과 json> [out.json]"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "planning"))
import p1_t1_model_analyze as A   # noqa: E402

T1A_PARAMS = {2: {"model": "M1", "d_up": 1.95, "o_down": 0.216}, 3: {"model": "M1", "d_up": 1.88, "o_down": 0.252}}   # T1-A 선택 모델(반올림 값, 2026-11-15 17:5x 분석)
T1A_HOLD_SD = {2: 0.34, 3: 0.226}
Q1_RMS = 0.40


def pairs(rows_joint):
    """관절의 탐침 행을 이동 순서대로 2개씩(위·아래 왕복 한 쌍) 묶는다."""
    return [rows_joint[i:i + 2] for i in range(0, len(rows_joint), 2)]


def cv_rms_pairs(model, cal):
    bs = pairs(cal); res = []
    for k in range(len(bs)):
        train = [r for i, b in enumerate(bs) if i != k for r in b]
        P = A.fit(model, train)
        res += [A.predicted_stop(P, r) - r["stop"] for r in bs[k]]
    return float(np.sqrt(np.mean(np.square(res)))) if res else float("nan")


def select_pairs(cal):
    sc = {m: cv_rms_pairs(m, cal) for m in A.MODELS}
    return min(A.MODELS, key=lambda m: (round(sc[m], 4), A.NPAR[m])), sc


def transfer(P, rows, j):
    if not rows: return None
    st = A.residual_stats(P, rows, j); st["verdict_q1"] = "전이 성립(RMS ≤ %.2f°)" % Q1_RMS if st["rms"] <= Q1_RMS else "전이 실패(큰 이동에서 모델이 바뀐다)"
    return st


def by_size(P, rows):
    out = {}
    for r in rows:
        s = int(round(abs(r["dreq"])) // 5 * 5)   # 5° 구간
        out.setdefault(str(s), []).append(A.predicted_stop(P, r) - r["stop"])
    return {k: {"n": len(v), "mean": round(float(np.mean(v)), 3), "rms": round(float(np.sqrt(np.mean(np.square(v)))), 3)} for k, v in sorted(out.items(), key=lambda kv: int(kv[0]))}


def by_dir(P, rows):
    out = {}
    for r in rows:
        out.setdefault(A._dirclass(r), []).append(A.predicted_stop(P, r) - r["stop"])
    return {k: {"n": len(v), "mean": round(float(np.mean(v)), 3), "rms": round(float(np.sqrt(np.mean(np.square(v)))), 3)} for k, v in out.items()}


def analyze(res):
    rows = A.rows_from_result(res); out = {"n_rows": len(rows), "joints": {}}
    for j in sorted({r["joint"] for r in rows}):
        rj = [r for r in rows if r["joint"] == j]; cal = [r for r in rj if r["role"] == "cal"]; hold = [r for r in rj if r["role"] == "hold"]
        best, cv = select_pairs(cal); fits = {m: A.fit(m, cal) for m in A.MODELS}
        info = {"n_cal": len(cal), "n_hold": len(hold), "cv_rms": {m: round(cv[m], 3) for m in A.MODELS}, "selected": best, "params": fits[best],
                "q1_transfer_hold": transfer(T1A_PARAMS[j], hold, j), "q1_transfer_cal": transfer(T1A_PARAMS[j], cal, j),
                "transfer_by_size_dreq_bin5": by_size(T1A_PARAMS[j], cal + hold), "transfer_by_dir": by_dir(T1A_PARAMS[j], cal + hold)}
        if hold:
            info["holdout"] = {m: A.residual_stats(fits[m], hold, j) for m in A.MODELS}; info["selected_holdout"] = info["holdout"][best]; info["verdict_q2"] = A.verdict(info["selected_holdout"])
            info["q3_sd_ratio_vs_t1a"] = round(info["selected_holdout"]["sd"] / T1A_HOLD_SD[j], 2)
        out["joints"][str(j)] = info
    out["interpretation_scope"] = "6b 판정 보류: J4·J1 미측정, 자세 의존 미해결(J2는 90 이상 구간만 시험) — 이 결과만으로 6b 달성 여부를 판정하지 않는다"
    return out


def main():
    res = json.load(open(sys.argv[1], encoding="utf-8")); out = analyze(res)
    if len(sys.argv) > 2: Path(sys.argv[2]).write_text(json.dumps(out, ensure_ascii=False, indent=1, default=float), encoding="utf-8")
    for j, i in out["joints"].items():
        print("J%s: 보정 %d 홀드아웃 %d | CV RMS %s → 선택 %s %s" % (j, i["n_cal"], i["n_hold"], i["cv_rms"], i["selected"], {k: (round(v, 3) if isinstance(v, float) else v) for k, v in i["params"].items()}))
        print("    Q1 전이(T1-A 값 그대로): 홀드아웃 %s → %s | 보정 행 RMS %s" % ({k: i["q1_transfer_hold"][k] for k in ("n", "rms", "max_abs", "mean")}, i["q1_transfer_hold"]["verdict_q1"], i["q1_transfer_cal"]["rms"]))
        print("    Q2 재적합 홀드아웃(선택 모델) %s → %s | Q3 잔차 SD 비(T1-A 대비) %s" % (i["selected_holdout"], i["verdict_q2"], i.get("q3_sd_ratio_vs_t1a")))
        print("    보조 크기 구간별 전이 잔차(예측−실제) %s | 방향별 %s" % (i["transfer_by_size_dreq_bin5"], i["transfer_by_dir"]))
    print(out["interpretation_scope"])


if __name__ == "__main__":
    main()
