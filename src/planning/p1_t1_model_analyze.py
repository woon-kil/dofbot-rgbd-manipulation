"""③ T1 서보 응답 모델 보정·홀드아웃 분석 라이브러리·CLI (오프라인, 이동 명령 없음).
**사전 등록 (T1 실기기 데이터 전에 고정; 모델 후보·선택 규칙은 세트 16 실기기 기록으로 개발했다):** docs/gates/G07-소각응답T1-승인-요청-문서.md §3, 변경 이력은 같은 문서 §3-1.
 행(row): 탐침 이동(phase T)마다 Δreq = 명령 − 직전 실제 위치(직전 이동 뒤 b 시점 원시 각도), Δact = 이번 b − 직전 b, prev_dir = 직전 이동의 명령 방향(위 +/아래 −).
 모델(관절별): M0 보정 없음(Δact = Δreq).
   M1 위: Δact = max(0, Δreq − d_up), 아래: Δact = Δreq − o_down.
   M2 M1을 직전 방향(위/아래)별로 따로(d_up, o_down 각 2개).
   M3 M2 + 크기 기울기: 위: 모자람 = d_up[p] + k_up·Δreq, 아래: 넘어감 = o_down[p] + k_down·|Δreq| (k 2개 추가, 움직인 이동으로 회귀, 위 모자람이 Δreq 이상이면 Δact = 0).
 선택: 보정용 이동의 **이동 묶음(4이동) 단위 leave-one-out** 예측 RMS가 가장 작은 모델(동률이면 매개변수가 적은 모델). 홀드아웃으로 고르지 않는다.
 평가: 선택한 모델로 홀드아웃(role "hold") 이동의 정지 위치(b)를 **한 걸음 앞 예측**(직전 실제 위치를 아는 상태에서 명령만으로)하고, 잔차 = 예측 정지 − 실제 정지(°)의 **RMS(편향 포함)**·SD·최대·평균, 90 % 상한(카이제곱 근사), tip 환산(감도 mm/°)을 보고. M0~M3 전부 병기.
 판정(관절별, RMS 기준): G1 시나리오 D 지지 = RMS ≤ 0.25° 그리고 최대 ≤ 0.6°, 90 % 상한 ≤ 0.40°(아니면 ‘G1 불확실’), G2 = RMS ≤ 0.40°, G3 = 그 밖.
 교차 자세 홀드아웃(role "xhold"): 보정은 한 자세에서 했고 다른 자세(J2: J3 22→14, J3: b 22→17)에서 한 묶음씩 예측한다. 모델 선택·판정에 쓰지 않고 같은 통계(RMS 등)를 따로 보고하며, 판정 수준(G1~G3)을 같은 임계값으로 표시하되 ‘표본 4이동·참고’로 쓴다(n이 작아 상한이 넓다). 교차 RMS가 같은 자세 홀드아웃 RMS보다 0.15° 넘게 크면 ‘자세 간 일반화 약함’으로 적는다.
 해석 범위: 이 결과만으로 6b 달성 여부를 판정하지 않는다(‘6b 판정 보류: J4·J1 미측정, 자세 2개’)."""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

SENS = {1: 2.65, 2: 2.52, 3: 3.46, 4: 2.87}   # mm/° (기준 B에서 FK 계산)
MODELS = ("M0", "M1", "M2", "M3")
NPAR = {"M0": 0, "M1": 2, "M2": 4, "M3": 6}


def rows_from_result(res):
    """실행 결과 json의 steps에서 탐침 행을 만든다. 직전 이동의 raw_after.b가 ‘직전 실제 위치’."""
    rows = []; prev = None; last_dir = {}
    for s in res["steps"]:
        ra = s.get("raw_after")
        if ra is None:
            prev = None; continue
        j = s["joint"]; cmd = float(s["target"]); cb = float(s["cmd_before"][str(j)]); dcmd = cmd - cb
        if s.get("phase") == "T" and prev is not None:
            p0 = prev["b"][str(j)]["deg"]; b = ra["b"][str(j)]["deg"]; a = ra["a"][str(j)]["deg"]
            if p0 is not None and b is not None:
                rows.append({"n": s["n"], "joint": j, "role": s.get("role"), "cmd": cmd, "prev_actual": p0, "dreq": cmd - p0, "dact": b - p0, "stop": b, "a": a,
                             "prev_dir": last_dir.get(j, 0), "cmd_dir": 1 if dcmd > 0 else (-1 if dcmd < 0 else 0)})
        if dcmd != 0:
            last_dir[j] = 1 if dcmd > 0 else -1
        prev = ra
    return rows


def _dirclass(r):
    return "up" if r["dreq"] > 0.5 else ("down" if r["dreq"] < -0.5 else "none")


def _p(r):
    return 1 if r["prev_dir"] >= 0 else -1   # 직전 방향(0은 위로 취급: 접기 뒤 첫 이동)


def fit(model, cal):
    """보정용 행으로 매개변수를 적합. 반환: dict(model, params)."""
    P = {"model": model}
    if model == "M0":
        return P
    up = [r for r in cal if _dirclass(r) == "up"]; dn = [r for r in cal if _dirclass(r) == "down"]

    def d_up_fit(rs):   # 1차원 격자: Δact = max(0, Δreq − d)의 SSE 최소
        if not rs: return 0.0
        grid = np.arange(0.0, 5.0001, 0.01); sse = [sum((r["dact"] - max(0.0, r["dreq"] - d)) ** 2 for r in rs) for d in grid]
        return float(grid[int(np.argmin(sse))])
    o_fit = lambda rs: float(np.mean([r["dreq"] - r["dact"] for r in rs])) if rs else 0.0
    if model == "M1":
        P.update(d_up=d_up_fit(up), o_down=o_fit(dn)); return P
    split = lambda rs, p: [r for r in rs if _p(r) == p]
    d2 = {p: (d_up_fit(split(up, p)) if len(split(up, p)) >= 2 else d_up_fit(up)) for p in (1, -1)}
    o2 = {p: (o_fit(split(dn, p)) if len(split(dn, p)) >= 2 else o_fit(dn)) for p in (1, -1)}
    if model == "M2":
        P.update(d_up=d2, o_down=o2); return P
    # M3: 움직인 이동으로 모자람/넘어감을 (직전 방향 지시자 + 크기) 회귀
    def reg(rs, sign):
        mv = [r for r in rs if abs(r["dact"]) > 0.3]
        if len(mv) < 4: return None
        X = np.array([[1.0 if _p(r) == 1 else 0.0, 1.0 if _p(r) == -1 else 0.0, abs(r["dreq"])] for r in mv]); y = np.array([r["dreq"] - r["dact"] for r in mv])
        coef, *_ = np.linalg.lstsq(X, y, rcond=None); return [float(c) for c in coef]
    cu, cd = reg(up, 1), reg(dn, -1)
    P.update(up=cu if cu else [d2[1], d2[-1], 0.0], down=cd if cd else [o2[1], o2[-1], 0.0]); return P


def predict_dact(P, r):
    m = P["model"]; dq = r["dreq"]; c = _dirclass(r)
    if m == "M0" or c == "none": return dq
    p = _p(r)
    if m == "M1": return max(0.0, dq - P["d_up"]) if c == "up" else dq - P["o_down"]
    if m == "M2": return max(0.0, dq - P["d_up"][p]) if c == "up" else dq - P["o_down"][p]
    co = P["up"] if c == "up" else P["down"]; sh = (co[0] if p == 1 else co[1]) + co[2] * abs(dq)
    return max(0.0, dq - sh) if c == "up" else dq - sh


def predicted_stop(P, r):
    return r["prev_actual"] + predict_dact(P, r)


def bundles(rows_joint):
    """관절의 탐침 행을 이동 순서대로 4개씩 묶는다(b+s, b, b−s, b)."""
    return [rows_joint[i:i + 4] for i in range(0, len(rows_joint), 4)]


def cv_rms(model, cal_joint):
    """보정용 이동 묶음 단위 leave-one-out 예측 RMS(°)."""
    bs = bundles(cal_joint); res = []
    for k in range(len(bs)):
        train = [r for i, b in enumerate(bs) if i != k for r in b]
        P = fit(model, train)
        res += [predicted_stop(P, r) - r["stop"] for r in bs[k]]
    return float(np.sqrt(np.mean(np.square(res)))) if res else float("nan")


def select_model(cal_joint):
    sc = {m: cv_rms(m, cal_joint) for m in MODELS}
    best = min(MODELS, key=lambda m: (round(sc[m], 4), NPAR[m]))   # 동률은 매개변수 적은 모델
    return best, sc


def residual_stats(P, rows_eval, joint):
    res = np.array([predicted_stop(P, r) - r["stop"] for r in rows_eval])
    n = len(res); rms = float(np.sqrt(np.mean(res ** 2))); sd = float(np.std(res, ddof=1)) if n > 1 else float("nan")
    # RMS의 90 % 상한(카이제곱 근사): sqrt(n·rms² / χ²_{0.10, n}); χ²_{0.10,n}은 Wilson–Hilferty 근사
    z = -1.2816; chi = n * (1 - 2 / (9 * n) + z * np.sqrt(2 / (9 * n))) ** 3; upper = float(np.sqrt(n * rms ** 2 / chi)) if n > 0 and chi > 0 else float("nan")
    return {"n": n, "rms": round(rms, 3), "sd": round(sd, 3), "max_abs": round(float(np.max(np.abs(res))), 3), "mean": round(float(np.mean(res)), 3), "rms_upper90": round(upper, 3), "rms_tip_mm": round(rms * SENS[joint], 2)}


def verdict(st):
    if st["n"] == 0: return "판정 불가"
    if st["rms"] <= 0.25 and st["max_abs"] <= 0.6: return "G1 시나리오 D 지지" if st["rms_upper90"] <= 0.40 else "G1 불확실(90 % 상한 > 0.40°)"
    if st["rms"] <= 0.40: return "G2 부분"
    return "G3 미달"


def analyze(res):
    rows = rows_from_result(res); out = {"n_rows": len(rows), "joints": {}}
    for j in sorted({r["joint"] for r in rows}):
        rj = [r for r in rows if r["joint"] == j]; cal = [r for r in rj if r["role"] == "cal"]; hold = [r for r in rj if r["role"] == "hold"]
        best, cv = select_model(cal); fits = {m: fit(m, cal) for m in MODELS}
        info = {"n_cal": len(cal), "n_hold": len(hold), "cv_rms": {m: round(cv[m], 3) for m in MODELS}, "selected": best, "params": fits[best],
                "holdout": {m: residual_stats(fits[m], hold, j) for m in MODELS} if hold else None}
        xh = [r for r in rj if r["role"] == "xhold"]
        if xh:
            info["n_xhold"] = len(xh); info["xhold"] = {m: residual_stats(fits[m], xh, j) for m in MODELS}; info["selected_xhold"] = info["xhold"][best]; info["xhold_verdict"] = verdict(info["selected_xhold"])
            if hold: info["generalization_weak"] = bool(info["selected_xhold"]["rms"] - info["holdout"][best]["rms"] > 0.15)
        if hold:
            info["selected_holdout"] = info["holdout"][best]; info["verdict"] = verdict(info["selected_holdout"])
        out["joints"][str(j)] = info
    out["interpretation_scope"] = "6b 판정 보류: J4·J1 미측정, 기준 자세 2개 — 이 결과만으로 6b 달성 여부를 판정하지 않는다"
    return out


def main():
    res = json.load(open(sys.argv[1], encoding="utf-8")); out = analyze(res)
    if len(sys.argv) > 2: Path(sys.argv[2]).write_text(json.dumps(out, ensure_ascii=False, indent=1, default=float), encoding="utf-8")
    for j, i in out["joints"].items():
        print("J%s: 보정 %d 홀드아웃 %d | CV RMS %s → 선택 %s" % (j, i["n_cal"], i["n_hold"], i["cv_rms"], i["selected"]))
        if i.get("selected_holdout"): print("    홀드아웃(선택 모델) %s → %s" % (i["selected_holdout"], i["verdict"]))
        if i.get("selected_xhold"): print("    교차 자세 홀드아웃(참고, n=%d) %s → %s%s" % (i["n_xhold"], i["selected_xhold"], i["xhold_verdict"], " · 자세 간 일반화 약함" if i.get("generalization_weak") else ""))
    print(out["interpretation_scope"])


if __name__ == "__main__":
    main()
