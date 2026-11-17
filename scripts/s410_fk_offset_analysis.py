#!/usr/bin/env python3
"""FK 오프셋 분석(오프라인, 하드웨어 없음): 손끝 최저점 실측(높이·y)과 손가락 축 각도를 몇 개의 모델로 설명할 수 있는지 판정한다.
모델: M1 TCP 프레임 고정점 오프셋(3) + y 공통 편향, M2 J2·J3·J4 영점 오프셋(3) + y 공통 편향, M3 −z축 오프셋 1개(H1 계열), M4 접근축(+y) 오프셋 1개(H3 계열), M5 J2 이득·영점(자세 의존 오차 후보). 모든 모델에 y 공통 편향과 손가락 각 공통 오프셋 β(±10°)를 둔다.
출력: evidence/G02/fk_offset_analysis.json, 표준 출력. 사용: python3 scripts/s410_fk_offset_analysis.py"""
import sys, json, itertools
import numpy as np
from scipy.optimize import least_squares
src = open("scripts/g01_pose_generator.py", encoding="utf-8").read().split("# 후보 격자")[0]
sys.argv = ["x"]; ns = {"__file__": "scripts/g01_pose_generator.py"}; exec(compile(src, "pg", "exec"), ns); fk = ns["fk"]
D = json.load(open("evidence/G02/fk_offset_data.json")); TOL = D["tolerance"]
Q = {k: np.array(v, float) for k, v in D["commanded_sdk_j1_j5"].items()}
def tcp(q, dj=(0, 0, 0)):
    qq = q.copy(); qq[1] += dj[0]; qq[2] += dj[1]; qq[3] += dj[2]; f = fk(qq[None, :]); return f["tcp"][0][0] * 1000, f["tcp"][1][0]
def axis_angle(R):   # 접근축(TCP +y)의 y–z 투영 각(세로 기준, +y 쪽 양수)
    v = R[:, 1]; v = v if v[2] < 0 else -v; return float(np.degrees(np.arctan2(v[1], -v[2])))
def predict(model, thf, pid, q=None):
    q = Q[pid] if q is None else q; th = thf[:-1]; beta = thf[-1]   # 마지막 원소 = 손가락 각 공통 오프셋 β(°)
    r = _predict(model, th, q); return r[0], r[1], r[2] + beta
def _predict(model, th, q):
    if model == "M1":
        p, R = tcp(q); lp = p + R @ th[:3]; return lp[2], lp[1] + th[3], axis_angle(R)
    if model == "M2":
        p, R = tcp(q, th[:3]); return p[2], p[1] + th[3], axis_angle(R)
    if model == "M3":
        p, R = tcp(q); lp = p - th[0] * R[:, 2]; return lp[2], lp[1] + th[1], axis_angle(R)
    if model == "M4":
        p, R = tcp(q); lp = p + th[0] * R[:, 1]; return lp[2], lp[1] + th[1], axis_angle(R)
    if model == "M5":
        qq = q.copy(); qq[1] = 90 + th[0] * (q[1] - 90) + th[1]; f = fk(qq[None, :]); p = f["tcp"][0][0] * 1000; R = f["tcp"][1][0]; return p[2], p[1] + th[2], axis_angle(R)
    if model == "M0":
        p, R = tcp(q); return p[2], p[1] + th[0], axis_angle(R)
NP = {"M0": 2, "M1": 5, "M2": 5, "M3": 3, "M4": 3, "M5": 4}   # 모델 매개변수 + β
def data(zkey, drop=None):
    z = D["height_plate_mm"][zkey]; zs = {k: v for k, v in z.items() if k != "source"}
    ys = {k: v for k, v in D["y_mm"].items() if k != "source"}; an = {k: v for k, v in D["finger_axis_angle_deg_from_vertical_toward_plus_y"].items() if k in Q and v is not None and not isinstance(v, str)}
    if drop is not None: zs.pop(drop, None); ys.pop(drop, None); an.pop(drop, None)
    return zs, ys, an
def resid(model, th, zs, ys, an):
    r = []
    for k, v in zs.items(): r.append((predict(model, th, k)[0] - v) / TOL["z_mm"])
    for k, v in ys.items(): r.append((predict(model, th, k)[1] - v) / TOL["y_diff_mm"])
    for k, v in an.items(): r.append((predict(model, th, k)[2] - v) / TOL["angle_deg_by_pose"][k])
    return np.array(r)
def fit(model, zs, ys, an):
    n = NP[model]; best = None
    for s in range(6):
        BM = TOL["y_common_bias_mm"]   # y 공통 편향 한도 ±4 mm
        B = 10.0   # β ±10°
        if model == "M2": lo, hi = np.array([-5, -5, -5, -BM, -B]), np.array([5, 5, 5, BM, B])        # 관절 영점 ±5°
        elif model == "M1": lo, hi = np.array([-50, -50, -50, -BM, -B]), np.array([50, 50, 50, BM, B])   # 공구 오프셋 ±50 mm(그리퍼 길이 상한)
        elif model in ("M3", "M4"): lo, hi = np.array([-50, -BM, -B]), np.array([50, BM, B])
        elif model == "M5": lo, hi = np.array([0.3, -5, -BM, -B]), np.array([1.3, 5, BM, B])         # J2 이득 0.3~1.3, 영점 ±5°
        else: lo, hi = np.array([-BM, -B]), np.array([BM, B])
        th0 = np.clip(np.random.default_rng(s).normal(0, 3, n) if s else (np.array([1.0, 0, 0, 0]) if model == "M5" else np.zeros(n)), lo, hi); lim = (lo, hi)
        r = least_squares(lambda t: resid(model, t, zs, ys, an), th0, bounds=lim)
        if best is None or r.cost < best.cost: best = r
    return best
BMX = TOL["y_common_bias_mm"]
def report(zkey):
    zs, ys, an = data(zkey); out = {}
    print("\n=== 높이 데이터: %s (z 자세 %s, y 자세 %s) ===" % (zkey, sorted(zs), sorted(ys)))
    for model in ("M0", "M1", "M2", "M3", "M4", "M5"):
        r = fit(model, zs, ys, an); th = r.x; res = resid(model, th, zs, ys, an)
        ar = {k: round(float(predict(model, th, k)[2] - v), 1) for k, v in an.items()}
        zr = {k: round(float(predict(model, th, k)[0] - v), 1) for k, v in zs.items()}; yr = {k: round(float(predict(model, th, k)[1] - v), 1) for k, v in ys.items()}
        J = r.jac; sv = np.linalg.svd(J, compute_uv=False); cond = float(sv[0] / sv[-1]) if sv[-1] > 1e-9 else float("inf")
        # LOO: 자세 하나의 높이·y를 빼고 맞춘 뒤 그 자세 예측
        loo = {}
        for k in ("14", "0", "5"):
            zs2, ys2, an2 = data(zkey, drop=k)
            if len(zs2) + len(ys2) <= NP[model]: loo[k] = None; continue   # 데이터 수 ≤ 매개변수 수: 예측 불가
            r2 = fit(model, zs2, ys2, an2); p = predict(model, r2.x, k); loo[k] = {"z": round(float(p[0] - zs[k]), 1) if k in zs else None, "y": round(float(p[1] - ys[k]), 1)}
        ok_fit = bool(np.all(np.abs(res) <= 1.0)); ok_bias = abs(th[-1] if model in ("M0", "M1", "M2") else th[-1]) <= TOL["y_common_bias_mm"]
        ok_loo = all(v is None or ((v["z"] is None or abs(v["z"]) <= TOL["z_mm"]) and abs(v["y"]) <= TOL["y_diff_mm"] + TOL["y_common_bias_mm"]) for v in loo.values())
        LIM = {"M0": [BMX, 10], "M1": [50, 50, 50, BMX, 10], "M2": [5, 5, 5, BMX, 10], "M3": [50, BMX, 10], "M4": [50, BMX, 10], "M5": [1.3, 5, BMX, 10]}
        ok_bound = not np.any(np.isclose(np.abs(th), np.array(LIM[model]), atol=0.05)) and not (model == "M5" and abs(th[0] - 0.3) < 0.02)
        verdict = "모순" if not ok_fit else ("구분 불가(LOO 불가·조건수·한도 도달)" if (any(v is None for v in loo.values()) or not ok_loo or cond > 30 or not ok_bound) else "모순 없음")
        print("%s 매개변수(마지막 = β) %s | 높이 잔차 %s | y 잔차 %s | 각도 잔차 %s | 조건수 %.1f | LOO %s | 판정 %s" % (model, np.round(th, 2).tolist(), zr, yr, ar, cond, loo, verdict))
        out[model] = {"params": np.round(th, 3).tolist(), "z_resid_mm": zr, "y_resid_mm": yr, "angle_resid_deg": ar, "cond": round(cond, 1), "loo": loo, "ok_fit": ok_fit, "verdict": verdict}
    return out
result = {"data": "evidence/G02/fk_offset_data.json", "tolerance": TOL}
result["fit_video_h25.5"] = report("video_s410_h_strip_25.5")
# 민감도: 띠 높이 +2 mm(사용자 \"종이 위 2밀리\" 해석 불확실)
Dz = D["height_plate_mm"]["video_s410_h_strip_25.5"]; D["height_plate_mm"]["video_plus2"] = {k: (v + 2.0 if k != "source" else v) for k, v in Dz.items()}
result["fit_video_plus2"] = report("video_plus2")
D["height_plate_mm"]["eye_all"] = D["height_plate_mm"]["eye_s48_s49"]; result["fit_eye_all"] = report("eye_all")
# 손가락 축 각도: 영점·공통 오프셋은 자세 간 차이에 영향 없음 → 14→5 변화량 비교(모델 무관)
Dang = D["finger_axis_angle_deg_from_vertical_toward_plus_y"]
fk14 = axis_angle(tcp(Q["14"])[1]); fk5 = axis_angle(tcp(Q["5"])[1]); fk0 = axis_angle(tcp(Q["0"])[1])
meas_delta = Dang["5"] - Dang["14"]; fk_delta = fk5 - fk14
print("\n=== 손가락 축 각도 ===\nFK 접근축 각(14/0/5) = %.1f / %.1f / %.1f°, 측정(14/5) = %.1f / %.1f°" % (fk14, fk0, fk5, Dang["14"], Dang["5"]))
print("14→5 변화량: 측정 %.1f° vs FK %.1f° → 차이 %.1f° (불확도 √(2.5²+6.0²) = %.1f°)" % (meas_delta, fk_delta, meas_delta - fk_delta, (TOL["angle_deg_by_pose"]["14"]**2 + TOL["angle_deg_by_pose"]["5"]**2) ** 0.5))
print("각 자세 차이(측정 − FK): 14 %.1f°, 5 %.1f° (고정 각 오프셋이면 두 값이 같아야 함)" % (Dang["14"] - fk14, Dang["5"] - fk5))
result["finger_angle"] = {"fk_deg": {"14": round(fk14, 1), "0": round(fk0, 1), "5": round(fk5, 1)}, "measured_deg": {"14": Dang["14"], "5": Dang["5"]}, "delta_measured": round(meas_delta, 1), "delta_fk": round(fk_delta, 1), "delta_diff": round(meas_delta - fk_delta, 1), "within_tol": abs(meas_delta - fk_delta) <= (TOL["angle_deg_by_pose"]["14"]**2 + TOL["angle_deg_by_pose"]["5"]**2) ** 0.5}
json.dump(result, open("evidence/G02/fk_offset_analysis.json", "w"), ensure_ascii=False, indent=1)
