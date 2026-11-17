#!/usr/bin/env python3
"""P-A 시험 평가 v2(2회차 구조, 플랜 §6 개정 2, 새 데이터에만 적용): 사전 등록 지표를 측정값에 적용한다. 문턱은 코드 상수이며 결과를 보고 바꾸지 않는다. 1회차(v1 구조) 판정은 스냅샷 261117-pa2의 pa_eval.py로 재현한다.
사용: python3 scripts/pa_eval.py <세션접두사>   (파일: data/g04/pa/<접두사>_*)
입력 파일(세션 오케스트레이터 scripts/pa_test_session.py가 만든다): <S>_snapNN.{mac_timing.txt,meta.json,detect.json,req_id.txt,color.png,depth_mm.npy}(NN 01~10 = 기동 #1, 11~20 = 기동 #2), 36 s 기준 4장 <S>_base_{a1,a2,b1,b2}.{detect.json,color.png,depth_mm.npy,probe.json(참고)}, <S>_probe_start/end.json(기동 #1), <S>_cycles.json."""
import json, math, sys
from pathlib import Path

# ---- 사전 등록 문턱(승인 PA-2, 2026-11-17) ----
M1_MAX_S = 3.0                        # 요청→검출 결과, 20회 모두
FRESH_RECV_MINUS_STAMP = (0.0, 0.2)   # 0 < 수신−stamp < 0.2 s 가 아니면 M1' 판정 불가(stamp 가정 위반)
BIAS_CENTER_PX, BIAS_SIDE_PX, BIAS_YAW_DEG = 0.32, 0.25, 1.0       # M2b 편향(잡음 수준)
GATE_CENTER_PX, GATE_SIDE_PX, GATE_YAW_DEG = 0.75, 0.75, 3.0       # 판정 불가(관문 수준, 기준 앞·뒤 차)
E1_SIDE_PX = 0.7                      # M2c(개정 3): 같은 기동 안 18쌍 변 길이 차 최대 < 0.7 (= E1 (ii) 문턱 +0.7, E-2b §12)
M3_BRIGHT, M3_VALID = 2.0, 0.02       # 같은 세션 기준 2장 범위 ± 
M4_MIN_HZ = 29.0
N_SNAPS = 20
START_OF = lambda i: 1 if i <= 10 else 2       # snap 번호 → 기동(개정 2: 기동 2회 × snap 10장)
BASES = ('a1', 'a2', 'b1', 'b2')                 # 36 s 기준 앞 2장·뒤 2장(독립 기동 4회)
N_STARTS_LOGS = 3                                # USB 판정 대상 로그 수: 기동 #1·#2·자동 종료 시험
# 사전 등록(advisor 반영, 측정 전): 기동마다 확인용 snap 1회(<S>_warmcheck1·2)는 **평가에서 제외**한다. 시험 장치 점검(평가 지표 아님, 사전 등록): 기동 #1·#2·자동 종료 시험 start 직후 status '드라이버 프로세스 있음' 확인, 아니면 즉시 중단. 이 snap이나 기준 사진 A가 실패하면 세션을 즉시 중단한다(검증 안 된 부분을 30분 세션에서 처음 돌리는 위험 차단).


def ang_diff(a, b): return (a - b + 180.0) % 360.0 - 180.0


def mean_angle_near(vals, ref):
    """ref 근처로 접어 평균(±180 경계 처리)."""
    return ref + sum(ang_diff(v, ref) for v in vals) / len(vals)


def dist(a, b): return math.hypot(a[0] - b[0], a[1] - b[1])


def load(p):
    p = Path(p)
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else None


def marks(p):
    d = {}
    p = Path(p)
    if p.exists():
        for line in p.read_text(errors="replace").splitlines():
            t = line.split()
            if len(t) == 2:
                try: d[t[0]] = float(t[1])
                except ValueError: pass
    return d


def m1(snaps):
    lat = []
    for s in snaps:
        mk = s["marks"]
        lat.append(mk["m_detect_done"] - mk["m_start"] if "m_detect_done" in mk and "m_start" in mk else None)
    ok = len(lat) == N_SNAPS and all(x is not None and x <= M1_MAX_S for x in lat)
    return {"pass": ok, "n": len(lat), "latencies_s": lat, "max_s": max([x for x in lat if x is not None], default=None)}


def m1p(snaps):
    rows, undecidable = [], False
    for s in snaps:
        m = s["meta"] or {}
        for k in ("color", "depth"):
            f = m.get(k)
            if not f: rows.append((s["name"], k, None, None)); continue
            rows.append((s["name"], k, f["stamp_minus_req"], f["recv_minus_stamp"]))
    for _, _, smr, rms in rows:
        if rms is None or not (FRESH_RECV_MINUS_STAMP[0] < rms < FRESH_RECV_MINUS_STAMP[1]): undecidable = True
    fresh = all(smr is not None and smr > 0 for _, _, smr, _ in rows) and len(rows) == 2 * N_SNAPS
    return {"pass": (None if undecidable else fresh), "undecidable": undecidable, "fresh_all": fresh, "n_rows": len(rows),
            "min_stamp_minus_req": min([smr for _, _, smr, _ in rows if smr is not None], default=None),
            "recv_minus_stamp_range": [min([r for *_, r in rows if r is not None], default=None), max([r for *_, r in rows if r is not None], default=None)]}


def m1pp(snaps, cycles):
    ids_ok = all((s["meta"] or {}).get("request_id") == s["req_id"] and s["req_id"] for s in snaps) and len(snaps) == N_SNAPS
    c = cycles or {}
    after_stop = c.get("snap_after_stop_failed"); auto_exit = c.get("auto_exit_snap_failed")
    return {"pass": bool(ids_ok and after_stop and auto_exit), "ids_match": ids_ok, "snap_after_stop_failed": after_stop, "auto_exit_snap_failed": auto_exit}


def m2(snaps, bases):
    """bases = 기준 4장의 detect 목록(a1,a2,b1,b2 순). M2b 편향은 P-A 20장 평균 대 기준 4장 평균, 판정 불가는 abs(mean(A1,A2) − mean(B1,B2)) > 관문 수준."""
    det = [s["detect"] for s in snaps]
    found = sum(1 for d in det if d and d.get("found"))
    out = {"m2a": {"pass": found == N_SNAPS and len(det) == N_SNAPS, "found": found, "n": len(det)}}
    pts = [d for d in det if d and d.get("found")]
    if len(bases) != 4 or not all(b and b.get("found") for b in bases) or found < 2:
        out["m2b"] = {"pass": None, "undecidable": True, "reason": "기준 4장 검출 실패 또는 P-A 검출 부족"}
    else:
        def mean_c(bs): return [sum(b["center"][i] for b in bs) / len(bs) for i in (0, 1)]
        def mean_s(bs): return sum(b["side_px"] for b in bs) / len(bs)
        ref_y = bases[0]["yaw_deg"]
        def mean_y(bs): return mean_angle_near([b["yaw_deg"] for b in bs], ref_y)
        A, B = bases[:2], bases[2:]
        gate = (dist(mean_c(A), mean_c(B)), abs(mean_s(A) - mean_s(B)), abs(ang_diff(mean_y(A), mean_y(B))))
        moved = gate[0] > GATE_CENTER_PX or gate[1] > GATE_SIDE_PX or gate[2] > GATE_YAW_DEG   # G2 원문: 0.75 px 이하이면 정상 → 넘으면 판정 불가
        pair_in = {"A1-A2": (dist(A[0]["center"], A[1]["center"]), abs(A[0]["side_px"] - A[1]["side_px"]), abs(ang_diff(A[0]["yaw_deg"], A[1]["yaw_deg"]))),
                   "B1-B2": (dist(B[0]["center"], B[1]["center"]), abs(B[0]["side_px"] - B[1]["side_px"]), abs(ang_diff(B[0]["yaw_deg"], B[1]["yaw_deg"])))}   # 참고: 연달아 찍은 36 s 방식의 기동 간 잡음(장면 이동 판정에 쓰지 않는다)
        pa_c, pa_s, pa_y = mean_c(pts), mean_s(pts), mean_angle_near([p["yaw_deg"] for p in pts], ref_y)
        b_c, b_s, b_y = mean_c(bases), mean_s(bases), mean_y(bases)
        bias = (dist(pa_c, b_c), abs(pa_s - b_s), abs(ang_diff(pa_y, b_y)))
        ok = bias[0] <= BIAS_CENTER_PX and bias[1] <= BIAS_SIDE_PX and bias[2] <= BIAS_YAW_DEG
        out["m2b"] = {"pass": (None if moved else ok), "undecidable": moved, "front_back_mean_diff(center,side,yaw)": gate, "within_pair_diff_reference": pair_in, "bias(center,side,yaw)": bias, "ok_if_decidable": ok}
    pairs = []
    for k, (a, b) in enumerate(zip(snaps, snaps[1:]), 1):
        if START_OF(k) != START_OF(k + 1): continue                # 기동이 바뀌는 쌍 제외(E1은 한 기동 안에서 A·B 비교)
        if a["detect"] and b["detect"] and a["detect"].get("found") and b["detect"].get("found"): pairs.append((a["detect"], b["detect"]))
    sd = [abs(a["side_px"] - b["side_px"]) for a, b in pairs]
    out["m2c"] = {"pass": (len(pairs) == N_SNAPS - 2 and max(sd) < E1_SIDE_PX), "n_pairs": len(pairs), "max_side_diff_px": max(sd, default=None),
                  "max_center_diff_px": max([dist(a["center"], b["center"]) for a, b in pairs], default=None),
                  "ref_e0g_36s_repeat_side_diff_px": "0~0.27 (docs/gates/G07-E-2-파지-들어올림-판정-설계-초안.md:97)"}
    return out


def m3(snaps, bases):
    """단위 = 기동. P-A 기동 #1·#2의 color.png 평균 밝기가 각각 기준 4기동 [최소, 최대] ± 2.0 안, 깊이 유효 비율은 P-A 모든 프레임이 기준 4장 [최소, 최대] ± 0.02 안."""
    if len(bases) != 4 or any(b["bright"] is None or b["valid"] is None for b in bases) or any(s["bright"] is None or s["valid"] is None for s in snaps):
        return {"pass": None, "reason": "측정값 부족"}
    bb = [b["bright"] for b in bases]; bv = [b["valid"] for b in bases]
    lo_b, hi_b = min(bb) - M3_BRIGHT, max(bb) + M3_BRIGHT; lo_v, hi_v = min(bv) - M3_VALID, max(bv) + M3_VALID
    unit = {}
    for st in (1, 2):
        v = [s["bright"] for i, s in enumerate(snaps, 1) if START_OF(i) == st]; unit[st] = sum(v) / len(v)
    okb = all(lo_b <= unit[st] <= hi_b for st in (1, 2)); okv = all(lo_v <= s["valid"] <= hi_v for s in snaps)
    return {"pass": bool(okb and okv), "bright_ok": okb, "valid_ok": okv, "bright_range": [lo_b, hi_b], "valid_range": [lo_v, hi_v], "pa_unit_mean_bright": unit,
            "ref_36s_start_to_start_spread": max(bb) - min(bb), "ref_pa_start1_start2_diff": abs(unit[1] - unit[2]), "base_bright": bb}


def m4(p_start, p_end, cycles):
    def hz(p):
        try: return min(p["rate"]["color"]["arrival_hz"], p["rate"]["depth"]["arrival_hz"])
        except Exception: return None
    hs, he = hz(p_start or {}), hz(p_end or {}); c = cycles or {}
    counts = c.get("usb_counts") or {}
    usb_ok = len(counts) == N_STARTS_LOGS and all(v == 0 for v in counts.values())       # 이번 세션 시작 epoch 이후 로그 3개 모두, 원격에서 파일별로 센 값
    ok = hs is not None and he is not None and hs >= M4_MIN_HZ and he >= M4_MIN_HZ and usb_ok and c.get("containers_left_after_stop") == [0, 0, 0]
    return {"pass": bool(ok), "hz_start": hs, "hz_end": he, "usb_counts": counts, "usb_logs_expected": N_STARTS_LOGS, "containers_left_after_stop": c.get("containers_left_after_stop")}


def _img_stats(d, name):
    try:
        import cv2, numpy as np
        im = cv2.imread(str(Path(d) / (name + ".color.png"))); dp = np.load(Path(d) / (name + ".depth_mm.npy"))
        return (float(im.mean()) if im is not None else None), float((dp > 0).sum() / dp.size)
    except Exception: return None, None


def collect(S, d):
    d = Path(d); snaps = []
    for i in range(1, N_SNAPS + 1):
        n = "%s_snap%02d" % (S, i)
        rid = (d / (n + ".req_id.txt")).read_text().strip() if (d / (n + ".req_id.txt")).exists() else ""
        b, v = _img_stats(d, n)
        snaps.append({"name": n, "marks": marks(d / (n + ".mac_timing.txt")), "meta": load(d / (n + ".meta.json")), "detect": load(d / (n + ".detect.json")), "req_id": rid, "bright": b, "valid": v})
    bases = []
    for t in BASES:
        n = "%s_base_%s" % (S, t); b, v = _img_stats(d, n)
        pr = load(d / (n + ".probe.json")); pm = None
        try: br = pr["color_mean_brightness_per_frame"]; pm = sum(br) / len(br)
        except Exception: pass
        bases.append({"detect": load(d / (n + ".detect.json")), "bright": b, "valid": v, "probe_mean_bright_reference": pm})
    return snaps, bases


def evaluate(S, d):
    snaps, bases = collect(S, d); cycles = load(Path(d) / ("%s_cycles.json" % S))
    r = {"M1": m1(snaps), "M1'": m1p(snaps), "M1''": m1pp(snaps, cycles)}
    r.update(m2(snaps, [b["detect"] for b in bases]))
    r["M3"] = m3(snaps, bases); r["M4"] = m4(load(Path(d) / ("%s_probe_start.json" % S)), load(Path(d) / ("%s_probe_end.json" % S)), cycles)
    keys = ["M1", "M1'", "M1''", "m2a", "m2b", "m2c", "M3", "M4"]
    verdict = [r[k]["pass"] for k in keys]
    r["overall"] = ("불성립" if any(v is False for v in verdict) else ("판정 불가 포함" if any(v is None for v in verdict) else "성립"))
    c = cycles or {}
    # 사전 등록(측정 전): 시험 장치 점검·확인용 snap·기준 사진 A·snap 3회 연속 실패로 중단된 세션, 가짜 환경(FAST) 세션은 P-A 판정이 아니다(재실행 대상). 아래 지표는 참고용 계산이다.
    if c.get("aborted"): r["overall"] = "시험 중단(판정 없음): %s" % c["aborted"]
    elif c.get("fast_mode"): r["overall"] = "시험용 FAST 세션(판정 없음)"
    return r


if __name__ == "__main__":
    if len(sys.argv) != 2: sys.exit("사용: python3 scripts/pa_eval.py <세션접두사>")
    D = Path(__file__).resolve().parents[1] / "data" / "g04" / "pa"
    res = evaluate(sys.argv[1], D)
    (D / ("%s_eval.json" % sys.argv[1])).write_text(json.dumps(res, indent=2, ensure_ascii=False, default=str) + "\n", encoding="utf-8")
    for k in ["M1", "M1'", "M1''", "m2a", "m2b", "m2c", "M3", "M4"]:
        print("%-5s %s" % (k, {True: "PASS", False: "FAIL", None: "판정 불가"}[res[k]["pass"]]), json.dumps({a: b for a, b in res[k].items() if a != "pass" and a != "latencies_s"}, ensure_ascii=False, default=str)[:300])
    if res["overall"].startswith(("시험 중단", "시험용")): print("★ P-A 판정 없음 — 아래 지표는 참고용 계산이다(재실행 대상)")
    print("전체:", res["overall"])
