#!/usr/bin/env python3
"""P-A 시험 평가: 사전 등록 지표(플랜 docs/261117-2335-P-A-상시-드라이버-단일프레임-개발-플랜.md §3)를 측정값에 적용한다. 문턱은 코드 상수이며 결과를 보고 바꾸지 않는다.
사용: python3 scripts/pa_eval.py <세션접두사>   (파일: data/g04/pa/<접두사>_*)
입력 파일(세션 오케스트레이터 scripts/pa_test_session.py가 만든다): <S>_snapNN.{mac_timing.txt,meta.json,detect.json,req_id.txt,color.png,depth_mm.npy}, <S>_base_a/b.{detect.json,color.png,depth_mm.npy}, <S>_probe_start/end.json, <S>_cycles.json."""
import json, math, sys
from pathlib import Path

# ---- 사전 등록 문턱(승인 PA-2, 2026-11-17) ----
M1_MAX_S = 3.0                        # 요청→검출 결과, 20회 모두
FRESH_RECV_MINUS_STAMP = (0.0, 0.2)   # 0 < 수신−stamp < 0.2 s 가 아니면 M1' 판정 불가(stamp 가정 위반)
BIAS_CENTER_PX, BIAS_SIDE_PX, BIAS_YAW_DEG = 0.32, 0.25, 1.0       # M2b 편향(잡음 수준)
GATE_CENTER_PX, GATE_SIDE_PX, GATE_YAW_DEG = 0.75, 0.75, 3.0       # 판정 불가(관문 수준, 기준 앞·뒤 차)
E1_SIDE_PX = 0.5                      # M2c: 연속 19쌍 변 길이 차 최대 < 0.5 (E1 (ii) 문턱 미접촉)
M3_BRIGHT, M3_VALID = 2.0, 0.02       # 같은 세션 기준 2장 범위 ± 
M4_MIN_HZ = 29.0
N_SNAPS = 20
# 사전 등록(advisor 반영, 측정 전): 시작 직후 확인용 snap 1회(<S>_warmcheck)는 **평가에서 제외**한다. 이 snap이나 기준 사진 A가 실패하면 세션을 즉시 중단한다(검증 안 된 부분을 30분 세션에서 처음 돌리는 위험 차단).


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


def m2(snaps, base_a, base_b):
    det = [s["detect"] for s in snaps]
    found = sum(1 for d in det if d and d.get("found"))
    out = {"m2a": {"pass": found == N_SNAPS and len(det) == N_SNAPS, "found": found, "n": len(det)}}
    if not (base_a and base_b and base_a.get("found") and base_b.get("found")) or found < 2:
        out["m2b"] = {"pass": None, "undecidable": True, "reason": "기준 사진 검출 실패 또는 P-A 검출 부족"}
    else:
        bc, bs, by = base_a["center"], base_a["side_px"], base_a["yaw_deg"]
        gate = (dist(base_a["center"], base_b["center"]), abs(base_a["side_px"] - base_b["side_px"]), abs(ang_diff(base_a["yaw_deg"], base_b["yaw_deg"])))
        moved = gate[0] > GATE_CENTER_PX or gate[1] > GATE_SIDE_PX or gate[2] > GATE_YAW_DEG   # G2 원문: 0.75 px 이하이면 정상 → 넘으면 판정 불가
        pts = [d for d in det if d and d.get("found")]
        pa_c = [sum(p["center"][i] for p in pts) / len(pts) for i in (0, 1)]; pa_s = sum(p["side_px"] for p in pts) / len(pts)
        pa_y = mean_angle_near([p["yaw_deg"] for p in pts], base_a["yaw_deg"])
        b_c = [(base_a["center"][i] + base_b["center"][i]) / 2 for i in (0, 1)]; b_s = (base_a["side_px"] + base_b["side_px"]) / 2
        b_y = mean_angle_near([base_a["yaw_deg"], base_b["yaw_deg"]], base_a["yaw_deg"])
        bias = (dist(pa_c, b_c), abs(pa_s - b_s), abs(ang_diff(pa_y, b_y)))
        ok = bias[0] <= BIAS_CENTER_PX and bias[1] <= BIAS_SIDE_PX and bias[2] <= BIAS_YAW_DEG
        out["m2b"] = {"pass": (None if moved else ok), "undecidable": moved, "baseline_pair_diff(center,side,yaw)": gate, "bias(center,side,yaw)": bias, "ok_if_decidable": ok}
    pairs = [(a["detect"], b["detect"]) for a, b in zip(snaps, snaps[1:]) if a["detect"] and b["detect"] and a["detect"].get("found") and b["detect"].get("found")]
    sd = [abs(a["side_px"] - b["side_px"]) for a, b in pairs]
    out["m2c"] = {"pass": (len(pairs) == N_SNAPS - 1 and max(sd) < E1_SIDE_PX), "n_pairs": len(pairs), "max_side_diff_px": max(sd, default=None),
                  "max_center_diff_px": max([dist(a["center"], b["center"]) for a, b in pairs], default=None),
                  "ref_e0g_36s_repeat_side_diff_px": "0~0.27 (docs/gates/G07-E-2-파지-들어올림-판정-설계-초안.md:97)"}
    return out


def m3(snaps, base_a, base_b):
    def stat(s): return s["bright"], s["valid"]
    if not (base_a and base_b) or any(stat(s)[0] is None for s in snaps) or base_a["bright"] is None or base_b["bright"] is None:
        return {"pass": None, "reason": "측정값 부족"}
    lo_b, hi_b = min(base_a["bright"], base_b["bright"]) - M3_BRIGHT, max(base_a["bright"], base_b["bright"]) + M3_BRIGHT
    lo_v, hi_v = min(base_a["valid"], base_b["valid"]) - M3_VALID, max(base_a["valid"], base_b["valid"]) + M3_VALID
    okb = all(lo_b <= s["bright"] <= hi_b for s in snaps); okv = all(lo_v <= s["valid"] <= hi_v for s in snaps)
    return {"pass": bool(okb and okv), "bright_ok": okb, "valid_ok": okv, "bright_range": [lo_b, hi_b], "valid_range": [lo_v, hi_v],
            "pa_bright": [min(s["bright"] for s in snaps), max(s["bright"] for s in snaps)], "pa_valid": [min(s["valid"] for s in snaps), max(s["valid"] for s in snaps)]}


def m4(p_start, p_end, cycles):
    def hz(p):
        try: return min(p["rate"]["color"]["arrival_hz"], p["rate"]["depth"]["arrival_hz"])
        except Exception: return None
    hs, he = hz(p_start or {}), hz(p_end or {}); c = cycles or {}
    ok = hs is not None and he is not None and hs >= M4_MIN_HZ and he >= M4_MIN_HZ and c.get("usb_disconnect_lines") == 0 and c.get("containers_left_after_stop") == [0, 0, 0]
    return {"pass": bool(ok), "hz_start": hs, "hz_end": he, "usb_disconnect_lines": c.get("usb_disconnect_lines"), "containers_left_after_stop": c.get("containers_left_after_stop")}


def collect(S, d):
    d = Path(d); snaps = []
    for i in range(1, N_SNAPS + 1):
        n = "%s_snap%02d" % (S, i)
        det = load(d / (n + ".detect.json")); meta = load(d / (n + ".meta.json"))
        rid = (d / (n + ".req_id.txt")).read_text().strip() if (d / (n + ".req_id.txt")).exists() else ""
        s = {"name": n, "marks": marks(d / (n + ".mac_timing.txt")), "meta": meta, "detect": det, "req_id": rid, "bright": None, "valid": None}
        try:
            import cv2, numpy as np
            im = cv2.imread(str(d / (n + ".color.png"))); dp = np.load(d / (n + ".depth_mm.npy"))
            if im is not None: s["bright"] = float(im.mean())
            s["valid"] = float((dp > 0).sum() / dp.size)
        except Exception: pass
        snaps.append(s)
    def base(tag):
        det = load(d / ("%s_base_%s.detect.json" % (S, tag))); b = {"detect": det, "bright": None, "valid": None}
        try:
            import cv2, numpy as np
            im = cv2.imread(str(d / ("%s_base_%s.color.png" % (S, tag)))); dp = np.load(d / ("%s_base_%s.depth_mm.npy" % (S, tag)))
            b["bright"] = float(im.mean()); b["valid"] = float((dp > 0).sum() / dp.size)
        except Exception: pass
        return b
    return snaps, base("a"), base("b")


def evaluate(S, d):
    snaps, ba, bb = collect(S, d); cycles = load(Path(d) / ("%s_cycles.json" % S))
    r = {"M1": m1(snaps), "M1'": m1p(snaps), "M1''": m1pp(snaps, cycles)}
    r.update(m2(snaps, ba["detect"], bb["detect"]))
    r["M3"] = m3(snaps, ba, bb); r["M4"] = m4(load(Path(d) / ("%s_probe_start.json" % S)), load(Path(d) / ("%s_probe_end.json" % S)), cycles)
    keys = ["M1", "M1'", "M1''", "m2a", "m2b", "m2c", "M3", "M4"]
    verdict = [r[k]["pass"] for k in keys]
    r["overall"] = ("불성립" if any(v is False for v in verdict) else ("판정 불가 포함" if any(v is None for v in verdict) else "성립"))
    return r


if __name__ == "__main__":
    if len(sys.argv) != 2: sys.exit("사용: python3 scripts/pa_eval.py <세션접두사>")
    D = Path(__file__).resolve().parents[1] / "data" / "g04" / "pa"
    res = evaluate(sys.argv[1], D)
    (D / ("%s_eval.json" % sys.argv[1])).write_text(json.dumps(res, indent=2, ensure_ascii=False, default=str) + "\n", encoding="utf-8")
    for k in ["M1", "M1'", "M1''", "m2a", "m2b", "m2c", "M3", "M4"]:
        print("%-5s %s" % (k, {True: "PASS", False: "FAIL", None: "판정 불가"}[res[k]["pass"]]), json.dumps({a: b for a, b in res[k].items() if a != "pass" and a != "latencies_s"}, ensure_ascii=False, default=str)[:300])
    print("전체:", res["overall"])
