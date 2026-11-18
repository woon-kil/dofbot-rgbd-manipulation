#!/usr/bin/env python3
"""들어올림 사진 경로 dry run 보고(Mac, 읽기 전용). 판독 규칙은 실행 전에 등록되어 있다(플랜 §3-2):
 - **창 충족** = 따뜻한 20회와 차가운 3회(첫 A·130 s 휴지·310 s 휴지) 모두 D ≤ photo_wait_max_s(3.0 s)이고 결과 누락 0회, request_id 불일치 0회, 기동 불일치 0회.
 - 하나라도 어긋나면 **등급 A**(사용자에게 올림). 3.0 s 값 변경이나 연결 예열 같은 사진 다리 변경은 별도 설계 검토 뒤 사용자가 정한다.
 - **사진 A의 실제 상태에 가장 가까운 시험은 310 s 휴지 A**(상태·푸시용 lift-ssh 마스터와 pa-ssh 마스터가 모두 만료)이고, '기준 스냅 직후 첫 A'는 pa-ssh가 아직 따뜻해 일부만 차갑다.
 - 정렬 보고 경로: 정렬 보고 문구(OK 또는 NG)가 15 s 안에 나와야 한다(K1에서는 캡 미검출 NG가 정상).
 - D(Jetson 시계)와 Mac 단계 시각(Mac 시계)은 서로 빼지 않고 각각 따로 보고한다.
사용: .venv/bin/python -I scripts/g01_lift_dryrun_report.py <스텁 결과 JSON> [--log <사진 다리 로그>] [--cap 3.0] [--out <JSON>]"""
import argparse, json, os, statistics, sys

COLD = ("cold_first", "idle_130s", "idle_310s")


def _pct(v, p):
    s = sorted(v); k = (len(s) - 1) * p; f = int(k); c = min(f + 1, len(s) - 1); return s[f] + (s[c] - s[f]) * (k - f)


def report(stub, bridge=None, cap=3.0):
    tr = stub.get("trials", []); warm = [t for t in tr if t["label"].startswith("warm_")]; cold = [t for t in tr if t["label"] in COLD or t["label"].startswith("idle_")]
    def stats(ts):
        d = [t["D_s"] for t in ts if t.get("D_s") is not None]
        return {"n": len(ts), "received": len(d), "min": min(d) if d else None, "median": statistics.median(d) if d else None, "p95": _pct(d, 0.95) if d else None, "max": max(d) if d else None, "over_cap": sum(1 for x in d if x > cap)}
    bad = {"missing": sum(1 for t in tr if not t.get("ok")), "request_id_mismatch": sum(1 for t in tr if t.get("ok") and not t.get("request_id_match")), "start_mismatch": sum(1 for t in tr if t.get("ok") and not t.get("same_start")),
           "no_image_hash": sum(1 for t in tr if t.get("ok") and not t.get("image_sha256"))}
    over = [t["label"] for t in tr if t.get("D_s") is not None and t["D_s"] > cap]
    expected = 1 + 20 + 2
    met = len(tr) == expected and not over and not any(bad.values()) and stub.get("precheck", {}).get("ok", False)
    out = {"window_cap_s": cap, "window_met": met, "grade": "통과(창 충족)" if met else "등급 A: 사용자에게 올림", "trials": len(tr), "expected_trials": expected, "warm": stats(warm), "cold": {t["label"]: t.get("D_s") for t in cold},
           "over_cap_labels": over, "bad": bad, "precheck": stub.get("precheck"), "python": stub.get("python"),
           "note": "D는 Jetson 시계, Mac 단계 시각은 Mac 시계라 서로 빼지 않는다. 사진 A에 가장 가까운 시험은 310 s 휴지 A."}
    if bridge:
        hist = bridge.get("history", []); out["mac_stage_seconds_mac_clock"] = [{"name": h.get("name"), "snap": round(h["mac"]["snap_done"] - h["mac"]["req_seen"], 3), "gate": round(h["mac"]["gate_done"] - h["mac"]["req_seen"], 3), "push": round(h["mac"]["push_done"] - h["mac"]["req_seen"], 3) if "push_done" in h["mac"] else None} for h in hist if h.get("mac")]
        out["align_seconds_mac_clock"] = (bridge.get("results") or {}).get("align_seconds"); out["align_text_present"] = bool((bridge.get("results") or {}).get("align"))
        out["align_ok_within_15s"] = out["align_text_present"] and (out["align_seconds_mac_clock"] or 99) <= 15
        if not out["align_ok_within_15s"]: out["window_met"] = False; out["grade"] = "등급 A: 사용자에게 올림"
    else: out["align_note"] = "사진 다리 로그 없음: 정렬 보고 경로는 판정 못 함"; out["window_met"] = False; out["grade"] = "등급 A: 사용자에게 올림"
    return out


def last_bridge(path):
    if not path or not os.path.exists(path): return None
    for line in reversed(open(path, encoding="utf-8").read().splitlines()):
        if line.startswith("{") and '"results"' in line:
            try: return json.loads(line)
            except Exception: return None
    return None


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("stub"); ap.add_argument("--log", default=None); ap.add_argument("--cap", type=float, default=3.0); ap.add_argument("--out", default=None); a = ap.parse_args()
    out = report(json.load(open(a.stub, encoding="utf-8")), last_bridge(a.log), a.cap)
    if a.out: json.dump(out, open(a.out, "w", encoding="utf-8"), ensure_ascii=False, indent=2)
    w = out["warm"]; print("dry run 보고: %s (시험 %d/%d)" % (out["grade"], out["trials"], out["expected_trials"]))
    print("  따뜻한 20회 D(Jetson 시계): 최소 %s 중앙 %s p95 %s 최대 %s s, %.1f s 초과 %d회, 수신 %d/%d" % tuple([("%.3f" % w[k]) if w[k] is not None else "-" for k in ("min", "median", "p95", "max")] + [out["window_cap_s"], w["over_cap"], w["received"], w["n"]]))
    print("  차가운 시험 D:", {k: (round(v, 3) if v is not None else None) for k, v in out["cold"].items()}, "| 불일치·누락:", out["bad"])
    if "align_seconds_mac_clock" in out: print("  정렬 보고: 문구 %s, %s s(Mac 시계)" % ("있음" if out["align_text_present"] else "없음", out["align_seconds_mac_clock"]))
    print("  Mac 단계 시각(Mac 시계)은 JSON의 mac_stage_seconds_mac_clock 참조. " + out["note"])


if __name__ == "__main__": main()
