#!/usr/bin/env python3
"""세트 33 실행 결과의 E1·E2·E3·Q3 사후 계산(R8, Mac, 로봇·카메라 없음). **사전 등록 산출물**: 첫 실기기 실행 전에 코드를 고정하고, 첫 실행 뒤 이 파일을 바꾸면 그 실행의 재채점은 '참고용, 결정 근거 아님'이다.
규칙은 새 문턱 없이 이미 등록·승인된 함수만 쓴다: E1 `e1_decide`(위 ≥ 8 px, 변 길이 ≥ +0.7 px, 옆 ≤ 4 px), E3 `e3_verdict`(P5), Q3 `q3_record`/`q3_verdict`(영역 (800,0,900,100), ZNCC 최솟값 < 0.90 → 판정 불가),
E2는 '정체 명령 150 + 회독 ≤ 147'(R3 진행 조건과 같다 — 독립 증거가 아니다; 회독 − 146은 기록만, 곡선 허용 폭은 첫 기록 이후 별도 결정). **성공 = E2·E1·E3·Q3가 모두 합격**(E-2b `:14` 승인 규칙, 판정 불가·검출 실패는 실패).
- **E1'(마커 가려짐 대체 지표)는 구현하지 않는다**(사용자 결정 2026-11-18): 가려짐은 '판정 불가 = 실패'이고 그 시도의 사진 B·C와 E4(사람 입력·영상)는 기록 전용이다. E-2a를 하지 않았고, 첫 시도가 쥔 상태 마커 가림의 첫 관측이며, E1-R 경로(E1'와 E2·E4 합의)는 구현하지 않는다.
- 입력은 **러너가 받아들인 것**(`run2.lift.photos`·`photo_times`·`clock_log`)이다. 사진 다리 로그는 보조 기록(없어도 계산, 있으면 request_id·started_epoch 대조, 어긋나면 그 사진 '판정 없음').
- 사진은 **내용 해시**로 묶는다: 다리가 결과에 넣은 `image_sha256`이 이미지 파일의 sha256과 같아야 하고(없거나 다르면 사진 기반 기준 '판정 없음'), `pa_detect` 재실행 검출이 러너의 `detect`와 같은지는 보조 일치 점검이다.
- E3 사진 C: 유지 생략이면 판정 불가(E-2 원래 규칙) → C 미요청이고 `clock_log`가 photo_c를 거부했으면 P5(J6 회독판만) → C 요청했으나 결과 없음은 판정 불가(자체 결정 2026-11-18, 더 보수적) → C 있는데 기동 불일치·해시 불일치는 판정 불가(R5).
- 집계: 시도 수(분류기)와 판정 기록 수(네 기준이 모두 합격 또는 불합격)를 분리하고, 그중 'E1 신호 약함 가능' 라벨 기록 수를 따로 낸다(병목 해소로 셀지는 사용자 몫).
사용: .venv/bin/python -I scripts/g01_lift_r8.py <결과 JSON> --ts <TS> [--root <프로젝트 루트>] [--log <사진 다리 로그>]"""
import argparse, hashlib, json, os, subprocess, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g01_lift_logic as L, g01_lift_classify as K

NA = "판정 없음"; NJ = "판정 불가"


def _sha(path):
    with open(path, "rb") as f: return hashlib.sha256(f.read()).hexdigest()


def _run2_lift(result):
    for p in result.get("poses", []) or []:
        r = p.get("run2") or {}
        if "close_steps" in r or "lift" in r: return r, r.get("lift") or {}
    return {}, {}


def _bridge_results(log_path):
    """사진 다리 로그의 마지막 results 줄(없으면 None)."""
    if not log_path or not os.path.exists(log_path): return None
    for line in reversed(open(log_path, encoding="utf-8").read().splitlines()):
        line = line.strip()
        if line.startswith("{") and '"results"' in line:
            try: return json.loads(line)["results"]
            except Exception: return None
    return None


def load_photos(lift, root, ts, ref, bridge, detect_fn=None, P=None):
    """사진별 사용 가능 여부와 이유. 반환 {이름: {"ok": bool, "why": str, "detect":..., "path":...}} (ref 포함)."""
    pa = os.path.join(root, "data", "g04", "pa"); out = {}
    def check(name, expected_sha, detect_runner, ids):
        path = os.path.join(pa, "lift_%s_%s.color.png" % (ts, name)); d = {"ok": False, "path": path, "detect": detect_runner, "why": ""}
        if not os.path.exists(path): d["why"] = "이미지 파일 없음"; return d
        got = _sha(path); d["sha256"] = got
        if not expected_sha: d["why"] = "결과에 image_sha256 없음(내용 연결 불가)"; return d
        if got != expected_sha: d["why"] = "이미지 해시 불일치(파일이 바뀌었거나 다른 사진)"; return d
        if bridge is not None and name in bridge:                      # 보조 기록과의 대조: 어긋나면 판정 없음(보수적)
            b = bridge[name] or {}
            if b.get("request_id") != ids.get("request_id") or b.get("started_epoch") != ids.get("started_epoch"): d["why"] = "사진 다리 로그의 request_id·started_epoch가 러너 기록과 다르다"; return d
        d["ok"] = True; return d
    ref_ok = check("ref", (ref or {}).get("image_sha256"), (ref or {}).get("detect"), {"request_id": None, "started_epoch": None}) if ref else {"ok": False, "why": "기준 JSON 없음", "path": None, "detect": None}
    out["ref"] = ref_ok
    for name in ("A", "B", "C"):
        r = (lift.get("photos") or {}).get(name); pt = (lift.get("photo_times") or {}).get(name)
        if r is None: out[name] = {"ok": False, "why": ("요청했으나 결과 없음(시간 초과·토큰 불일치)" if pt else "요청하지 않음"), "requested": bool(pt), "detect": None}; continue
        d = check(name, r.get("image_sha256"), r.get("detect"), {"request_id": r.get("request_id"), "started_epoch": r.get("started_epoch")}); d["requested"] = True; d["started_epoch"] = r.get("started_epoch")
        if d["ok"] and detect_fn is not None:                          # 보조 일치 점검: 재검출이 러너의 detect와 같아야 한다
            try:
                import cv2; again = detect_fn(cv2.imread(d["path"]))
                d["redetect_equal"] = (again == r.get("detect"))
                if not d["redetect_equal"]: d["ok"] = False; d["why"] = "pa_detect 재검출이 러너의 detect와 다르다"
            except Exception as e: d["redetect_equal"] = None; d["note"] = "재검출 실패: %r" % e
        out[name] = d
    return out


def _post_return_record(root, ts, ref):
    """복귀 뒤 P-A 스냅(`lift_<TS>_post`, 기록용 P4)의 블록 마커 yaw·위치를 기준 스냅과 비교해 기록한다(판정 아님). 낙하 뒤 복귀 경로의 블록 yaw 자료: 사용자 결정(2026-11-18)으로 첫 실행은 사람 확인만 쓰고 이 기록을 본 뒤 복귀 경로를 정한다."""
    p = os.path.join(root, "data", "g04", "pa", "lift_%s_post.detect.json" % ts)
    if not os.path.exists(p): return {"available": False, "why": "복귀 뒤 스냅 detect.json 없음"}
    try: d = json.load(open(p, encoding="utf-8"))
    except Exception as e: return {"available": False, "why": "읽기 실패 %r" % e}
    out = {"available": True, "found": bool(d.get("found")), "detect": d}
    rd = (ref or {}).get("detect") or {}
    if d.get("found") and rd.get("found"):
        out["yaw_delta_deg"] = round((d["yaw_deg"] - rd["yaw_deg"] + 180.0) % 360.0 - 180.0, 2); out["center_shift_px"] = [round(d["center"][0] - rd["center"][0], 2), round(d["center"][1] - rd["center"][1], 2)]
    out["note"] = "기록 전용(판정 아님). 마커가 가려지거나 복귀가 안 끝나면 없음. 낙하 뒤 복귀 경로 간격은 블록 yaw 약 5° 이내에서만 2 mm(오프라인 UNVERIFIED)"
    return out


def judge(result, root, ts, ref, bridge=None, detect_fn=None, P=None):
    P = L.PARAMS if P is None else P
    cls = K.classify(result); r2, lift = _run2_lift(result); notes = []
    photos = load_photos(lift, root, ts, ref, bridge, detect_fn, P)
    moves = lift.get("moves") or []; lifted = any(m.get("step") == "lift3" and not m.get("aborted") for m in moves) and not lift.get("code")
    partial = bool(moves) and not lifted
    state = "들어올림 완료" if lifted else ("들어올림 중단(첫 이동 뒤 열림)" if partial else "들어올림 없음")
    none_why = {"들어올림 없음": "판정 불가(들어올림 없음)", "들어올림 중단(첫 이동 뒤 열림)": "판정 불가(들어올림 중단)"}.get(state)
    epochs = lambda names: [ (ref or {}).get("started_epoch") ] + [photos[n].get("started_epoch") for n in names]
    crit = {}
    # --- E2 ---
    sc, sr = r2.get("stall_command"), r2.get("stall_readback")
    if sc is None or sr is None: crit["E2"] = {"verdict": NJ, "reason": "정체 기록 없음"}
    else: crit["E2"] = {"verdict": "합격" if (sc == 150 and sr <= 147) else "불합격", "stall_command": sc, "stall_readback": sr, "minus_146": sr - 146, "note": "R3 진행 조건(명령 150, 회독 ≤ 147)과 동일하며 독립 증거가 아니다. 146과의 차이는 기록만, 곡선 허용 폭은 첫 기록 이후 별도 결정"}
    # --- E1 ---
    if none_why: crit["E1"] = {"verdict": none_why}
    elif not (photos["A"]["ok"] and photos["B"]["ok"] and photos["ref"]["ok"]): crit["E1"] = {"verdict": NA, "reason": "; ".join("%s: %s" % (n, photos[n]["why"]) for n in ("ref", "A", "B") if not photos[n]["ok"])}
    elif not L.same_start(epochs(("A", "B"))): crit["E1"] = {"verdict": NA, "reason": "기준·A·B가 같은 P-A 기동이 아니다(R5)"}
    else:
        e1 = L.e1_decide(photos["A"]["detect"], photos["B"]["detect"], P)
        if e1["verdict"] == NJ: e1["note"] = "마커 가려짐 등 미검출: E1'는 구현하지 않는다(판정 불가 = 실패). 이 시도의 사진 B·C와 E4는 기록 전용"
        crit["E1"] = L.e1_attempt_label(e1, lift)
    # --- E3 (확인 순서: 유지 → 사진 C 경우) ---
    j6c, j6l = lift.get("j6_after_close"), lift.get("j6_after_lift")
    if none_why: crit["E3"] = {"verdict": none_why}
    elif not lift.get("hold_done"): crit["E3"] = {"verdict": NJ, "counts_as": "실패", "rule": "E-2 원래 규칙(유지 생략·미실행)"}
    elif (lift.get("photo_times") or {}).get("C") is None:
        refused = any(e[0] == "photo_c" and not e[3] for e in (lift.get("clock_log") or []))
        if refused: crit["E3"] = dict(L.e3_verdict(j6c, j6l, True, False, None, P), case="① C 요청 안 함(시계 거부): P5 J6 회독판만")
        else: crit["E3"] = {"verdict": NJ, "counts_as": "실패", "rule": "C 미요청인데 시계 거부 기록이 없다"}
    elif photos["C"].get("detect") is None and not photos["C"].get("ok"):
        crit["E3"] = {"verdict": NJ, "counts_as": "실패", "case": "② C 요청했으나 결과 없음 또는 사용 불가(자체 결정 2026-11-18, P5 범위 밖)", "reason": photos["C"]["why"]}
    elif not (photos["B"]["ok"] and photos["C"]["ok"] and L.same_start(epochs(("B", "C")))): crit["E3"] = {"verdict": NJ, "counts_as": "실패", "case": "③ C 기동·해시 불일치(R5)", "reason": "B: %s / C: %s" % (photos["B"]["why"] or "ok", photos["C"]["why"] or "ok")}
    else: crit["E3"] = dict(L.e3_verdict(j6c, j6l, True, True, (photos["B"]["detect"], photos["C"]["detect"]), P), case="유지 확인 사진 C 포함")
    # --- Q3 ---
    if none_why: crit["Q3"] = {"verdict": none_why}
    elif not (photos["A"]["ok"] and photos["B"]["ok"] and photos["ref"]["ok"]): crit["Q3"] = {"verdict": NA, "reason": "; ".join("%s: %s" % (n, photos[n]["why"]) for n in ("ref", "A", "B") if not photos[n]["ok"])}
    elif not L.same_start(epochs(("A", "B"))): crit["Q3"] = {"verdict": NA, "reason": "기준·A·B가 같은 P-A 기동이 아니다(R5)"}
    else:
        import cv2
        k1, a, b = (cv2.imread(photos[n]["path"]) for n in ("ref", "A", "B"))
        rec = L.q3_record(a, b, P, k1_bgr=k1); v = L.q3_verdict(rec, P); crit["Q3"] = dict(v, record=rec)
    verdicts = {k: v["verdict"] for k, v in crit.items()}
    judged = all(x in ("합격", "불합격") for x in verdicts.values())
    success = all(x == "합격" for x in verdicts.values())
    weak = bool((crit["E1"] or {}).get("signal_shortfall_possible"))
    try: commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root, capture_output=True, text=True, timeout=10).stdout.strip() or "unknown"
    except Exception: commit = "unknown"
    notes += ["E1'(가려짐 대체 지표)·E1-R 경로는 구현하지 않는다. E-2a 미실시. 첫 시도가 쥔 상태 마커 가림의 첫 관측일 수 있다.", "E4(사람 입력·영상)는 기록 전용.",
              "사진 다리 로그 " + ("대조함" if bridge is not None else "없음: 러너 기록과 이미지 해시만으로 계산")]
    post = _post_return_record(root, ts, ref)
    return {"ts": ts, "r8_code_commit": commit, "post_return_record": post, "classification": cls, "state": state, "criteria": crit, "verdicts": verdicts, "success": success,
            "judged_record": judged, "weak_signal_label": weak, "photos": {n: {k: v for k, v in d.items() if k != "detect"} for n, d in photos.items()},
            "params": L.params_jsonable(P), "notes": notes}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("result"); ap.add_argument("--ts", required=True); ap.add_argument("--root", default=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))); ap.add_argument("--log", default=None); ap.add_argument("--out", default=None); a = ap.parse_args()
    result = json.load(open(a.result, encoding="utf-8"))
    refp = os.path.join(a.root, "data", "g04", "pa", "lift_%s_ref.lift_photo_ref.json" % a.ts); ref = json.load(open(refp, encoding="utf-8")) if os.path.exists(refp) else None
    import pa_detect
    out = judge(result, a.root, a.ts, ref, _bridge_results(a.log), detect_fn=pa_detect.detect)
    path = a.out or os.path.join(a.root, "evidence", "G01", "raw", "lift_r8_%s.json" % a.ts)
    with open(path, "w", encoding="utf-8") as f: json.dump(out, f, ensure_ascii=False, indent=2, default=lambda o: float(o))
    print("R8 판정(사전 등록 규칙, 코드 커밋 %s)" % out["r8_code_commit"][:7])
    for k in ("E2", "E1", "E3", "Q3"): print("  %s: %s%s" % (k, out["verdicts"][k], (" — " + (out["criteria"][k].get("note") or out["criteria"][k].get("reason") or out["criteria"][k].get("case") or "")) if out["verdicts"][k] not in ("합격", "불합격") or out["criteria"][k].get("note") else ""))
    print("성공: %s | 판정 기록(네 기준 모두 합격/불합격): %s | E1 신호 약함 가능 라벨: %s" % (out["success"], out["judged_record"], out["weak_signal_label"]))
    pr = out["post_return_record"]; print("복귀 뒤 블록(기록 전용): " + (("yaw 변화 %s°, 중심 이동 %s px" % (pr.get("yaw_delta_deg"), pr.get("center_shift_px"))) if pr.get("available") and pr.get("found") else "자료 없음 또는 마커 미검출"))
    print("저장:", path)


if __name__ == "__main__": main()
