#!/usr/bin/env python3
"""들어올림 사진 경로 dry run 스텁(Jetson에서 실행, **로봇 명령 없음**: Arm_Lib을 import하지 않는다). 러너의 접촉 분기가 쓰는 `FilePhotoChannel`로 사진 요청 줄을 출력하고 Mac 사진 다리의 결과 파일을 기다려 D를 잰다.
계획 §3 step 4(사진 요청·확인·전달 경로 N=20)의 변형: 차가운 첫 A 1회 → 따뜻한 20회(A형 5·B형 5·교대 10, 간격 5 s) → 130 s 휴지 뒤 A → 310 s 휴지 뒤 A. 시작 전에 `[정렬 확인]` 줄을 한 번 내 정렬 보고 경로(15 s 하위 프로세스)를 쓰고,
러너가 쓰는 `g01_lift_params`·`g01_lift_logic`·`g01_lift_hook`을 Jetson에서 처음 import하고 `precheck_before_close`(R6)를 실제 기준 JSON으로 호출한다.
D는 **Jetson 시계**(요청 에포크 → 결과 수신 에포크)이고 Mac 단계 시각은 Mac 시계다. 서로 빼지 않는다.
사용(래퍼가 호출): cd ~/dofbot-g01_5b4 && python3 g01_lift_dryrun_stub.py --run-id <TS>"""
import argparse, json, platform, sys, time
sys.path.insert(0, ".")


def say(s): print(s, flush=True)


def schedule(n_warm=20, idle1=130, idle2=310, gap=5):
    """(라벨, 이름, 요청 전 대기 s). 따뜻한 20회 = A 5 → B 5 → 교대 10."""
    types = ["A"] * 5 + ["B"] * 5 + ["A", "B"] * 5
    return [("cold_first", "A", 0)] + [("warm_%02d" % (i + 1), t, gap) for i, t in enumerate(types[:n_warm])] + [("idle_%ds" % idle1, "A", idle1), ("idle_%ds" % idle2, "A", idle2)]


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--run-id", required=True); ap.add_argument("--q", default="86.3,68.32,11.37,31.42"); ap.add_argument("--wait-cap", type=float, default=10.0)
    ap.add_argument("--align-wait", type=float, default=20.0); ap.add_argument("--out", default="dryrun_stub_result.json"); ap.add_argument("--idle1", type=int, default=130); ap.add_argument("--idle2", type=int, default=310); ap.add_argument("--gap", type=float, default=5.0)
    ap.add_argument("--n-warm", type=int, default=20); a = ap.parse_args()
    import g01_lift_params as LP, g01_lift_logic as L, g01_lift_hook as LH           # 러너가 접촉 분기에서 import하는 모듈(Jetson에서 처음 import)
    say("[스텁] python %s, g01_lift_params·g01_lift_logic·g01_lift_hook import 성공 (Arm_Lib 미사용, 로봇 명령 없음)" % platform.python_version())
    ch = LH.FilePhotoChannel(say, folder="."); ok, why, ref = LH.precheck_before_close(ch, LP.PARAMS, time.monotonic(), a.run_id)
    rec = {"python": platform.python_version(), "precheck": {"ok": ok, "why": why, "ref_started_epoch": (ref or {}).get("started_epoch"), "elapsed_at_ref_s": (ref or {}).get("elapsed_at_ref_s")}, "trials": []}
    say("[스텁] 기준 사전 점검(precheck_before_close): %s %s" % ("통과" if ok else "실패", why))
    if not ok: json.dump(rec, open(a.out, "w", encoding="utf-8"), ensure_ascii=False, indent=1); say("[스텁 중단] 기준 JSON 점검 실패"); sys.exit(2)
    say("\n[정렬 확인] (dry run 스텁: 정렬 보고 경로 시험, K1에서는 캡 미검출로 NG가 정상이다)"); rec["align_line_epoch"] = time.time(); time.sleep(a.align_wait)   # Mac이 정렬 보고를 끝낼 때까지 기다린다(첫 D에 섞이지 않게)
    q = [float(x) for x in a.q.split(",")]; extra = " q=" + ",".join("%.3f" % v for v in q)
    for label, name, wait in schedule(a.n_warm, a.idle1, a.idle2, a.gap):
        time.sleep(wait); t_req = time.time(); tok = ch.request(name, a.run_id, extra if name == "A" else ""); r = ch.wait(name, a.wait_cap, tok); t_rx = time.time()
        t = {"label": label, "name": name, "req_epoch": t_req, "rx_epoch": t_rx, "D_s": (t_rx - t_req) if r else None, "ok": r is not None, "request_id_match": bool(r) and r.get("request_id") == tok,
             "same_start": bool(r) and L.same_start([ref.get("started_epoch"), r.get("started_epoch")]), "image_sha256": bool(r and r.get("image_sha256")), "gate": (r or {}).get("gate"), "mac": (r or {}).get("mac")}
        rec["trials"].append(t); say("[D] " + json.dumps({k: t[k] for k in ("label", "name", "D_s", "ok", "request_id_match", "same_start")}, ensure_ascii=False))
        json.dump(rec, open(a.out, "w", encoding="utf-8"), ensure_ascii=False, indent=1, default=str)
    say("[스텁 완료] 시험 %d회" % len(rec["trials"]))


if __name__ == "__main__": main()
