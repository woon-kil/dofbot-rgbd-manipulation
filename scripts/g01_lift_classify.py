#!/usr/bin/env python3
"""세트 33 실행 결과의 시도 분류(Mac 사후 계산, 로봇·카메라 없음). 사용자 결정(2026-11-18): 시도 여부의 단일 출처는 **접촉 어댑터 쓰기 로그**(`j6_adapter_logs.contact`, 중단 때도 finally에서 저장).
 - 닫기 첫 쓰기 이전의 중단(정렬 NG·사전 점검·높이 관문 등)은 '시도 아님'. dry_run·fake(`result["mode"] != "real"`)는 세지 않는다(가짜 실행도 로그에 'real'이 찍히므로 모드로 판단).
 - 닫기 쓰기가 한 번이라도 있으면 시도: 정체가 감지되면 '시도(정체)', 정체 없이 비정상 중단이면 '시도(접촉 미확정)'(사용자 결정). **정체 없이 150까지 정상 완료한 경우를 '시도(정체 없음)'으로 세는 것은 이 도구의 확장 해석이다.**
 - 시도의 결과는 lifted / no_lift와 사유 코드(G1·G2·G4·G5·P10·SAG_EARLY·SAG_LATE·CLOCK·PHOTO_A·P_A_START·STALL_<명령>·NO_STALL_150·ABORT). SAG_LATE는 첫 J2 이동이 한 번 나간 뒤 열린 경우(lifted_once).
 - **판정 기록 수(E1·E2·E3·Q3가 판정된 기록)는 계산하지 않는다**: R8(Mac 사후 계산) 구현 뒤 기준별 입력으로 계산한다(사진 B 유무로 대체하지 않는다).
사용: .venv/bin/python -I scripts/g01_lift_classify.py <결과 JSON>"""
import json, sys


def _run2(result):
    for p in result.get("poses", []) or []:
        r = p.get("run2")
        if r and ("close_steps" in r or "lift" in r or "hold_%s_open" % p.get("pose_id") in r): return r
    return None


def classify(result):
    out = {"judgement_record": "R8 구현 뒤 기준별 입력으로 계산(현재 미계산)"}
    if result.get("mode") != "real": return dict(out, counted=False, attempt="세지 않음", why="mode=%s(dry_run·fake는 세지 않는다)" % result.get("mode"))
    log = (result.get("j6_adapter_logs") or {}).get("contact") or []
    writes = [l for l in log if l and l[0] == "write" and not (len(l) > 4 and l[4] == "return_to_start")]
    if not writes: return dict(out, counted=False, attempt="시도 아님", why="닫기 첫 쓰기 이전에 끝남(정렬 NG·사전 점검·높이 관문 등)")
    r2 = _run2(result) or {}; steps = r2.get("close_steps"); lift = r2.get("lift")
    stall = r2.get("stall_readback") is not None or any(s.get("stall") for s in (steps or []))
    written = [l[1] for l in writes]; last_cmd = written[-1]
    if stall: attempt = "시도(정체)"
    elif steps is None: attempt = "시도(접촉 미확정)"
    else: attempt = "시도(정체 없음)"
    res = dict(out, counted=True, attempt=attempt, close_writes=written, wrote_145=145 in written)
    if lift and lift.get("code"):
        res.update(outcome="no_lift", code=lift["code"], lifted_once=bool(lift.get("lifted_once")))
    elif lift and any(m.get("step") == "lift3" and not m.get("aborted") for m in lift.get("moves", [])):
        res.update(outcome="lifted", code="")
    elif lift: res.update(outcome="no_lift", code=("ABORT" if any(m.get("aborted") for m in lift.get("moves", [])) else (lift.get("code") or "UNKNOWN")), lifted_once=bool(lift.get("moves")))
    elif stall: res.update(outcome="no_lift", code="STALL_%s" % (r2.get("stall_command") or last_cmd))
    elif steps is not None: res.update(outcome="no_lift", code="NO_STALL_%s" % last_cmd)
    else: res.update(outcome="no_lift", code="ABORT")
    late = ((lift or {}).get("sag") or {}).get("late") or {}
    if late.get("cap") is not None and late.get("plan_max") is not None and late["cap"] < late["plan_max"]:
        res["note"] = "처짐 cap으로 둘째 J2 목표가 %d→%d: E1 신호 약함 가능, 파지 실패의 증거로 쓰지 않는다. 탐색 단계의 이런 시도는 30회 0건 집계에 넣지 않는다(사용자 결정 2026-11-18)" % (late["plan_max"], late["cap"])
    return res


def main():
    if len(sys.argv) != 2: sys.exit(__doc__)
    c = classify(json.load(open(sys.argv[1], encoding="utf-8")))
    print("분류: %s%s%s" % (c["attempt"], (" + " + c["outcome"] + ((" (" + c["code"] + ")") if c.get("code") else "") + (", 들어올림 이동 뒤 열림(실행한 이동 수는 결과 JSON의 lift.moves)" if c.get("lifted_once") and c["outcome"] == "no_lift" else "")) if c.get("counted") else "", (" — " + c["why"]) if c.get("why") else ""))
    if c.get("note"): print("주의: " + c["note"])
    print("판정 기록: " + c["judgement_record"])


if __name__ == "__main__": main()
