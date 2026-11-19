#!/usr/bin/env python3
"""세트 33 마커 기준(재등록 1회 한정) 읽기·등록. 로봇·카메라 명령 없음(사진 파일만 읽는다).
사용자 결정(2026-11-18): 마커 기준을 현재 위치로 **이번 1회만** 재등록한다(들어올림 이후 실행은 E-5 물리 복원). 옛 기준 `p2k_marker_ref.txt`(676.25, 266.5)는 지우지 않고 기록으로 보존한다.
 - 읽기: `evidence/G06/lift_marker_ref_<YYMMDD-HHMM>.txt`가 **정확히 1개**일 때만 쓴다. 0개·2개 이상은 오류(Run 1 위치나 옛 기준으로 되돌아가지 않는다).
 - 등록: `register --photo <새 36 s 사진 png> --approval "<건별 승인 문구>"`: Run 4 복귀 뒤 사진(파일에서 검출, 하드코딩 없음)에서 마커가 1.0 px 이내일 때만 쓴다(문턱 불변). 이미 파일이 있으면 거부(덮어쓰기 없음). 시도는 모두 `evidence/G06/lift_marker_attempts.log`에 기록하고 3회 실패하면 더 받지 않는다(사용자에게 올림).
 - `--info`: 래퍼 질문 문구용 한 줄."""
import argparse, datetime, glob, hashlib, math, os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
G06 = os.path.join(ROOT, "evidence", "G06"); OLD_REF = os.path.join(G06, "p2k_marker_ref.txt"); ATT = os.path.join(G06, "lift_marker_attempts.log")
RUN4_POST = os.path.join(ROOT, "data", "g04", "p2kpost_261117-153705.png")   # 재등록이 비교하는 '마지막으로 알려진 블록 위치'(Run 4 복귀 뒤 사진)
GATE_PX = 1.0; MAX_ATTEMPTS = 3


class MarkerRefError(Exception): pass


def files(root=None): return sorted(glob.glob(os.path.join(root or ROOT, "evidence", "G06", "lift_marker_ref_*.txt")))


def load(root=None):
    """(x, y, J1 기대값, 파일명, 메타 줄 목록). 파일이 정확히 1개가 아니면 MarkerRefError."""
    fs = files(root)   # root=None이면 호출 시점의 ROOT
    if len(fs) != 1: raise MarkerRefError("세트 33 마커 기준 파일이 %d개다(정확히 1개 필요, evidence/G06/lift_marker_ref_*.txt). 재등록 전에는 시작하지 않는다." % len(fs))
    lines = open(fs[0], encoding="utf-8").read().splitlines(); data = [l for l in lines if l.strip() and not l.startswith("#")]
    x, y = (float(v) for v in data[0].split()[:2]); j1 = float(data[1].split()[0])
    return x, y, j1, os.path.basename(fs[0]), [l for l in lines if l.startswith("#")]


def j1_files(root=None): return sorted(glob.glob(os.path.join(root or ROOT, "evidence", "G06", "lift_j1_assumption_*.txt")))


def _j1_pair(root=None):
    """(명령 J1, 원시 가정 J1). `evidence/G06/lift_j1_assumption_<시각>.txt`가 **정확히 1개**이고 데이터 줄이 정확히 1줄 `<명령 정수> <원시 실수>`일 때만. 아니면 MarkerRefError(정렬 NG, fail-closed).
    마커 기준 파일(lift_marker_ref_*.txt, 정확히 1개 규칙)과 분리된 파일이라 마커 좌표·옛 파일은 건드리지 않는다(플랜 v2 §4 J1 85)."""
    fs = j1_files(root)
    if len(fs) != 1: raise MarkerRefError("J1 가정 파일이 %d개다(정확히 1개 필요, evidence/G06/lift_j1_assumption_*.txt)." % len(fs))
    data = [l for l in open(fs[0], encoding="utf-8").read().splitlines() if l.strip() and not l.startswith("#")]
    try:
        if len(data) != 1 or len(data[0].split()) != 2: raise ValueError
        cmd, raw = int(data[0].split()[0]), float(data[0].split()[1])
        if cmd <= 0 or raw <= 0: raise ValueError
    except ValueError: raise MarkerRefError("J1 가정 파일 형식 오류(데이터 1줄 `<명령 정수> <원시 실수>`): " + os.path.basename(fs[0]))
    return cmd, raw


def j1_assumption(root=None): return _j1_pair(root)[1]


def j1_command(root=None): return _j1_pair(root)[0]


def check_plan_j1(plan_path, root=None):
    """계획(세트 33)의 접촉 자세 J1 명령이 가정 파일의 명령 J1과 같은지 대조한다. 다르면 MarkerRefError(정렬 NG)."""
    import json
    plan = json.load(open(plan_path, encoding="utf-8")); k = plan["lift"]["contact_moves_done"]; j1 = int(plan["return_routes"][k]["state"][0]); cmd = j1_command(root)
    if j1 != cmd: raise MarkerRefError("계획의 접촉 자세 J1 명령 %d가 J1 가정 파일의 명령 %d와 다르다." % (j1, cmd))
    return cmd


def j1_raw_for_plan(plan, root=None):
    """계획이 가정하는 J1 원시값(간격·정렬 도구용). **세트 번호가 아니라 계획의 접촉 자세 J1 명령**으로 정한다: 들어올림 계획(plan["lift"])이면 명령이 J1 가정 파일의 명령(85)과 같을 때 그 파일의 원시 가정(85.4),
    86이면 기존 관례 86.4, 그 밖이면 MarkerRefError. 들어올림이 아닌 계획은 기존 규칙(세트 31 → 86.4, 그 밖 → 87.4)."""
    if plan.get("lift"):
        j1 = int(round(float(plan["return_routes"][plan["lift"]["contact_moves_done"]]["state"][0])))
        if j1 == j1_command(root): return j1_assumption(root)
        if j1 == 86: return 86.4
        raise MarkerRefError("계획의 접촉 자세 J1 명령 %d에 대한 원시 가정이 없다(J1 가정 파일 명령 %d, 기존 86)." % (j1, j1_command(root)))
    return 86.4 if str(plan.get("set")) == "31" else 87.4


def _detect(path):
    import cv2; sys.path.insert(0, os.path.join(ROOT, "scripts")); import pa_detect
    d = pa_detect.detect(cv2.imread(path))
    if not d.get("found"): raise MarkerRefError("마커 id 7 미검출: " + path)
    return d


def _attempt_count():
    return sum(1 for l in open(ATT, encoding="utf-8")) if os.path.exists(ATT) else 0


def register(photo, approval, now=None):
    if not approval or not approval.strip(): raise MarkerRefError("건별 승인 문구(--approval)가 필요하다")
    if files(): raise MarkerRefError("이미 재등록되어 있다(이번 1회 한정, 덮어쓰기 없음): %s" % files()[0])
    if _attempt_count() >= MAX_ATTEMPTS: raise MarkerRefError("시도 %d회를 모두 썼다. 사용자에게 올린다(문턱은 넓히지 않는다)." % MAX_ATTEMPTS)
    now = now or datetime.datetime.now(); prev = _detect(RUN4_POST); new = _detect(photo)
    dist = math.hypot(new["center"][0] - prev["center"][0], new["center"][1] - prev["center"][1]); ok = dist <= GATE_PX + 1e-9
    with open(ATT, "a", encoding="utf-8") as f: f.write("%s photo=%s center=(%.2f, %.2f) side=%.2f dist_to_run4_post=%.2f px %s\n" % (now.strftime("%Y-%m-%d %H:%M"), os.path.basename(photo), new["center"][0], new["center"][1], new["side_px"], dist, "합격" if ok else "거부"))
    if not ok: raise MarkerRefError("새 사진 마커가 Run 4 복귀 뒤 위치에서 %.2f px(문턱 %.1f px): 블록이 Run 4 뒤 움직였거나 검출 잡음일 수 있다. 재촬영(남은 시도 %d회) 또는 사용자에게 올린다." % (dist, GATE_PX, MAX_ATTEMPTS - _attempt_count()))
    old = [l for l in open(OLD_REF, encoding="utf-8").read().splitlines() if l.strip() and not l.startswith("#")]
    out = os.path.join(G06, "lift_marker_ref_%s.txt" % now.strftime("%y%m%d-%H%M"))
    sha = hashlib.sha256(open(photo, "rb").read()).hexdigest()
    with open(out, "x", encoding="utf-8") as f:
        f.write("# 세트 33 마커 기준(재등록 1회 한정). 건별 승인: %s\n# 등록 시각 %s, 원사진 %s (sha256 %s)\n" % (approval.strip(), now.strftime("%Y-%m-%d %H:%M"), os.path.basename(photo), sha))
        f.write("# 비교 사진 %s의 마커 (%.2f, %.2f)에서 %.2f px(문턱 %.1f px). 옛 기준 p2k_marker_ref.txt(보존): %s / %s\n# 규칙: 재등록은 이번 1회, 들어올림 이후 실행은 E-5 물리 복원을 거친다. 정렬 NG 중단은 '시도 아님'.\n" % (os.path.basename(RUN4_POST), prev["center"][0], prev["center"][1], dist, GATE_PX, old[0], old[1]))
        f.write("%.2f %.2f\n%s\n" % (new["center"][0], new["center"][1], old[1].split()[0]))
    return out


def main():
    ap = argparse.ArgumentParser(); sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("register"); r.add_argument("--photo", required=True); r.add_argument("--approval", required=True)
    sub.add_parser("info"); sub.add_parser("check")
    a = ap.parse_args()
    try:
        if a.cmd == "register": print("재등록 완료:", register(a.photo, a.approval))
        else:
            x, y, j1, name, meta = load(); print("%s (%.2f, %.2f) 마커 파일의 옛 J1 %.1f / J1 가정 파일: 명령 %d → 원시 %.1f" % (name, x, y, j1, j1_command(), j1_assumption())) if a.cmd == "check" else print("%s 재등록 사진 (%.2f, %.2f)" % (name, x, y))
    except MarkerRefError as e: print("오류:", e); sys.exit(7)


if __name__ == "__main__": main()
