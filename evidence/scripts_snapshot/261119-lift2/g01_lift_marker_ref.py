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
            x, y, j1, name, meta = load(); print("%s (%.2f, %.2f) J1 %.1f" % (name, x, y, j1)) if a.cmd == "check" else print("%s 재등록 사진 (%.2f, %.2f)" % (name, x, y))
    except MarkerRefError as e: print("오류:", e); sys.exit(7)


if __name__ == "__main__": main()
