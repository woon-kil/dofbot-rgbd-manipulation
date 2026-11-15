#!/usr/bin/env python3
"""프로젝트 방향 점검 트리거(읽기 전용, 하드웨어 명령 없음, 표준 라이브러리만).

일정 단위 작업이 쌓이면 '방향 점검 필요'를 알리고(훅), 객관 지표를 출력한다(--status). 점검 자체는 Claude가 `docs/direction-checks/CHECKLIST.md`로 수행한다.
사용:
  python3 tools/direction_check.py --hook            # 훅용: 기준 도달 시에만 JSON(additionalContext) 출력, 아니면 침묵
  python3 tools/direction_check.py --status          # 지표 표(항상 출력)
  python3 tools/direction_check.py --mark-reviewed docs/<보고서>.md   # 점검을 마쳤을 때 기준점 갱신
기준(환경변수로 덮어쓰기 가능, 기본값은 제안이며 사용자가 조정): 마지막 점검 이후 커밋 DC_COMMITS=8, 실기기 `--real` 기록 DC_REAL=3, 경과일 DC_DAYS=2.
"""
import json
import os
import re
import subprocess
import sys
from datetime import date, datetime
from pathlib import Path

ROOT = Path(subprocess.run(["git", "rev-parse", "--show-toplevel"], capture_output=True, text=True).stdout.strip() or ".")
STATE = ROOT / "docs/direction-checks/state.json"
THRESH = {"commits": int(os.environ.get("DC_COMMITS", 8)), "real": int(os.environ.get("DC_REAL", 3)), "days": int(os.environ.get("DC_DAYS", 2))}


def git(*a):
    return subprocess.run(["git", *a], cwd=ROOT, capture_output=True, text=True).stdout.strip()


def real_count():
    return len(list((ROOT / "evidence/G01/raw").glob("*real*.json"))) if (ROOT / "evidence/G01/raw").exists() else 0


def metrics():
    st = json.loads(STATE.read_text(encoding="utf-8")) if STATE.exists() else {}
    last = st.get("last_review_commit")
    commits = int(git("rev-list", "--count", f"{last}..HEAD") or 0) if last else None
    days = (date.today() - date.fromisoformat(st["last_review_date"])).days if st.get("last_review_date") else None
    new_real = real_count() - st.get("last_real_count", real_count()) if st else None
    ssot = (ROOT / "PROJECT-STATUS.md").read_text(encoding="utf-8") if (ROOT / "PROJECT-STATUS.md").exists() else ""
    m = re.search(r"마지막 갱신:\s*(\d{4}-\d{2}-\d{2})", ssot)
    head_date = git("log", "-1", "--format=%ad", "--date=short")
    ssot_lag = (date.fromisoformat(head_date) - date.fromisoformat(m.group(1))).days if m and head_date else None
    dec = ROOT / "docs/gates/열린-결정-정리표.md"
    open_dec = len(re.findall(r"\|\s*\[ \]\s*\|\s*$", dec.read_text(encoding="utf-8"), re.M)) if dec.exists() else None
    dirty = len([l for l in git("status", "--short").splitlines() if l.strip()])
    snap = None
    mf = sorted((ROOT / "evidence/scripts_snapshot").glob("*/MANIFEST.sha256"))
    if mf and (ROOT / "scripts").exists():
        import hashlib
        diff = 0
        for line in mf[-1].read_text(encoding="utf-8").splitlines():
            h, _, name = line.partition("  ")
            p = ROOT / "scripts" / name.strip()
            if p.exists() and hashlib.sha256(p.read_bytes()).hexdigest() != h:
                diff += 1
        snap = diff
    return {"commits_since_review": commits, "new_real_runs": new_real, "days_since_review": days, "ssot_lag_days": ssot_lag, "open_decisions": open_dec, "uncommitted_files": dirty, "scripts_changed_vs_snapshot": snap, "last_review": st.get("last_review_report")}


def due(m):
    why = []
    if m["commits_since_review"] is None:
        return ["기준점 없음(`--mark-reviewed`로 첫 기준점을 만들어야 한다)"]
    if m["commits_since_review"] >= THRESH["commits"]:
        why.append("커밋 %d개(기준 %d)" % (m["commits_since_review"], THRESH["commits"]))
    if m["new_real_runs"] is not None and m["new_real_runs"] >= THRESH["real"]:
        why.append("신규 실기기 실행 기록 %d개(기준 %d)" % (m["new_real_runs"], THRESH["real"]))
    if m["days_since_review"] is not None and m["days_since_review"] >= THRESH["days"]:
        why.append("경과 %d일(기준 %d)" % (m["days_since_review"], THRESH["days"]))
    return why


def table(m):
    rows = [("마지막 점검 이후 커밋", m["commits_since_review"], THRESH["commits"]), ("신규 실기기 실행 기록", m["new_real_runs"], THRESH["real"]), ("경과일", m["days_since_review"], THRESH["days"]),
            ("SSOT 갱신 지연(일, HEAD 커밋일 − PROJECT-STATUS 갱신일, 참고)", m["ssot_lag_days"], None), ("열린 결정(미확인 `[ ]`)", m["open_decisions"], None), ("미커밋 파일", m["uncommitted_files"], None),
            ("스냅샷과 다른 실행 스크립트 수", m["scripts_changed_vs_snapshot"], None)]
    return "\n".join("- %s: %s%s" % (n, v, (" (기준 %s)" % t) if t is not None else "") for n, v, t in rows) + ("\n- 마지막 점검 보고서: %s" % m["last_review"] if m["last_review"] else "")


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else "--status"
    if arg == "--mark-reviewed":
        rep = sys.argv[2] if len(sys.argv) > 2 else ""
        STATE.parent.mkdir(parents=True, exist_ok=True)
        STATE.write_text(json.dumps({"last_review_commit": git("rev-parse", "HEAD"), "last_review_date": date.today().isoformat(), "last_review_report": rep, "last_real_count": real_count(), "marked_at": datetime.now().isoformat(timespec="seconds")}, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
        print("기준점 갱신:", rep); return
    m = metrics(); why = due(m)
    if arg == "--status":
        print(table(m)); print("판정:", ("방향 점검 필요 — " + "; ".join(why)) if why else "아직 기준 미달"); return
    if arg == "--hook":
        if not why:
            return
        msg = ("[방향 점검 알림] 일정 단위 작업이 쌓였다: " + "; ".join(why) + ".\n" + table(m) + "\n지금 진행 중인 작업을 멈추고 이번 응답의 끝에서 **방향 점검을 수행**하라: ① `docs/direction-checks/CHECKLIST.md`의 질문으로 현재 방향을 평가(근거를 읽고, 추측 금지), "
               "② `docs/YYMMDD-HHMM-방향-점검-보고서.md` 작성(양호/주의/경고, 권고, 한계), ③ `python3 tools/direction_check.py --mark-reviewed <보고서 경로>`로 기준점 갱신, ④ 사용자에게 요약 보고하고 결정이 필요한 것만 묻는다. 사용자가 '점검 보류'라고 하면 기준점은 갱신하지 않는다.")
        print(json.dumps({"hookSpecificOutput": {"hookEventName": os.environ.get("DC_EVENT", "PostToolUse"), "additionalContext": msg}}, ensure_ascii=False)); return
    print(__doc__)


if __name__ == "__main__":
    main()
