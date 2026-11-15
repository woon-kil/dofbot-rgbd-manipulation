#!/usr/bin/env python3
"""진행 지도 데이터 생성기 (읽기 전용, 표준 라이브러리만, 하드웨어 명령 없음).

입력
- docs/progress-map/status.json : 관문·접촉 파일럿 단계·서보 시험·남은 것의 상태(사람이 고침)
- docs/gates/열린-결정-정리표.md  : 열린 결정 건수를 직접 센다([x] 확정, [~] 부분, [ ] 미확인)

출력(--out 폴더, 기본 data/progress-map/ 는 gitignore):
gates.json, pilot.json, servo.json, remaining.json, decisions.json
이 파일들을 진행 지도 아티팩트의 데이터 파일로 올린다(절차는 docs/progress-map/README.md).

사용:
  python3 tools/progress_map.py            # 파일 생성 + 요약 출력
  python3 tools/progress_map.py --check    # 정리표 건수와 status.json 구조만 점검, 파일은 만들지 않음
"""
import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
STATUS = ROOT / "docs/progress-map/status.json"
TABLE = ROOT / "docs/gates/열린-결정-정리표.md"
STATES = {"done", "cond", "partial", "draft", "wait", "none"}
NEED = {
    "gates": {"id", "label", "name", "state", "state_ko", "note", "source", "current"},
    "pilot": {"id", "label", "name", "state", "state_ko", "note", "source"},
    "servo": {"id", "label", "name", "state", "state_ko", "note", "source"},
    "remaining": {"id", "item", "state", "state_ko", "detail", "source"},
}


def count_decisions():
    """정리표의 번호 행(| 1 | ... | 확인칸 |)에서 마지막 칸의 표시로 센다."""
    n = {"done": 0, "partial": 0, "open": 0}
    rows = 0
    for line in TABLE.read_text(encoding="utf-8").splitlines():
        if not re.match(r"^\|\s*\d+\s*\|", line):
            continue
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        last = cells[-1]
        rows += 1
        if last.startswith("[x]"):
            n["done"] += 1
        elif last.startswith("[~]"):
            n["partial"] += 1
        elif last.startswith("[ ]"):
            n["open"] += 1
        else:
            raise SystemExit("확인 칸 형식을 알 수 없는 행: %r" % line[:80])
    src = "docs/gates/열린-결정-정리표.md"
    return [
        {"id": "total", "label": "열린 결정 전체", "count": rows, "source": src},
        {"id": "done", "label": "확정", "count": n["done"], "source": src + " ([x])"},
        {"id": "partial", "label": "부분 수용", "count": n["partial"], "source": src + " ([~])"},
        {"id": "open", "label": "미확인", "count": n["open"], "source": src + " ([ ])"},
    ]


def load_status():
    st = json.loads(STATUS.read_text(encoding="utf-8"))
    for key, need in NEED.items():
        ids = set()
        for r in st[key]:
            miss = need - set(r)
            if miss:
                raise SystemExit("%s 행 %s에 빠진 열: %s" % (key, r.get("id"), sorted(miss)))
            if r["state"] not in STATES:
                raise SystemExit("%s 행 %s의 state가 알 수 없는 값: %s" % (key, r["id"], r["state"]))
            if r["id"] in ids:
                raise SystemExit("%s에 같은 id가 두 번: %s" % (key, r["id"]))
            ids.add(r["id"])
    return st


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "data/progress-map"))
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    st = load_status()
    dec = count_decisions()
    by = {d["id"]: d["count"] for d in dec}
    if by["done"] + by["partial"] + by["open"] != by["total"]:
        raise SystemExit("결정 건수 합이 맞지 않음: %s" % by)
    print("as_of(status.json):", st["as_of"])
    print("관문 %d, 접촉 파일럿 %d, 서보 %d, 남은 것 %d" % tuple(len(st[k]) for k in ("gates", "pilot", "servo", "remaining")))
    print("열린 결정 %(total)d건: 확정 %(done)d, 부분 %(partial)d, 미확인 %(open)d" % by)
    if a.check:
        return 0
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    data = {k: st[k] for k in NEED}
    data["decisions"] = dec
    for name, rows in data.items():
        (out / (name + ".json")).write_text(json.dumps(rows, ensure_ascii=False, indent=0), encoding="utf-8")
    print("출력:", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
