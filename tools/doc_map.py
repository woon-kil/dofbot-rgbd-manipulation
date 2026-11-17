#!/usr/bin/env python3
"""설계 문서 지도 생성기 (읽기 전용, 표준 라이브러리만, 하드웨어 명령 없음).

범위: docs/gates/ 에서 파일명에 '설계'가 든 문서 + 명시 포함 목록(INCLUDE). 각 문서에서 아래를 뽑는다.
- 제목(첫 '# ' 줄), 마지막 커밋일(git log), 결정 칸 패턴 [x] [~] [ ] 수
- 본문의 백틱 안 `...md` 인용(앵커 제거) → 실제 파일로 해석(저장소 루트, docs/gates, docs/adr, docs, docs/handoff 순). 해석 못 하면 '미해결'로 센다.
인용은 네 종류로 나눈다: 범위 안 문서 / ADR / roadmap / 그 밖(승인 문서·결과 보고서 등 — 선을 그리지 않고 수만 센다).
출력: docs/architecture/design-doc-map.md, docs/architecture/design-doc-map.svg (그린 선 = 뽑은 선을 assert로 확인).
사용: python3 tools/doc_map.py [--check]   (--check: 파일을 쓰지 않고 요약만 출력)
"""
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
GATES = ROOT / "docs" / "gates"
INCLUDE = ["G07-첫접촉시험-선행조건과-최소시험안.md"]
CITE = re.compile(r"`([^`\n]*?\.md)(?:#[^`\n]*)?`")
BOX = re.compile(r"\[([x~ ])\]")


def scope_files(gates=GATES, include=INCLUDE):
    names = sorted(p.name for p in gates.glob("*설계*.md"))
    for n in include:
        if n not in names and (gates / n).exists():
            names.append(n)
    return names


def resolve(ref, root=ROOT):
    """인용 문자열 -> 저장소 안 실제 파일의 root 기준 상대 경로(str) 또는 None."""
    for base in ("", "docs/gates/", "docs/adr/", "docs/", "docs/handoff/"):
        p = root / (base + ref)
        if p.is_file():
            return str(p.relative_to(root))
    return None


def classify(rel, scope_rel):
    if rel in scope_rel:
        return "scope"
    if rel.startswith("docs/adr/"):
        return "adr"
    if rel == "docs/gates/roadmap.md":
        return "roadmap"
    return "other"


def analyze(root=ROOT, gates=None, include=None):
    gates = gates or root / "docs" / "gates"
    include = INCLUDE if include is None else include
    names = scope_files(gates, include)
    scope_rel = {str((gates / n).relative_to(root)) for n in names}
    docs = []
    for n in names:
        path = gates / n
        text = path.read_text(encoding="utf-8")
        rel = str(path.relative_to(root))
        title = next((l[2:].strip() for l in text.splitlines() if l.startswith("# ")), n)
        boxes = {k: 0 for k in "x~ "}
        for m in BOX.finditer(text):
            boxes[m.group(1)] += 1
        refs = sorted({m.group(1) for m in CITE.finditer(text)})
        cites = {"scope": set(), "adr": set(), "roadmap": set(), "other": set()}
        unresolved = []
        for r in refs:
            t = resolve(r, root)
            if t is None:
                unresolved.append(r)
            elif t != rel:
                cites[classify(t, scope_rel)].add(t)
        docs.append({"rel": rel, "name": n, "title": title, "boxes": boxes,
                     "cites": cites, "unresolved": unresolved, "date": last_date(rel, root)})
    return docs


def last_date(rel, root=ROOT):
    try:
        out = subprocess.run(["git", "-C", str(root), "log", "-1", "--format=%ad", "--date=format:%m-%d", "--", rel],
                             capture_output=True, text=True, timeout=20).stdout.strip()
        return out or "미커밋"
    except Exception:
        return "?"


def edges(docs):
    """SVG에 그릴 선: 범위 안 문서 -> (범위 안 문서 | ADR | roadmap)."""
    e = set()
    for d in docs:
        for kind in ("scope", "adr", "roadmap"):
            for t in d["cites"][kind]:
                e.add((d["rel"], t))
    return e


def short(rel):
    n = Path(rel).stem
    return n.replace("G07-", "").replace("-설계-초안", "").replace("-설계초안", "")


def render_md(docs, gen_date):
    L = ["# 설계 문서 지도 (자동 생성)", "",
         f"생성: `python3 tools/doc_map.py` · 기준일 {gen_date} · 범위: `docs/gates/` 파일명에 '설계'가 든 문서와 명시 포함 목록 {INCLUDE}.",
         "읽기 전용 지도다. 결정 칸 수는 `[x] [~] [ ]` 패턴을 센 것이라 결정 외 체크 표기도 섞일 수 있다. 상태 판정은 각 문서와 `PROJECT-STATUS.md`가 기준이다.", "",
         "| 문서 | 제목 | 수정일 | 결정 칸 [x]/[~]/[ ] | 범위 안 인용 | ADR | roadmap | 그 밖의 인용 수 | 미해결 인용 |",
         "|---|---|---|---|---|---|---|---|---|"]
    for d in docs:
        b = d["boxes"]; c = d["cites"]
        box = "칸 없음" if sum(b.values()) == 0 else f"{b['x']}/{b['~']}/{b[' ']}"
        sc = ", ".join(short(x) for x in sorted(c["scope"])) or "-"
        adr = ", ".join(Path(x).stem[:7] for x in sorted(c["adr"])) or "-"
        un = ", ".join(f"`{u}`" for u in d["unresolved"]) or "0"
        L.append(f"| `{d['name']}` | {d['title'][:48]} | {d['date']} | {box} | {sc} | {adr} | {'예' if c['roadmap'] else '-'} | {len(c['other'])} | {un} |")
    L += ["", "그림: `docs/architecture/design-doc-map.svg` (범위 안 문서·ADR·roadmap 사이 선만 그린다).", ""]
    return "\n".join(L)


def render_svg(docs):
    scope_nodes = [d["rel"] for d in docs]
    targets = sorted({t for d in docs for k in ("adr", "roadmap") for t in d["cites"][k]})
    W, rowh, bw, bh = 1120, 54, 300, 38
    top = 90
    H = top + max(len(scope_nodes), len(targets)) * rowh + 60
    pos = {}
    for i, r in enumerate(scope_nodes):
        pos[r] = (40, top + i * rowh)
    for i, r in enumerate(targets):
        pos[r] = (W - 40 - bw, top + i * rowh)
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" role="img" aria-label="설계 문서 지도">',
           "<title>설계 문서 지도</title>",
           "<style>:root{--bg:#fbfaf7;--fg:#1c1b19;--mut:#6b675f;--line:#8a857b;--f1:#e3edf8;--s1:#2f6db3;--f2:#fbeadb;--s2:#b5601a}"
           "@media (prefers-color-scheme:dark){:root{--bg:#1b1a18;--fg:#efece5;--mut:#a29d92;--line:#8d887e;--f1:#1e2f44;--s1:#6aa6e6;--f2:#3b2a1b;--s2:#e0904a}}"
           'text{font-family:-apple-system,"Apple SD Gothic Neo","Noto Sans KR",sans-serif;fill:var(--fg);font-size:12px}'
           ".m{fill:var(--mut)}.e{fill:none;stroke:var(--line);stroke-width:1.4}</style>",
           '<defs><marker id="a" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto"><path d="M0 0L10 5L0 10z" fill="var(--line)"/></marker></defs>',
           f'<rect width="{W}" height="{H}" fill="var(--bg)"/>',
           '<text x="40" y="30" style="font-size:18px;font-weight:600">설계 문서 지도 — 화살표는 "본문에서 인용한다" 방향</text>',
           f'<text x="40" y="52" class="m">docs/gates 설계 문서 {len(docs)}개의 백틱 .md 인용을 읽어 그렸다. 범위 안 문서·ADR·roadmap 사이 선만 그리고 승인 문서·결과 보고서 인용은 표의 수로만 센다(그림 {len(edges(docs))}선 = 추출 {len(edges(docs))}선).</text>']
    drawn = set()
    # 범위 안 문서 사이 선은 오른쪽 여백(x=bw+40 이후 곡선), ADR/roadmap 선은 직선
    for (a, b) in sorted(edges(docs)):
        xa, ya = pos[a]; xb, yb = pos[b]
        if b in pos and b in scope_nodes:
            x1, y1 = xa + bw, ya + bh / 2; x2, y2 = xb + bw, yb + bh / 2
            bend = bw + 40 + 40 + abs(scope_nodes.index(a) - scope_nodes.index(b)) * 10
            out.append(f'<path class="e" d="M{x1},{y1} C{bend},{y1} {bend},{y2} {x2 + 2},{y2}" marker-end="url(#a)"/>')
        else:
            out.append(f'<path class="e" d="M{xa + bw},{ya + bh / 2} L{xb - 2},{yb + bh / 2}" marker-end="url(#a)"/>')
        drawn.add((a, b))
    for r in scope_nodes:
        x, y = pos[r]
        out.append(f'<rect x="{x}" y="{y}" width="{bw}" height="{bh}" rx="7" fill="var(--f1)" stroke="var(--s1)" stroke-width="1.4"/>')
        out.append(f'<text x="{x + 10}" y="{y + 16}" style="font-weight:600">{short(r)[:30]}</text>')
        d = next(dd for dd in docs if dd["rel"] == r); b = d["boxes"]
        box = "결정 칸 없음" if sum(b.values()) == 0 else f"칸 {b['x']}/{b['~']}/{b[' ']} · 그 밖의 인용 {len(d['cites']['other'])}"
        out.append(f'<text x="{x + 10}" y="{y + 31}" class="m">{box}</text>')
    for r in targets:
        x, y = pos[r]
        out.append(f'<rect x="{x}" y="{y}" width="{bw}" height="{bh}" rx="7" fill="var(--f2)" stroke="var(--s2)" stroke-width="1.4"/>')
        out.append(f'<text x="{x + 10}" y="{y + 24}" style="font-weight:600">{Path(r).stem[:34]}</text>')
    out.append("</svg>")
    assert drawn == edges(docs), "그린 선과 추출한 선이 다르다"
    return "\n".join(out)


def main(argv):
    docs = analyze()
    e = edges(docs)
    print(f"범위 {len(docs)}개 문서, 선 {len(e)}개, 미해결 인용 {sum(len(d['unresolved']) for d in docs)}개")
    for d in docs:
        c = d["cites"]
        print(f"  {d['name'][:44]:44s} 칸={sum(d['boxes'].values())} 범위={len(c['scope'])} ADR={len(c['adr'])} roadmap={len(c['roadmap'])} 그밖={len(c['other'])} 미해결={d['unresolved']}")
    if "--check" in argv:
        return 0
    import datetime
    out = ROOT / "docs" / "architecture"
    (out / "design-doc-map.md").write_text(render_md(docs, datetime.date.today().isoformat()), encoding="utf-8")
    (out / "design-doc-map.svg").write_text(render_svg(docs), encoding="utf-8")
    print("wrote docs/architecture/design-doc-map.md, design-doc-map.svg")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
