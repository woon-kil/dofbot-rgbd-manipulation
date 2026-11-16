#!/usr/bin/env python3
"""인용한 실기기 기록(JSON)의 추적 복사본 관리·검증 (2026-11-16, 사용자 승인: 인용한 기록 JSON만 복사해 추적하고 해시를 남긴다).
`evidence/**/raw/`는 .gitignore로 추적하지 않는다(사진·중복·개인 정보 위험). 문서가 근거로 인용하는 JSON만 `evidence/records/`에 복사하고 `evidence/records/MANIFEST.tsv`에 sha256을 남긴다.
사용:
  python3 tools/verify_records.py                 # 검증: 복사본 해시 = 매니페스트, 원본이 로컬에 있으면 원본 해시 = 매니페스트
  python3 tools/verify_records.py --add <raw 경로> [...]   # 원본을 복사하고 매니페스트에 추가(이미 있으면 갱신 금지, 불일치면 오류)
규칙: 사진(.png/.jpg)·영상·.npy·G00 원문 로그는 복사하지 않는다. 한 파일 5 MB 초과는 거부한다."""
import sys, os, re, hashlib, shutil
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); os.chdir(ROOT)
MAN = "evidence/records/MANIFEST.tsv"
def sha(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""): h.update(b)
    return h.hexdigest()
def load():
    rows = []
    if os.path.exists(MAN):
        for l in open(MAN, encoding="utf-8"):
            l = l.rstrip("\n")
            if l and not l.startswith("#"): rows.append(l.split("\t"))
    return rows
def dest_for(raw):
    m = re.match(r"evidence/([^/]+)/raw/(.+)$", raw)
    if not m: sys.exit("raw 경로 형식이 아니다: %s" % raw)
    return "evidence/records/%s/%s" % (m.group(1), m.group(2).replace("/", "__"))
def add(raws):
    rows = load(); have = {r[2]: r for r in rows}; new = []
    for raw in raws:
        if not os.path.isfile(raw): print("없음(건너뜀):", raw); continue
        if not raw.endswith(".json"): print("JSON이 아니라 건너뜀:", raw); continue
        if os.path.getsize(raw) > 5 << 20: print("5 MB 초과 거부:", raw); continue
        d = dest_for(raw); s = sha(raw)
        if raw in have:
            if have[raw][0] != s: sys.exit("이미 등록된 원본의 해시가 달라졌다(갱신 금지): %s" % raw)
            continue
        os.makedirs(os.path.dirname(d), exist_ok=True); shutil.copyfile(raw, d)
        assert sha(d) == s; new.append([s, d, raw])
    rows += new
    with open(MAN, "w", encoding="utf-8") as f:
        f.write("# sha256\t추적 복사본\t원본(로컬 raw, 추적 안 함)\n")
        for r in sorted(rows, key=lambda r: r[2]): f.write("\t".join(r) + "\n")
    print("추가 %d, 전체 %d" % (len(new), len(rows)))
def verify():
    rows = load(); bad = 0; nolocal = 0
    for s, d, raw in rows:
        if not os.path.isfile(d): print("복사본 없음:", d); bad += 1; continue
        if sha(d) != s: print("복사본 해시 불일치:", d); bad += 1
        if os.path.isfile(raw):
            if sha(raw) != s: print("원본이 복사본과 다름(원본 변경 의심):", raw); bad += 1
        else: nolocal += 1
    print("항목 %d, 복사본 정상 %d, 원본 로컬 없음 %d, 문제 %d" % (len(rows), len(rows) - bad, nolocal, bad)); return 1 if bad else 0
if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--add": add(sys.argv[2:])
    else: sys.exit(verify())
