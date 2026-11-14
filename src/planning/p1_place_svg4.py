"""③ P1 블록 배치 재확인 SVG(Gemini 사진 2026-11-15 2차) + 윗면 마커 부착 도식. 사진 위 요소는 호모그래피(종이 평면)로 그리고 역산 검증한다."""
import base64
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
PHOTO = ROOT / "data/g04/p1_block_check4_261115.png"
PX = np.float32([[653.8, 301.3], [747.5, 305.0], [748.8, 372.5], [652.5, 368.8]])
WD = np.float32([[-36, 199], [36, 199], [36, 259], [-36, 259]])
H = cv2.getPerspectiveTransform(WD, PX); HI = np.linalg.inv(H)
CROP = (540, 90, 1000, 700); SC = 1.6; OX, OY = 20.0, 50.0
def P(x, y): v = H @ np.array([x, y, 1.0]); return v[:2] / v[2]
def S(p): return (OX + (p[0] - CROP[0]) * SC, OY + (p[1] - CROP[1]) * SC)
def inv(sx, sy):
    px = ((sx - OX) / SC + CROP[0], (sy - OY) / SC + CROP[1]); v = HI @ np.array([px[0], px[1], 1.0]); return v[:2] / v[2]
els, checks = [], []
def poly(id_, pts, cls):
    sp = [S(P(*p)) for p in pts]; els.append('<polygon id="%s" class="%s" points="%s"/>' % (id_, cls, " ".join("%.2f,%.2f" % q for q in sp))); checks.append((id_, pts, sp))
def line(id_, a, b, cls, arrow=False):
    sa, sb = S(P(*a)), S(P(*b)); els.append('<line id="%s" class="%s" x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f"%s/>' % (id_, cls, sa[0], sa[1], sb[0], sb[1], ' marker-end="url(#ah)"' if arrow else "")); checks.append((id_, [a, b], [sa, sb]))
def text(x, y, t): els.append('<text x="%.1f" y="%.1f" class="lb">%s</text>' % (x, y, t))

def build():
    img = cv2.imread(str(PHOTO)); c = img[CROP[1]:CROP[3], CROP[0]:CROP[2]]; c = cv2.resize(c, None, fx=SC, fy=SC, interpolation=cv2.INTER_CUBIC)
    ok, buf = cv2.imencode(".jpg", c, [cv2.IMWRITE_JPEG_QUALITY, 88]); b64 = base64.b64encode(buf.tobytes()).decode(); W_, H_ = c.shape[1], c.shape[0]
    head = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="1000" viewBox="0 0 1000 1000">',
            '<style>.lb{font:13px sans-serif;fill:#fff;paint-order:stroke;stroke:#000;stroke-width:3px}.hd{font:bold 17px sans-serif;fill:#000}.nt{font:13px sans-serif;fill:#222}'
            '.tgt{fill:rgba(0,200,0,.20);stroke:#0c0;stroke-width:2.5;stroke-dasharray:6 4}.act{fill:rgba(255,140,0,.18);stroke:#f80;stroke-width:2.5}.edge{stroke:#f33;stroke-width:2.5}.axis{stroke:#f0f;stroke-width:1.5;stroke-dasharray:5 4}'
            '.dim{stroke:#0ff;stroke-width:2.5}.rm{fill:none;stroke:#f33;stroke-width:3}</style>',
            '<defs><marker id="ah" markerWidth="8" markerHeight="8" refX="6" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 z" fill="#0ff"/></marker></defs>',
            '<rect width="1000" height="1000" fill="#fff"/>',
            '<text x="20" y="26" class="hd">③ P1 블록 위치 조정 안내 (Gemini 2026-11-15 4차, 승인 아님)</text>',
            '<text x="20" y="44" class="nt">위=베이스 쪽, 아래=사용자 쪽, 오른쪽=프로젝트 +x. 호모그래피(종이 평면)로 그림, 보드가 없어 같은 1차 호모그래피 재사용(카메라 이동 0.6 px 이하 확인).</text>',
            '<image x="%.0f" y="%.0f" width="%d" height="%d" href="data:image/jpeg;base64,%s"/>' % (OX, OY, W_, H_, b64)]
    poly("target", [(-15, 160), (15, 160), (15, 220), (-15, 220)], "tgt")
    poly("actual", [(-12.4, 218.9), (19.5, 218.9), (19.5, 278.9), (-12.4, 278.9)], "act")
    line("move", (3.6, 248.9), (0, 190.0), "dim", arrow=True)
    line("edge73", (-92, 73), (92, 73), "edge"); line("axis", (0, 0), (0, 235), "axis")
    line("dim87", (45, 73), (45, 160), "dim", arrow=True); line("tick_a", (40, 73), (50, 73), "dim"); line("tick_b", (40, 160), (50, 160), "dim")
    sx, sy = S(P(45, 116)); text(sx + 8, sy, "87 mm (판 앞 가장자리 → 블록 앞면)")
    sx, sy = S(P(-92, 73)); text(sx + 4, sy - 6, "판 앞 가장자리 y=73 (실측)")
    sx, sy = S(P(15, 218)); text(sx + 8, sy + 36, "초록 = 목표 y 160~220, 주황 = 현재(앞 모서리 y 278.9) → 베이스 쪽 약 59 mm")
    # 제거 후보 마커 두 개(사진 검출 px): ArUco 4x4 id 1 (862,205) 변 60 px, id 0 (922,650) 변 102 px
    for nm, (cx, cy), sd in (("id 1", (862.0, 205.0), 60.0), ("id 0", (922.0, 650.0), 102.0)):
        a = S((cx - sd * 0.62, cy - sd * 0.62)); b = S((cx + sd * 0.62, cy + sd * 0.62))
        els.append('<rect class="rm" x="%.1f" y="%.1f" width="%.1f" height="%.1f"/>' % (a[0], a[1], b[0] - a[0], b[1] - a[1]))
        text(a[0] - 120 if nm == "id 1" else a[0] - 150, a[1] - 6, "제거 후보 ArUco 4×4 %s" % nm)
    foot = ['<text x="20" y="985" class="nt">마커 AprilTag id 7(25 mm)은 윗면에 붙어 있음. 지도 인쇄 마커(id 1)는 블록 바로 뒤(베이스 쪽)에 보임.</text>', '</svg>']
    return "\n".join(head + els + foot)

def verify():
    rows = []
    for id_, mm, sp in checks:
        got = [tuple(inv(*q)) for q in sp]; err = max(np.linalg.norm(np.array(g) - np.array(w)) for g, w in zip(got, mm))
        rows.append((id_, [tuple(round(v, 1) for v in w) for w in mm], [tuple(round(float(v), 1) for v in g) for g in got], round(float(err), 3)))
    return rows

# --- 윗면 마커 부착 도식 (도식, 사진 아님) ---
def attach_svg():
    K = 9.0; ox, oy = 150.0, 130.0      # px/mm, 블록 윗면 좌상단(앞뒤: 위=베이스, 폭: 왼쪽=−x)
    Wd, Ln, M = 30.0, 60.0, 20.0
    def X(x): return ox + K * (x + Wd / 2)
    def Yv(y): return oy + K * (y + Ln / 2)
    out = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="1000" viewBox="0 0 1000 1000">', '<style>.t{font:14px sans-serif;fill:#000}.hd{font:bold 17px sans-serif}.d{stroke:#06c;stroke-width:1.5}.blk{fill:#ffe066;stroke:#a80;stroke-width:2}.mk{fill:#111}.mkw{fill:#fff}</style>',
           '<rect width="1000" height="1000" fill="#fff"/>', '<text x="20" y="28" class="hd">③ 블록 윗면 마커 부착 도식 (도식, 사진 아님)</text>',
           '<text x="20" y="50" class="t">위에서 본 블록 윗면. 위=베이스 쪽, 아래=사용자 쪽. 단위 mm.</text>']
    chk = []
    def rect(id_, x0, y0, x1, y1, cls):
        out.append('<rect id="%s" class="%s" x="%.2f" y="%.2f" width="%.2f" height="%.2f"/>' % (id_, cls, X(x0), Yv(y0), K * (x1 - x0), K * (y1 - y0))); chk.append((id_, (x0, y0, x1, y1), (X(x0), Yv(y0), X(x0) + K * (x1 - x0), Yv(y0) + K * (y1 - y0))))
    rect("block_top", -15, -30, 15, 30, "blk"); rect("marker_quiet", -14, -14, 14, 14, "mkw"); rect("marker", -10, -10, 10, 10, "mk")
    for (a, b, c, d, lab) in ((-15, 36, 15, 36, "블록 폭 30"), (-10, 42, 10, 42, "마커 20")):
        out.append('<line class="d" x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f"/><text x="%.1f" y="%.1f" class="t">%s</text>' % (X(a), Yv(b), X(c), Yv(d), X(c) + 8, Yv(b) + 5, lab))
    out.append('<line class="d" x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f"/><text x="%.1f" y="%.1f" class="t">블록 길이 60</text>' % (X(24), Yv(-30), X(24), Yv(30), X(24) + 6, Yv(0)))
    out.append('<text x="20" y="%.0f" class="t">마커: 인쇄 파일 results/aruco_4x4_id2_20mm_a4.pdf (ArUco DICT_4X4_50 id 2, 검은 정사각형 한 변 20 mm)</text>' % 830)
    out.append('<text x="20" y="%.0f" class="t">윗면 중앙에 붙임: 폭 방향 여유 각 5 mm, 길이 방향 여유 각 20 mm. 마커 가로변을 블록 짧은 변(30 mm)과 평행하게.</text>' % 854)
    out.append('<text x="20" y="%.0f" class="t">마커는 평평하게(주름·들뜸 없이) 양면테이프로 전면 접착. 마커 둘레 흰 여백 약 4 mm는 남기되 블록 가장자리를 넘지 않게 자름.</text>' % 878)
    out.append('</svg>')
    rows = []
    for id_, mm, sp in chk:
        x0 = (sp[0] - ox) / K - Wd / 2; y0 = (sp[1] - oy) / K - Ln / 2; x1 = (sp[2] - ox) / K - Wd / 2; y1 = (sp[3] - oy) / K - Ln / 2
        rows.append((id_, tuple(round(v, 1) for v in mm), tuple(round(v, 2) for v in (x0, y0, x1, y1)), round(max(abs(a - b) for a, b in zip((x0, y0, x1, y1), mm)), 3)))
    return "\n".join(out), rows

if __name__ == "__main__":
    svg = build(); (ROOT / "data/g04/p1_placement_v4.svg").write_text(svg, encoding="utf-8")
    rows = verify(); print("[배치 재확인] 요소 %d개 역산 최대 오차 %.3f mm" % (len(rows), max(r[3] for r in rows)))
    for r in rows: print("  %-8s 글 %s → 역산 %s 오차 %.3f" % r)
