"""③ P1 블록 배치 안내 SVG(Gemini 사진 위)와 역산 검증. 사진 속 보드 모서리 호모그래피(종이 평면)를 쓴다.
사진: data/g04/p1_block_check_261115.png (개인 공간이 찍힌 사진이라 gitignore된 data/g04에 저장)."""
import base64
import re
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
PHOTO = ROOT / "data/g04/p1_block_check_261115.png"
OUT = ROOT / "data/g04/p1_placement_v1.svg"
# 호모그래피: 1차 촬영 평판 보드 검은 무늬 네 모서리(눈으로 짚음) <-> 월드 mm(프로젝트 x 오른쪽, y 사용자 쪽)
PX = np.float32([[653.8, 301.3], [747.5, 305.0], [748.8, 372.5], [652.5, 368.8]])
WD = np.float32([[-36, 199], [36, 199], [36, 259], [-36, 259]])
H = cv2.getPerspectiveTransform(WD, PX)
HI = np.linalg.inv(H)
CROP = (540, 90, 860, 400)      # x0, y0, x1, y1 (원본 px)
SC = 3.0
OX, OY = 20.0, 50.0

def P(x, y):
    v = H @ np.array([x, y, 1.0]); return v[:2] / v[2]
def S(p):   # 원본 px -> SVG 좌표
    return (OX + (p[0] - CROP[0]) * SC, OY + (p[1] - CROP[1]) * SC)
def inv(sx, sy):   # SVG 좌표 -> 월드 mm
    px = ((sx - OX) / SC + CROP[0], (sy - OY) / SC + CROP[1]); v = HI @ np.array([px[0], px[1], 1.0]); return v[:2] / v[2]

els, checks = [], []
def poly(id_, pts_mm, cls, label=None, lp=None):
    sp = [S(P(*p)) for p in pts_mm]
    els.append('<polygon id="%s" class="%s" points="%s"/>' % (id_, cls, " ".join("%.2f,%.2f" % q for q in sp)))
    checks.append((id_, [tuple(p) for p in pts_mm], sp))
    if label: els.append('<text x="%.1f" y="%.1f" class="lb">%s</text>' % (lp[0], lp[1], label))
def line(id_, a_mm, b_mm, cls, label=None, lp=None, arrow=False):
    sa, sb = S(P(*a_mm)), S(P(*b_mm))
    els.append('<line id="%s" class="%s" x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f"%s/>' % (id_, cls, sa[0], sa[1], sb[0], sb[1], ' marker-end="url(#ah)"' if arrow else ""))
    checks.append((id_, [tuple(a_mm), tuple(b_mm)], [sa, sb]))
    if label: els.append('<text x="%.1f" y="%.1f" class="lb">%s</text>' % (lp[0], lp[1], label))

def build():
    img = cv2.imread(str(PHOTO)); c = img[CROP[1]:CROP[3], CROP[0]:CROP[2]]
    c = cv2.resize(c, None, fx=SC, fy=SC, interpolation=cv2.INTER_CUBIC)
    ok, buf = cv2.imencode(".jpg", c, [cv2.IMWRITE_JPEG_QUALITY, 88]); b64 = base64.b64encode(buf.tobytes()).decode()
    W_, H_ = c.shape[1], c.shape[0]
    head = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="1030" viewBox="0 0 1000 1030">',
            '<style>.lb{font:13px sans-serif;fill:#fff;paint-order:stroke;stroke:#000;stroke-width:3px}.hd{font:bold 17px sans-serif;fill:#000}.nt{font:13px sans-serif;fill:#222}'
            '.tgt{fill:rgba(0,200,0,.25);stroke:#0c0;stroke-width:2.5;stroke-dasharray:6 4}.mk{fill:none;stroke:#fff;stroke-width:2}.edge{stroke:#f33;stroke-width:2.5}.axis{stroke:#f0f;stroke-width:1.5;stroke-dasharray:5 4}'
            '.dim{stroke:#0ff;stroke-width:2.5}.cur{fill:rgba(255,140,0,.15);stroke:#f80;stroke-width:2.5}.mv{stroke:#fff;stroke-width:4}</style>',
            '<defs><marker id="ah" markerWidth="8" markerHeight="8" refX="6" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 z" fill="#fff"/></marker></defs>',
            '<rect width="1000" height="1030" fill="#fff"/>',
            '<text x="20" y="26" class="hd">③ P1 블록 배치 안내 (Gemini 사진 2026-11-15 위, 승인 아님)</text>',
            '<text x="20" y="44" class="nt">위=베이스 쪽, 아래=사용자 쪽, 오른쪽=프로젝트 +x. 호모그래피(종이 평면, 보드 모서리 눈 짚기 ±2 px)로 그림. 높이 있는 물체 윗면은 시차로 어긋남.</text>',
            '<image x="%.0f" y="%.0f" width="%d" height="%d" href="data:image/jpeg;base64,%s"/>' % (OX, OY, W_, H_, b64)]
    # 목표 자리·기준선
    poly("target", [(-15, 160), (15, 160), (15, 220), (-15, 220)], "tgt")
    line("edge73", (-92, 73), (92, 73), "edge")
    line("axis", (0, 0), (0, 235), "axis")
    line("dim87", (45, 73), (45, 160), "dim", arrow=True)
    line("tick_a", (40, 73), (50, 73), "dim")
    line("tick_b", (40, 160), (50, 160), "dim")
    sx, sy = S(P(45, 116)); els.append('<text x="%.1f" y="%.1f" class="lb">87 mm (판 앞 가장자리 → 블록 앞면)</text>' % (sx + 8, sy))
    sx, sy = S(P(-92, 73)); els.append('<text x="%.1f" y="%.1f" class="lb">판 앞 가장자리 y=73 (실측)</text>' % (sx + 4, sy - 6))
    sx, sy = S(P(0, 0)); els.append('<text x="%.1f" y="%.1f" class="lb">J1 축 (0,0)</text>' % (sx + 8, sy + 4))
    sx, sy = S(P(15, 215)); els.append('<text x="%.1f" y="%.1f" class="lb">목표: 블록 x ±15, y 160~220 (중심 y 190)</text>' % (sx + 8, sy + 16))
    # 현재 블록(사진에서 검출한 윤곽, 윗면·앞면 포함) — 월드 환산은 참고
    cur = [(711.7, 266.6), (675.3, 255.4), (698.8, 178.8), (735.3, 190.0)]
    sp = [S(p) for p in cur]
    els.append('<polygon id="current" class="cur" points="%s"/>' % " ".join("%.2f,%.2f" % q for q in sp))
    cc = np.mean(cur, axis=0); tc = P(0, 190)
    a, b = S(cc), S(tc)
    els.append('<line id="move" class="mv" x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" marker-end="url(#ah)"/>' % (a[0], a[1], b[0], b[1]))
    els.append('<text x="%.1f" y="%.1f" class="lb">현재 블록(노란색, 약 15° 기울어짐)</text>' % (sp[1][0] - 40, sp[3][1] - 8))
    els.append('<text x="%.1f" y="%.1f" class="lb">이 방향으로 옮김(사용자 쪽 약 70 mm, 기울기 0°로)</text>' % (min(a[0], b[0]) + 40, (a[1] + b[1]) / 2))
    # 마커 종이보드(지금 종이 위): 안쪽 검은 정사각형 20 mm, 중심 (1.1,184.1) — 블록 윗면으로 옮길 것
    mk = [(-8.9, 174.5), (10.3, 174.6), (10.3, 193.7), (-8.9, 193.7)]
    sp2 = [S(P(*p)) for p in mk]
    els.append('<polygon id="marker_now" class="mk" points="%s"/>' % " ".join("%.2f,%.2f" % q for q in sp2))
    checks.append(("marker_now", [tuple(p) for p in mk], sp2))
    els.append('<text x="%.1f" y="%.1f" class="lb">마커 종이보드(지금 종이 위) → 블록 윗면 중앙에 붙임</text>' % (sp2[0][0] - 90, sp2[2][1] + 22))
    foot = ['<text x="20" y="1000" class="nt">할 일: ① 노란 블록을 초록 점선 자리(긴 변이 앞뒤 방향, 기울기 0°)로 옮기고 ② 20 mm 마커를 윗면 중앙에 붙임(마커 가로 = 블록 짧은 변과 평행). 붙인 뒤 알려 주세요.</text>',
            '<text x="20" y="1018" class="nt">그린 요소는 아래 검증표처럼 SVG 좌표를 호모그래피 역변환해 글의 mm와 대조했다(검증표는 답변에 첨부).</text>', '</svg>']
    return "\n".join(head + els + foot)

def verify(svg):
    rows = []
    for id_, mm, sp in checks:
        got = [tuple(inv(*q)) for q in sp]
        err = max(np.linalg.norm(np.array(g) - np.array(w)) for g, w in zip(got, mm))
        rows.append((id_, [tuple(round(v, 1) for v in w) for w in mm], [tuple(round(float(v), 1) for v in g) for g in got], round(float(err), 3)))
    return rows

if __name__ == "__main__":
    svg = build(); OUT.write_text(svg, encoding="utf-8")
    rows = verify(svg)
    print("요소 %d개 역산 검증, 최대 오차 %.3f mm" % (len(rows), max(r[3] for r in rows)))
    for r in rows: print("  %-10s 글 %s → 역산 %s 오차 %.3f mm" % r)
    cur = np.mean([(711.7, 266.6), (675.3, 255.4), (698.8, 178.8), (735.3, 190.0)], axis=0); w = inv(*S(cur))
    print("현재 블록 윤곽 중심(윗면·앞면 포함 종이 평면 환산) x %.1f, y %.1f mm → 목표 중심 (0, 190) 까지 이동 약 %.0f mm (높이 시차 포함, 참고)" % (w[0], w[1], 190 - w[1]))
