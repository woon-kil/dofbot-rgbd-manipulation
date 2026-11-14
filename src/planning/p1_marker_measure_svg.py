"""블록 윗면 마커의 실제 크기·위치를 재는 자리를 보여 주는 도식(SVG). 그림은 이상적(중앙) 배치이며 실제 값은 사용자가 잰다."""
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
K = 10.0; OX, OY = 320.0, 160.0           # px/mm, 블록 윗면 좌상단(왼쪽 = -x, 위 = 베이스 쪽)
WD, LN, M = 30.0, 60.0, 25.0
def X(x): return OX + K * (x + WD / 2)
def Y(y): return OY + K * (y + LN / 2)
checks = []
out = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="1000" viewBox="0 0 1000 1000">',
       '<style>.t{font:15px sans-serif;fill:#000}.hd{font:bold 18px sans-serif}.blk{fill:#ffe066;stroke:#a80;stroke-width:2}.mk{fill:#111}.w{fill:#fff}.d{stroke:#06c;stroke-width:2}.k{font:bold 20px sans-serif;fill:#c00}</style>',
       '<rect width="1000" height="1000" fill="#fff"/>',
       '<text x="20" y="30" class="hd">블록 윗면 마커 측정 안내 (도식, 사진 아님)</text>',
       '<text x="20" y="54" class="t">위에서 본 블록 윗면. 위 = 베이스 쪽, 아래 = 사용자 쪽, 왼쪽 = −x. 그림은 마커가 중앙에 있는 이상적인 모습이고 실제 값을 재 주세요. 단위 mm.</text>']
def rect(id_, x0, y0, x1, y1, cls):
    out.append('<rect id="%s" class="%s" x="%.2f" y="%.2f" width="%.2f" height="%.2f"/>' % (id_, cls, X(x0), Y(y0), K * (x1 - x0), K * (y1 - y0)))
    checks.append((id_, (x0, y0, x1, y1), (X(x0), Y(y0), X(x1), Y(y1))))
def dim(id_, x0, y0, x1, y1, label, lx, ly):
    out.append('<line id="%s" class="d" x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f"/><text x="%.1f" y="%.1f" class="k">%s</text>' % (id_, X(x0), Y(y0), X(x1), Y(y1), lx, ly, label))
    checks.append((id_, (x0, y0, x1, y1), (X(x0), Y(y0), X(x1), Y(y1))))
h = M / 2
rect("block_top", -15, -30, 15, 30, "blk"); rect("marker_black", -h, -h, h, h, "mk")
dim("a_left", -15, 0, -h, 0, "a", X(-15) + 3, Y(0) - 8)
dim("b_right", h, 0, 15, 0, "b", X(h) + 3, Y(0) - 8)
dim("c_top", 0, -30, 0, -h, "c", X(0) + 10, Y(-22))
dim("d_bottom", 0, h, 0, 30, "d", X(0) + 10, Y(22))
dim("e_side", -h, h + 3, h, h + 3, "e", X(0) - 5, Y(h + 3) + 22)
dim("f_side", h + 3, -h, h + 3, h, "f", X(h + 3) + 6, Y(0) + 6)
tx = 40
for i, ln in enumerate(["재 주세요 (마커의 검은 정사각형 가장자리 기준, 흰 여백이나 종이 띠 끝이 아니라):",
        "e = 검은 정사각형 가로 한 변,  f = 세로 한 변   (둘 다 25.0 mm에 가까워야 함, 다르면 알려 주세요)",
        "a = 검은 정사각형 왼쪽 변 ~ 블록 왼쪽 변,  b = 오른쪽 변 ~ 블록 오른쪽 변   (a + e + b = 30)",
        "c = 검은 정사각형 위(베이스 쪽) 변 ~ 블록 위쪽 짧은 변,  d = 아래(사용자 쪽) 변 ~ 블록 아래쪽 짧은 변   (c + f + d = 60)",
        "자(캘리퍼)로 0.5 mm 단위면 충분합니다. 값 6개(a b c d e f)를 알려 주세요.",
        "이 값이 마커 중심과 블록 중심의 차이(오프셋)가 되어 마커 위치로부터 블록 위치를 계산하는 데 쓰입니다 (±1 mm 목표)."]):
    out.append('<text x="%d" y="%d" class="t">%s</text>' % (tx, 840 + i * 24, ln))
out.append('</svg>')
svg = "\n".join(out); (ROOT / "results/g07_p1_marker_measure_v1.svg").write_text(svg, encoding="utf-8")
print("요소 %d개 역산 검증(도식 mm ↔ SVG 좌표)" % len(checks))
for id_, mm, sp in checks:
    got = ((sp[0] - OX) / K - WD / 2, (sp[1] - OY) / K - LN / 2, (sp[2] - OX) / K - WD / 2, (sp[3] - OY) / K - LN / 2)
    print("  %-12s 글 %s → 역산 %s 오차 %.3f" % (id_, tuple(round(v, 1) for v in mm), tuple(round(v, 1) for v in got), max(abs(a - b) for a, b in zip(got, mm))))
