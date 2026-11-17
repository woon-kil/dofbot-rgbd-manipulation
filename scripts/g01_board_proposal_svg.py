#!/usr/bin/env python3
"""보정 보드 배치 제안을 Gemini 정합 컬러 사진 위에 그린 SVG(사진 포함, gitignore 폴더에 저장). 이론값·제안이다."""
import base64, math
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
fx, fy, cx, cy = 610.51, 610.35, 641.79, 364.44
d = np.load(ROOT/'evidence/G01/raw/gemini/registered/depth_mm.npy').astype(float)/1000.0
x0, x1, y0, y1 = 550, 830, 190, 395
vv, uu = np.mgrid[y0:y1, x0:x1]; dd = d[y0:y1, x0:x1]; ok = (dd > 0) & (dd < 3)
P = np.stack([(uu[ok]-cx)*dd[ok]/fx, (vv[ok]-cy)*dd[ok]/fy, dd[ok]], 1); Ps = P[np.random.default_rng(0).choice(len(P), 5000, replace=False)]
c0 = Ps.mean(0); _, _, vt = np.linalg.svd(Ps-c0, full_matrices=False); n = vt[2]; n = -n if n[2] > 0 else n
hit = lambda u, v: (lambda r: r*((c0@n)/(r@n)))(np.array([(u-cx)/fx, (v-cy)/fy, 1.0]))
proj = lambda p: (fx*p[0]/p[2]+cx, fy*p[1]/p[2]+cy)
base = hit(690, 175)
fwd = hit(690, 300)-base; fwd -= (fwd@n)*n; fwd /= np.linalg.norm(fwd)     # 베이스에서 사용자(카메라) 쪽
lat = np.cross(n, fwd); lat /= np.linalg.norm(lat)
def rect(r_c, w, h):   # 베이스에서 r_c m 앞, 좌우 폭 w, 반경 방향 길이 h (m)
    c = base + r_c*fwd
    return [proj(c + sx*w/2*lat + sy*h/2*fwd) for sx, sy in ((-1,-1),(1,-1),(1,1),(-1,1))]
import sys
R_C, W, H = (float(sys.argv[1])/1000, float(sys.argv[2])/1000, float(sys.argv[3])/1000) if len(sys.argv) > 3 else (0.165, 0.120, 0.080)
board = rect(R_C, W, H)
b64 = base64.b64encode(open(ROOT/'evidence/G01/raw/gemini/registered/color.png', 'rb').read()).decode()
pts = lambda q: " ".join("%.0f,%.0f" % p for p in q)
svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="1280" viewBox="0 0 1280 1280" font-family="Helvetica, Arial, sans-serif">','<rect width="1280" height="1280" fill="#fff"/>',f'<image x="0" y="0" width="1280" height="720" href="data:image/png;base64,{b64}"/>',
 '<rect x="550" y="190" width="280" height="205" fill="none" stroke="#fff" stroke-width="2"/>',
 f'<polygon points="{pts(board)}" fill="#12b886" fill-opacity="0.25" stroke="#12b886" stroke-width="4"/>',
 '<circle cx="%.0f" cy="%.0f" r="6" fill="#e03131" stroke="#fff"/>' % proj(base),
 '<rect x="672" y="276" width="40" height="30" fill="none" stroke="#f08c00" stroke-width="3"/>','<rect x="845" y="578" width="140" height="137" fill="none" stroke="#e03131" stroke-width="3"/>',
 '<rect x="0" y="720" width="1280" height="560" fill="#f1f3f5"/>']
lines = [("보정 보드 배치 제안 (초록, 이론값). 흰 사각형 = 확정 작업영역 마스크 v1, 빨간 점 = 베이스 중심 추정(±16 mm)", "#212529"),
 ("제안 위치(초록): 보드 %.0f x %.0f mm(좌우 x 앞뒤), 베이스에서 앞쪽 약 %.0f mm 가운데. 작업 용지 앞쪽 가장자리 부근입니다" % (W*1000, H*1000, R_C*1000), "#212529"),
 ("주황 = 기존 AprilTag(약 18.4 mm, 베이스에서 약 125 mm) / 빨강 = 기존 ArUco 4x4 id0(약 58.3 mm, 베이스에서 약 405 mm)", "#212529"),
 ("DaBai는 그리퍼와 같은 방향을 봐서 작업대를 내려다볼 때 거리가 약 0.15~0.25 m입니다.", "#c92a2a"),
 ("보드가 크면 시야에 다 들어오는 자세가 적어 크기를 줄이고 더 앞쪽에 둡니다.", "#c92a2a"),
 ("사용자 확인: 이 위치(초록)에 보드를 두어도 되는지, 용지 위에 고정 가능한지 알려 주세요. 보드를 만든 뒤 사진 위 SVG로 재확인합니다.", "#212529")]
y = 770
for tx, col in lines: svg.append('<text x="16" y="%d" font-size="21" fill="%s">%s</text>' % (y, col, tx)); y += 42
svg.append('</svg>')
out = ROOT/'evidence/G01/raw/gemini/registered/board-proposal-v2.svg'; out.write_text("\n".join(svg), encoding='utf-8'); print("저장:", out.name, "| 보드 모서리 픽셀:", [(round(a), round(b)) for a, b in board])
