#!/usr/bin/env python3
"""도달 반경 원을 작업대면에 놓고 Gemini 정합 컬러 위에 투영한 SVG와 시야 포함 비율을 만든다(이론값, 오프라인)."""
import base64, json, math
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
fx, fy, cx, cy = 610.51, 610.35, 641.79, 364.44
d = np.load(ROOT/'evidence/G01/raw/gemini/registered/depth_mm.npy').astype(float)/1000.0
x0, x1, y0, y1 = 550, 830, 190, 395
vv, uu = np.mgrid[y0:y1, x0:x1]; dd = d[y0:y1, x0:x1]; ok = (dd > 0) & (dd < 3)
P = np.stack([(uu[ok]-cx)*dd[ok]/fx, (vv[ok]-cy)*dd[ok]/fy, dd[ok]], 1)
Ps = P[np.random.default_rng(0).choice(len(P), 5000, replace=False)]
c0 = Ps.mean(0); _, _, vt = np.linalg.svd(Ps-c0, full_matrices=False); n = vt[2]
if n[2] > 0: n = -n
def hit(u, v):
    ray = np.array([(u-cx)/fx, (v-cy)/fy, 1.0]); return ray*((c0@n)/(ray@n))
def proj(p): return (fx*p[0]/p[2]+cx, fy*p[1]/p[2]+cy) if p[2] > 0 else None
base = hit(690, 175)
ex = hit(830, 190)-hit(550, 190); ex /= np.linalg.norm(ex); ey = np.cross(n, ex)
def circle(r, base=base, k=360):
    pts = []
    for i in range(k):
        a = 2*math.pi*i/k; p = base + r*(math.cos(a)*ex + math.sin(a)*ey); pts.append(proj(p))
    return pts
def inside(pt): return pt is not None and 0 <= pt[0] < 1280 and 0 <= pt[1] < 720
radii = {"r 125 (마스크 가까운 변)": 0.125, "r 203 (접근 기울기 ≤15°, 높이 10~30 mm)": 0.203, "r 248 (≤30°)": 0.248, "r 286 (≤45°)": 0.286, "r 316 (제약 없음, 높이 10~30 mm)": 0.316}
res = {}
for name, r in radii.items():
    pts = circle(r); vis = sum(inside(p) for p in pts)/len(pts)
    # 기준 베이스 픽셀 ±20 px 변동에서의 시야 포함 비율 범위
    fr = []
    for du in (-20, 0, 20):
        for dv in (-20, 0, 20):
            b = hit(690+du, 175+dv); pp = circle(r, b); fr.append(sum(inside(p) for p in pp)/len(pp))
    res[name] = {"visible_fraction": round(vis, 2), "range_over_base_uncertainty": [round(min(fr), 2), round(max(fr), 2)]}
    print(name, "시야 안 비율 %.2f (베이스 ±20px: %.2f~%.2f)" % (vis, min(fr), max(fr)))
json.dump(res, open(ROOT/'evidence/G01/reach_visibility_estimate.json', 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
b64 = base64.b64encode(open(ROOT/'evidence/G01/raw/gemini/registered/color.png', 'rb').read()).decode()
cols = ["#868e96", "#12b886", "#f08c00", "#e03131", "#7048e8"]
svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="1280" viewBox="0 0 1280 1280" font-family="Helvetica, Arial, sans-serif">', '<rect width="1280" height="1280" fill="#fff"/>', f'<image x="0" y="0" width="1280" height="720" href="data:image/png;base64,{b64}"/>', '<defs><clipPath id="c"><rect x="0" y="0" width="1280" height="720"/></clipPath></defs>', '<g clip-path="url(#c)" fill="none" stroke-width="3">']
for (name, r), col in zip(radii.items(), cols):
    pts = [p for p in circle(r) if p is not None]
    svg.append('<polyline stroke="%s" points="%s" stroke-dasharray="%s"/>' % (col, " ".join("%.0f,%.0f" % p for p in pts), "10 6" if r > 0.2 else "none"))
svg.append('<rect x="550" y="190" width="280" height="205" stroke="#fff" stroke-width="2"/>')
bp = proj(base); svg.append('<circle cx="%.0f" cy="%.0f" r="7" fill="#e03131" stroke="#fff"/>' % bp); svg.append('</g>')
svg.append('<rect x="0" y="720" width="1280" height="560" fill="#f1f3f5"/>')
t = ["도달 반경 이론값(URDF, 작업대면 가정). 중심 빨간 점 = 베이스 중심 추정(영상에서 눈으로 잡음, ±20px)", "흰 사각형 = 확정 작업영역 마스크 v1 (가로 248 mm, 세로 198 mm)"]
for (name, r), col in zip(radii.items(), cols): t.append("■ %s" % name); 
svg.append('<text x="16" y="760" font-size="22" fill="#212529">%s</text><text x="16" y="792" font-size="22" fill="#212529">%s</text>' % (t[0], t[1]))
y = 836
for (name, r), col in zip(radii.items(), cols):
    svg.append('<text x="16" y="%d" font-size="24" fill="%s" stroke="#000" stroke-width="0.3">■ %s  (영상 안 비율 %.2f)</text>' % (y, col, name, res[name]["visible_fraction"])); y += 36
svg.append('<text x="16" y="%d" font-size="22" fill="#c92a2a">이론값입니다: 충돌·베이스 판 두께·실제 서보 한계·작업대면 높이(R2)를 반영하지 않았습니다. 영상 밖으로 나간 부분은 그려지지 않습니다.</text>' % (y+30))
svg.append('<text x="16" y="%d" font-size="22" fill="#c92a2a">베이스 중심과 작업대면 모델이 추정이라 원의 위치는 약 ±20 mm 오차가 있습니다(Gemini 베이스 보정 G03 전).</text>' % (y+62))
svg.append('</svg>')
out = ROOT/'evidence/G01/raw/gemini/registered/reach-overlay.svg'; out.write_text("\n".join(svg), encoding='utf-8'); print("저장:", out.name)
