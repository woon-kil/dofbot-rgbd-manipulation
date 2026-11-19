#!/usr/bin/env python3
"""E-5 복원 안내 SVG 생성 + 그림 요소 역산 대조(읽기 전용, 하드웨어 없음). 양식: data/g04/restore_261118-2315.svg(확대 4배 잘라낸 사진 위에 현재·목표 마커 사각형).
사용: .venv/bin/python -I evidence/G01/analysis/e5_restore_svg.py <사진.png> <출력.svg>"""
import sys, importlib.util, base64, re, cv2, numpy as np
sys.path.insert(0,'evidence/G06'); sys.path.insert(0,'scripts'); sys.path.insert(0,'src')
spec=importlib.util.spec_from_file_location('pa','evidence/G06/p2k_analyze_261117.py'); PA=importlib.util.module_from_spec(spec); spec.loader.exec_module(PA)
img_path, out = sys.argv[1], sys.argv[2]; img = cv2.imread(img_path); m = PA.E.detect(img).get(7); assert m, "마커 id 7 미검출"
REF = (673.25, 267.25); Z = 4; CW, CH = 260, 200; X0, Y0 = 542, 163
cur = tuple(m['center']); side = m['side_px']
crop = cv2.resize(img[Y0:Y0+CH, X0:X0+CW], None, fx=Z, fy=Z, interpolation=cv2.INTER_CUBIC); ok, jpg = cv2.imencode('.jpg', crop, [cv2.IMWRITE_JPEG_QUALITY, 88]); b64 = base64.b64encode(jpg.tobytes()).decode()
def D(p): return ((p[0]-X0)*Z, (p[1]-Y0)*Z)
def box(c): h = side*Z/2; x, y = D(c); return "%.1f,%.1f %.1f,%.1f %.1f,%.1f %.1f,%.1f" % (x-h,y-h,x+h,y-h,x+h,y+h,x-h,y+h)
xb0, yb0 = PA.inv_P(REF[0], REF[1], 0.030); xb1, yb1 = PA.inv_P(cur[0], cur[1], 0.030); mx, my = xb0-xb1, yb0-yb1
px_r = PA.inv_P(REF[0]+1, REF[1], 0.030)[0]-xb0; px_d = PA.inv_P(REF[0], REF[1]+1, 0.030)[1]-yb0
cx, cy = D(cur); tx, ty = D(REF)
LR = '오른쪽' if mx >= 0 else '왼쪽'; UD = '사용자 쪽(아래)' if my >= 0 else '베이스 쪽(위)'
svg = f'''<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" width="1040" height="1060" viewBox="0 0 1040 1060">
<rect width="1040" height="1060" fill="#fff"/>
<image id="photo" x="0" y="0" width="1040" height="800" xlink:href="data:image/jpeg;base64,{b64}"/>
<polygon id="now" points="{box(cur)}" fill="none" stroke="#e00" stroke-width="3"/>
<polygon id="target" points="{box(REF)}" fill="none" stroke="#0a0" stroke-width="3" stroke-dasharray="10 6"/>
<line id="arrow" x1="{cx:.1f}" y1="{cy:.1f}" x2="{tx:.1f}" y2="{ty:.1f}" stroke="#06f" stroke-width="4"/>
<circle id="tip" cx="{tx:.1f}" cy="{ty:.1f}" r="5" fill="#06f"/>
<line id="scale" x1="20" y1="780" x2="{20+10/abs(px_r)*Z:.1f}" y2="780" stroke="#000" stroke-width="5"/>
<text x="{30+10/abs(px_r)*Z:.1f}" y="786" font-size="22" fill="#000">= 10 mm(가로)</text>
<text x="20" y="28" font-size="22" fill="#e00" font-weight="bold">빨강 실선: 지금 블록 윗면 마커 / 초록 점선: 놓을 자리(등록 기준 {REF[0]}, {REF[1]})</text>
<text x="20" y="830" font-size="24" fill="#000">사진 방향: 위 = 베이스 쪽, 아래 = 사용자 쪽, 오른쪽 = 사용자 오른쪽(G07-D0 승인 요청 문서 90행 기준, 추론 기록 포함)</text>
<text x="20" y="862" font-size="24" fill="#06f" font-weight="bold">블록을 {LR}으로 약 {abs(mx):.1f} mm, {UD}으로 약 {abs(my):.1f} mm 옮긴다</text>
<text x="20" y="892" font-size="24" fill="#06f" font-weight="bold">(직선 {np.hypot(mx,my):.1f} mm. 1 mm 안팎이면 손으로 살짝 미는 정도)</text>
<text x="20" y="926" font-size="18" fill="#333">환산: 높이 30 mm 평면 역투영(inv_P), 1 px 오른쪽 = {px_r:+.3f} mm, 1 px 아래 = {px_d:+.3f} mm.</text>
<text x="20" y="944" font-size="16" fill="#555">사진: {img_path}</text>
<text x="20" y="966" font-size="18" fill="#333">참고값이며 판정은 놓은 뒤 세션이 찍는 사진의 마커 재측정으로 한다(기준에 최대한 가깝게, 1.0 px는 상한).</text>
<text x="20" y="990" font-size="18" fill="#333">위치만 옮긴다(돌리지 않는다). 종이·팔은 건드리지 않는다.</text>
<text x="20" y="1014" font-size="18" fill="#333">블록에 붙은 검은 표식은 블록과 함께 움직인다. 옮기는 동안 팔은 정지·기립 상태.</text>
</svg>'''
open(out, 'w', encoding='utf-8').write(svg)
# 역산 대조: SVG 좌표 → 원본 사진 px → 월드 mm
t = open(out, encoding='utf-8').read()
def pts(id_): s = re.search(r'id="%s" points="([^"]+)"' % id_, t).group(1); return np.array([[float(v) for v in p.split(',')] for p in s.split()])
def center_px(id_): p = pts(id_).mean(0); return (p[0]/Z+X0, p[1]/Z+Y0)
a = re.search(r'id="arrow" x1="([\d.]+)" y1="([\d.]+)" x2="([\d.]+)" y2="([\d.]+)"', t); a = [float(v) for v in a.groups()]
rows = []
for name, c in (('now 사각형 중심', center_px('now')), ('target 사각형 중심', center_px('target')), ('화살표 시작', (a[0]/Z+X0, a[1]/Z+Y0)), ('화살표 끝', (a[2]/Z+X0, a[3]/Z+Y0))):
    x, y = PA.inv_P(c[0], c[1], 0.030); rows.append((name, c, x, y))
print('요소 | 사진 px | 월드 mm(x, y)'); [print('%s | (%.2f, %.2f) | (%.2f, %.2f)' % (r[0], r[1][0], r[1][1], r[2], r[3])) for r in rows]
print('글의 값: 이동 오른쪽 %+.2f mm, 아래 %+.2f mm' % (mx, my))
print('역산: 화살표(끝−시작) 오른쪽 %+.2f mm, 아래 %+.2f mm; target−now 오른쪽 %+.2f, 아래 %+.2f' % (rows[3][2]-rows[2][2], rows[3][3]-rows[2][3], rows[1][2]-rows[0][2], rows[1][3]-rows[0][3]))
print('측정 마커 (%.2f, %.2f) px, 기준 (%.2f, %.2f), 차 (%.2f, %.2f) px, 사각형 변 %.1f px, 확대 %d배' % (cur[0], cur[1], REF[0], REF[1], REF[0]-cur[0], REF[1]-cur[1], side, Z))
print('crop 안: now 중심 (%.0f, %.0f) target (%.0f, %.0f) / 크기 %dx%d' % (cx, cy, tx, ty, CW*Z, CH*Z))
