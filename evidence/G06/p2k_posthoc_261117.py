#!/usr/bin/env python3
"""※ 사후 분석: 참고용, 결정 근거 아님(사전 등록 규칙이 아님). P2k 실행(261117-015111) 사진에 detect_v2와 같은 규칙에서 '성분 면적 ≥ 800 px'만 300 px로 낮춰 적용(사전 등록 v2 규칙은 캡 하나가 약 700 px라 미검출).
사용: .venv/bin/python -I evidence/G06/p2k_posthoc_261117.py"""
import sys, json
sys.path.insert(0,'evidence/G06'); sys.path.insert(0,'scripts'); sys.path.insert(0,'src')
import cv2, numpy as np, importlib.util
spec = importlib.util.spec_from_file_location('pa','evidence/G06/p2k_analyze_261117.py'); PA = importlib.util.module_from_spec(spec); spec.loader.exec_module(PA)
AREA = int(sys.argv[1]) if len(sys.argv) > 1 else 300
def caps(img, ref, pred, area):
    g = cv2.cvtColor(cv2.imread(img), cv2.COLOR_BGR2GRAY).astype(np.int16); r = cv2.cvtColor(cv2.imread(ref), cv2.COLOR_BGR2GRAY).astype(np.int16)
    H, W = g.shape; x0 = int(max(0, pred[0]-60)); x1 = int(min(W, pred[0]+60)); y0 = int(max(0, pred[1]-60)); y1 = int(min(H, pred[1]+60))
    m = (np.abs(g[y0:y1, x0:x1]-r[y0:y1, x0:x1]) > 45).astype(np.uint8); m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    nc, lab, st, _ = cv2.connectedComponentsWithStats(m, 8); keep = np.zeros_like(m); areas = []
    for i in range(1, nc):
        if st[i, cv2.CC_STAT_AREA] >= area: keep[lab == i] = 1; areas.append(int(st[i, cv2.CC_STAT_AREA]))
    ys, xs = np.nonzero(keep)
    if len(ys) == 0: return None, areas
    ymax = ys.max(); sel = ys >= ymax-14; yy = ys[sel]; xx = xs[sel].astype(float); c = np.array([xx.min(), xx.max()])
    for _ in range(20):
        a = np.abs(xx[:, None]-c[None, :]).argmin(1)
        if (a == 0).sum() == 0 or (a == 1).sum() == 0: break
        c = np.array([xx[a == 0].mean(), xx[a == 1].mean()])
    a = np.abs(xx[:, None]-c[None, :]).argmin(1)
    if abs(c[0]-c[1]) < 8 or (a == 0).sum() < 15 or (a == 1).sum() < 15: return None, areas
    pts = []
    for k in (0, 1):
        yk = yy[a == k]; xk = xx[a == k]; yb = yk.max(); pts.append((x0+float(xk[yk >= yb-4].mean()), y0+float(yb)))
    return sorted(pts), areas
res = json.load(open('evidence/G01/raw/g01_5b4_s4225_result-real-261117-015111.json')); pts = res['poses'][0]['j6_points']; ref = 'data/g04/p2kref_261117-015111.png'; out = []
print('※ 사후 분석: 참고용, 결정 근거 아님(면적 문턱 %d px로 낮춘 비사전등록 규칙)' % AREA)
for i, pt in enumerate(pts):
    j6 = 178 if pt['label'] in ('start', '4') else pt['j6_target']; q = [pt['raw_pre_arm'][k]['deg'] for k in '1234']
    pc = PA.caps_px(q, j6); pm = ((pc[0][0]+pc[1][0])/2, (pc[0][1]+pc[1][1])/2)
    cp, areas = caps('data/g04/d1_261117-015111_2501%d.png' % i, ref, pm, AREA)
    row = {'label': pt['label'], 'j6': j6, 'pred_mid': [round(float(pm[0]), 1), round(float(pm[1]), 1)], 'pred_gap': round(abs(pc[0][0]-pc[1][0]), 1), 'areas': areas}
    if cp: row['caps'] = [[round(x, 1) for x in c] for c in cp]; row['mid'] = [round((cp[0][0]+cp[1][0])/2, 1), round((cp[0][1]+cp[1][1])/2, 1)]; row['gap'] = round(abs(cp[0][0]-cp[1][0]), 1)
    out.append(row); print(row)
if out[0].get('mid'):
    v0 = out[0]['mid'][1]; p0 = out[0]['pred_mid'][1]
    for r in out[1:]:
        if r.get('mid'): print('점 %s J6 %d: Δv 관측 %+.1f px, 예측 %+.1f px, 가로 간격 관측 %.1f 예측 %.1f' % (r['label'], r['j6'], r['mid'][1]-v0, r['pred_mid'][1]-p0, r['gap'], r['pred_gap']))
json.dump(out, open('evidence/G06/p2k_posthoc_real_261117.json', 'w'), ensure_ascii=False, indent=1)
