#!/usr/bin/env python3
"""P2k 분석(촬영 전 사전 등록, 2026-11-17; Run 1 추가 2026-11-17: 블록 마커 id 7 검출과 d_x). 사용: .venv/bin/python -I evidence/G06/p2k_analyze_261117.py <결과.json> <사진 접두사 d1_<TS>_> <기준 사진 png> [출력.json]
규칙(결과를 보기 전 고정, 사후 조정 금지):
 - 사진 이름: <접두사><자세 번호 2501 또는 2504>N.png, N = j6_points 라벨(0=시작 178, 1=140, 2=100, 3=140 닫는 쪽, 4=178 끝).
 - 예측: 그 점의 원시 각도(raw_pre_arm J1~J4)와 J6 목표(178/140/100)로 메시 투영(gemini_j6_visibility_check_261117.py와 같은 모델, UNVERIFIED)한 두 캡 끝 화소의 중점.
 - 검출(**R-P2k2에서 사전 등록 변경 2026-11-17, 첫 실행 자료는 사용하지 않고 새 촬영만 판정**): detect_v2와 같은 규칙(차분 > 45, 3x3 열기 1회, 노출 방어 0.40, 맨 아래 14행 2-means 군집, 군집 간격 ≥ 8 px, 군집당 ≥ 15 화소)에서 **성분 면적 문턱만 800 → 300 px**, **ROI 반폭 ±60 → ±80 px**(세로 ±60 유지). 근거: 첫 실행 사진 5장의 ROI 성분 면적(캡 457~871 px, 비캡 303~354 px는 맨 아래 14행 규칙으로 제외), 문턱 200/300/400 사후 결과 불변, ROI 반폭 60/80 성분 목록 동일. 첫 실행 사진은 문턱·ROI 점검에만 사용(참고용).
 - **측정 가능성 판정:** 점 0(오므림 178)에서 두 캡이 검출되지 않으면(미검출·병합·노출) 이 실행 전체를 '판정 불가'로 둔다(규칙을 고치지 않는다).
 - 점이 '정지 중 J2~J4 처짐'(drift)으로 표시됐거나 J6 도착 실패면 그 점은 '판정 불가'.
 - **사진 유효성(D 시리즈 규칙):** 래퍼가 남기는 evidence/G06/d1_photo_<TS>_2501N.log의 photo_end_epoch − photo_start_epoch(촬영 길이, Mac 시계)가 그 점의 hold_s − 2(Jetson 정지 길이)를 넘으면 '판정 불가: 사진 시간'. Mac·Jetson 절대 시각은 비교하지 않고 길이만 비교한다.
 - **캡별 기록(도구 확장, 규칙 변경 아님):** detect_v2 중점 판정은 그대로 두고, 같은 규칙으로 두 캡 점과 가로 간격(예측 13/45/66 px)을 함께 기록해 J6↔grip 대응 가정을 확인한다.
 - 화소 Δv = v(점) − v(점 0). 예측 Δv는 같은 투영의 예측 중점 차(점 2: 약 −11.5 px).
 - **허용치(수정 2026-11-17, advisor):** 자세 A에서 `height_scale`(gemini_fingertip_detect_v2.height_scale)을 계산해 0.865 px/mm(앞뒤 국소 1.10 px/mm). 높이 허용 ±2.5 mm = ±2.16 px, 검출 흩어짐 상한 약 2 mm × 1.10 = 2.2 px를 제곱합 평균으로 합쳐 **TOL = 3.1 px**. 이전 "1.5 px/mm → 3.7 px"은 철회. 0 근처 구간 |Δv| < 3.1 px와 메시 일치 구간 [−14.6, −8.4] px는 겹치지 않는다(간격 5.3 px).
 - 판정 역할: 점 2(J6 100)만 가설을 가린다: |Δv| < TOL이면 '0 근처(J6 영향 없음)', 예측 ±TOL 이내면 '메시 예측과 일치', 그 외 '다른 값'. 점 1·3(140)은 값 보고용. 반복성 = 점 4 대 점 0의 중점 차(px).
 - **Run 1(블록을 둔 채 세트 28) 추가 사전 등록 — 옆 오프셋 d_x:** 각 점 사진에서 AprilTag 36h11 id 7(블록 윗면 마커, g01_e0g_analyze.detect)의 중심 픽셀을 찾아 블록 윗면 평면(높이 0.030 m, E-0G·r15의 P 규칙)으로 역투영해 프로젝트 좌표 x_block를 얻고, 두 캡 중점 픽셀을 그 점의 예측 캡 끝 높이 평면(r15 predict의 hh=(z+11)/1000 규칙)으로 역투영해 x_cap를 얻는다. **d_x = x_cap − x_block (mm, 프로젝트 x, 열림 축)**. **x_block는 기준 사진(블록 있음, 팔 K1)의 마커 id 7을 주 값으로 쓴다**(점 사진에서는 캡·그림자가 마커를 가려 검출이 실패할 수 있음: 합성 시험에서 점 2~4 미검출), 점 사진에서 검출되면 블록 이동 교차 확인으로 함께 기록. 기준 사진에서 미검출이면 d_x 전체 판정 불가. **J1 보정 제안은 점 2(J6 100, 접근 상태)의 d_x로 정하고(수정), 점 0·4(오므림)는 교차 확인·반복성**. 점 2에서 **캡별 옆 여유 m_left/m_right = |x_cap_i − x_block| − 15 − 5 (mm, 블록 반폭 15, 캡 반폭 추정 5; 열림 축, §6 간격 표의 옆 오프셋과 같은 정의에서 오프셋 = d_x)**를 각각 기록(두 캡의 x는 캡별 픽셀 역투영). 그림자 영향은 Run 1 점 0 결과로 판정하고 실패 시 규칙 수정 없이 판정 불가. **J1 보정 제안 dJ1 = −d_x / (dx/dJ1)**: dx/dJ1은 그 점 원시 각도에서 J1을 +1° 바꾼 FK 캡 중점 프로젝트 x의 차분(mm/°). 주 값은 J6 오므림 점 0과 점 4의 평균이며 점 1~3은 같은 규칙 보고용, 점 0 대 점 4의 d_x 차는 반복성(J6 176 대 176). 이 값은 Run 2 계획에서 J1 목표를 정하는 입력이지만 안전 한도는 아니다(Run 2에서도 원시 J1 관문과 정렬 허용 ≤ 3 mm 사진 확인).
 - **높이와 앞뒤는 한 대의 카메라로 분리되지 않는다**: 결과 보고에서 mm 환산값은 '캡이 메시 방향으로 움직인다는 가정 아래 환산'이라고 쓴다."""
import sys, json, os
sys.path.insert(0, 'evidence/G06'); sys.path.insert(0, 'scripts'); sys.path.insert(0, 'src')
import numpy as np, struct, re
import gemini_fingertip_detect_r15 as D
import gemini_fingertip_detect_v2 as D2
def _stl(p):
    b = open(p, 'rb').read()
    if b[:5] == b'solid' and b'facet' in b[:400]: return np.array(re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', b), dtype=float)
    n = struct.unpack('<I', b[80:84])[0]; a = np.frombuffer(b[84:84+n*50], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')])); return a['v'].reshape(-1, 3).astype(float)
M = {k: _stl('data/refs/dofbot_meshes/%s.STL' % k) * 1e3 for k in ('rlink1', 'rlink2', 'rlink3', 'llink1', 'llink2', 'llink3')}
TOL = 3.1   # px (위 허용치 규칙)
PA_BLOCK = ('--block' in sys.argv)   # Run 1(블록 있음): d_x 계산을 켠다. sys.argv에서 제거해 위치 인자 순서를 유지
sys.argv = [x for x in sys.argv if x != '--block']
INNER_GAP = {0: 5.0, 1: 24.3, 2: 44.3, 3: 30.7, 4: 6.7}   # 예측 가로 간격(보고용, 판정과 분리): P2g 안쪽 간격(mm) + 캡 폭 10 mm(추정)를 옆 스케일 1.37 px/mm로. 점 2는 대체값 48.5(G01.5a 캘리퍼·사용자)와 함께 보고
GRIP = {178: -1.5, 140: -1.1, 100: -0.79}   # J6 → grip_joint(rad), P2g 안쪽 간격 대응(UNVERIFIED)
def _Rz(a): c, s = np.cos(a), np.sin(a); R = np.eye(4); R[:2, :2] = [[c, -s], [s, c]]; return R
def _Tm(xyz, yaw, ang): m = _Rz(yaw) @ _Rz(ang); m[:3, 3] = np.array(xyz) * 1e3; return m
def caps_px(q, j6):
    g = GRIP[j6]; o = D.fk.fk(np.array([list(q) + [90.0]])); t = o['tcp'][0][0] * 1000; R = o['tcp'][1][0]
    r1 = _Tm([0.012625, 0.029275, 0.00475], 0.42401, -g); r2 = r1 @ _Tm([0.027229, -0.012593, -0.0025], 1.1468, g)
    l1 = _Tm([-0.012375, 0.029275, 0.0045], 2.7292, g); l2 = l1 @ _Tm([0.027362, 0.012302, -0.0025], -1.1584, -g)
    r3 = _Tm([0.0045, 0.047275, 0.0045], 0.42401, -g); l3 = _Tm([-0.0045, 0.047275, 0.0045], 2.7292, g)
    ch = {'rlink1': r1, 'rlink2': r2, 'rlink3': r3, 'llink1': l1, 'llink2': l2, 'llink3': l3}; out = []
    for s in 'rl':
        pts = np.vstack([(np.c_[M[k], np.ones(len(M[k]))] @ ch[k].T)[:, :3] for k in ch if k[0] == s]); p = pts[np.argmax(pts[:, 1])].copy(); p[1] += 4.0
        w = t + R @ np.array([p[0], p[1] - 90, 0]); out.append(np.array(D.P(-w[0], w[1], (w[2] + 11) / 1000)))
    return out
import cv2
import g01_e0g_analyze as E
def inv_P(u, v, h, x0=1.4, y0=189.0):
    """r15의 P(x, y, h)(프로젝트 좌표 mm → 픽셀)의 역: 뉴턴 반복(수치 야코비안)."""
    x, y = x0, y0
    for _ in range(30):
        f = np.array(D.P(x, y, h)) - np.array([u, v]); Jx = (np.array(D.P(x+0.5, y, h)) - np.array(D.P(x-0.5, y, h)))/1.0; Jy = (np.array(D.P(x, y+0.5, h)) - np.array(D.P(x, y-0.5, h)))/1.0
        dx = np.linalg.solve(np.c_[Jx, Jy], -f); x += dx[0]; y += dx[1]
        if np.hypot(*dx) < 1e-4: break
    return x, y
def cap_mid_xworld(q, j6):
    c = caps_world(q, j6); return c
def caps_world(q, j6):
    """캡 끝 두 점의 월드 평균(프로젝트 x = −FK x, y, 높이 z)"""
    g = GRIP[j6]; o = D.fk.fk(np.array([list(q) + [90.0]])); t = o['tcp'][0][0] * 1000; R = o['tcp'][1][0]
    r1 = _Tm([0.012625, 0.029275, 0.00475], 0.42401, -g); r2 = r1 @ _Tm([0.027229, -0.012593, -0.0025], 1.1468, g)
    l1 = _Tm([-0.012375, 0.029275, 0.0045], 2.7292, g); l2 = l1 @ _Tm([0.027362, 0.012302, -0.0025], -1.1584, -g)
    r3 = _Tm([0.0045, 0.047275, 0.0045], 0.42401, -g); l3 = _Tm([-0.0045, 0.047275, 0.0045], 2.7292, g)
    ch = {'rlink1': r1, 'rlink2': r2, 'rlink3': r3, 'llink1': l1, 'llink2': l2, 'llink3': l3}; ws = []
    for sd in 'rl':
        pts = np.vstack([(np.c_[M[k], np.ones(len(M[k]))] @ ch[k].T)[:, :3] for k in ch if k[0] == sd]); p = pts[np.argmax(pts[:, 1])].copy(); p[1] += 4.0
        ws.append(t + R @ np.array([p[0], p[1] - 90, 0]))
    w = (ws[0] + ws[1]) / 2; return -w[0], w[1], w[2]
def dx_mm(img_path, q, j6, cap_mid_px, ref_path=None):
    """d_x(mm) = x_cap − x_block (프로젝트 x). **x_block는 기준 사진(블록 있음, 팔 K1)의 마커 id 7 중심을 주 값으로 쓴다**(캡·그림자가 점 사진의 마커를 가릴 수 있음, 합성 시험에서 점 2~4 미검출), 점 사진에서 검출되면 블록 이동 교차 확인으로 함께 기록한다. 기준 사진에서 미검출이면 None."""
    m = E.detect(cv2.imread(ref_path)).get(7) if ref_path else None
    if m is None: return None
    mp = E.detect(cv2.imread(img_path)).get(7)
    xb, yb = inv_P(m['center'][0], m['center'][1], 0.030); xc0, yc0, zc = caps_world(q, j6)
    xc, yc = inv_P(cap_mid_px[0], cap_mid_px[1], (zc + 11) / 1000.0)
    return {'marker_point_photo_px': ([round(mp['center'][0], 1), round(mp['center'][1], 1)] if mp else None), 'marker_px': [round(m['center'][0], 1), round(m['center'][1], 1)], 'x_block': round(xb, 2), 'y_block': round(yb, 2), 'x_cap': round(xc, 2), 'y_cap': round(yc, 2), 'd_x_mm': round(xc - xb, 2)}
def dx_per_J1(q, j6):
    q1 = list(q); q1[0] += 1.0; return caps_world(q1, j6)[0] - caps_world(q, j6)[0]
def detect_caps(img_path, ref_path, pred, area=300, half=80):
    """detect_v2와 같은 규칙으로 두 캡 아래끝 점을 돌려준다(도구 확장: 가로 간격 기록용). 실패하면 None."""
    g = cv2.cvtColor(cv2.imread(img_path), cv2.COLOR_BGR2GRAY).astype(np.int16); r = cv2.cvtColor(cv2.imread(ref_path), cv2.COLOR_BGR2GRAY).astype(np.int16)
    H, W = g.shape; x0 = int(max(0, pred[0]-half)); x1 = int(min(W, pred[0]+half)); y0 = int(max(0, pred[1]-60)); y1 = int(min(H, pred[1]+60))
    m = (np.abs(g[y0:y1, x0:x1]-r[y0:y1, x0:x1]) > 45).astype(np.uint8)
    if m.mean() > 0.40: return None
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)); nc, lab, st, _ = cv2.connectedComponentsWithStats(m, 8); keep = np.zeros_like(m)
    for i in range(1, nc):
        if st[i, cv2.CC_STAT_AREA] >= area: keep[lab == i] = 1
    ys, xs = np.nonzero(keep)
    if len(ys) == 0: return None
    ymax = ys.max(); sel = ys >= ymax-14; yy = ys[sel]; xx = xs[sel].astype(float); c = np.array([xx.min(), xx.max()])
    for _ in range(20):
        a = np.abs(xx[:, None]-c[None, :]).argmin(1)
        if (a == 0).sum() == 0 or (a == 1).sum() == 0: break
        c = np.array([xx[a == 0].mean(), xx[a == 1].mean()])
    a = np.abs(xx[:, None]-c[None, :]).argmin(1)
    if abs(c[0]-c[1]) < 8 or (a == 0).sum() < 15 or (a == 1).sum() < 15: return None
    pts = []
    for k in (0, 1):
        yk = yy[a == k]; xk = xx[a == k]; yb = yk.max(); pts.append((x0+float(xk[yk >= yb-4].mean()), y0+float(yb)))
    return pts
def photo_len(logpath):
    try:
        t = open(logpath).read(); a = int(re.search(r'photo_start_epoch=(\d+)', t).group(1)); b = int(re.search(r'photo_end_epoch=(\d+)', t).group(1)); return b-a
    except Exception: return None
if __name__ == '__main__':
    res = json.load(open(sys.argv[1])); pre = sys.argv[2]; ref = sys.argv[3]; rows = []
    rawq_by_label = {}
    pr = res['poses'][0]; pts = pr['j6_points']; drift = {d['point'] for d in pr.get('drift', [])}
    for pt in pts:
        lab = pt['label']; j6 = 178 if lab in ('start', '4') else pt['j6_target']; rawq = [pt['raw_pre_arm'][k]['deg'] for k in '1234']
        if any(v is None for v in rawq): rows.append({'label': lab, 'status': '판정 불가: 원시 각도 None'}); continue
        rawq_by_label[lab] = rawq; c = caps_px(rawq, j6); mid_pred = ((c[0][0] + c[1][0]) / 2, (c[0][1] + c[1][1]) / 2); idx = 0 if lab == 'start' else int(lab); PID = pr['pose_id']; img = '%s%d%d.png' % (pre, PID, idx)
        row = {'label': lab, 'j6': j6, 'pred_mid_px': [round(float(mid_pred[0]), 1), round(float(mid_pred[1]), 1)], 'pred_caps_px': [[round(float(x), 1) for x in cc] for cc in c]}
        if lab in drift: row['status'] = '판정 불가: 정지 중 처짐'
        elif not os.path.exists(img): row['status'] = '사진 없음'
        else:
            ts = pre[len('data/g04/d1_'):].rstrip('_') if pre.startswith('data/g04/d1_') else None; ln = photo_len('evidence/G06/d1_photo_%s_%d%d.log' % (ts, PID, idx)) if ts else None; row['photo_len_s'] = ln; row['hold_s'] = pt.get('hold_s')
            if ln is None or pt.get('hold_s') is None or ln > pt['hold_s'] - 2: rows.append({**row, 'status': '판정 불가: 사진 시간(촬영 %s s > 정지 %s s − 2 또는 기록 없음)' % (ln, pt.get('hold_s'))}); print(rows[-1]); continue
            cp = detect_caps(img, ref, mid_pred); mid = ((cp[0][0]+cp[1][0])/2, (cp[0][1]+cp[1][1])/2) if cp else None; row['status'] = 'ok' if cp else '미검출 또는 병합: 제외'
            if cp: row['caps_px'] = [[round(x, 1) for x in q_] for q_ in sorted(cp)]; row['cap_gap_px'] = round(abs(cp[0][0] - cp[1][0]), 1); row['pred_cap_gap_px'] = round(INNER_GAP[idx] * 1.37 + 10 * 1.37, 1); row['pred_cap_gap_mesh_tip_px'] = round(abs(c[0][0] - c[1][0]), 1)
            if cp is not None and PA_BLOCK:
                dxr = dx_mm(img, rawq, j6, mid, ref); row['dx'] = dxr
                if dxr: row['dJ1_suggest_deg'] = round(-dxr['d_x_mm'] / dx_per_J1(rawq, j6), 2); row['dJ1_primary'] = (lab == '2')
            if mid is not None: row['mid_px'] = [round(mid[0], 1), round(mid[1], 1)]; row['diff_px'] = [round(mid[0] - mid_pred[0], 1), round(mid[1] - mid_pred[1], 1)]
        rows.append(row); print(row)
    for r in rows:
        if r.get('label') == '2' and r.get('dx') and r.get('caps_px'):
            xbk = r['dx']['x_block']; hc = (r['dx']['x_cap'] if False else None)
            zc = caps_world(rawq_by_label.get('2'), 100)[2] if rawq_by_label.get('2') else None
            if zc is not None:
                mm = []
                for cpx in r['caps_px']:
                    xi, yi = inv_P(cpx[0], cpx[1], (zc + 11) / 1000.0); mm.append(round(abs(xi - xbk) - 15 - 5, 2))
                r['cap_side_margin_mm'] = mm   # [왼쪽 캡(u 작은 쪽), 오른쪽 캡]
    r0 = rows[0] if rows else None
    if r0 is None or 'mid_px' not in r0: print('판정 불가: 점 0(오므림 178)에서 두 캡이 검출되지 않았다(규칙 수정 없음).'); verdict = {'run': '판정 불가'}
    else:
        v0 = r0['mid_px'][1]; verdict = {'run': 'ok'}
        for r in rows[1:]:
            if 'mid_px' in r: r['dv_obs_px'] = round(r['mid_px'][1] - v0, 1); r['dv_pred_px'] = round(r['pred_mid_px'][1] - r0['pred_mid_px'][1], 1)
        r4 = next((r for r in rows if r['label'] == '4' and 'mid_px' in r), None)
        if r4: verdict['repeatability_px'] = [round(r4['mid_px'][0] - r0['mid_px'][0], 1), round(r4['mid_px'][1] - r0['mid_px'][1], 1)]
        r2 = next((r for r in rows if r['label'] == '2' and 'dv_obs_px' in r), None)
        if r2: verdict['point2'] = '0 근처(J6 영향 없음)' if abs(r2['dv_obs_px']) < TOL else ('메시 예측과 일치' if abs(r2['dv_obs_px'] - r2['dv_pred_px']) <= TOL else '다른 값')
    print(verdict)
    if len(sys.argv) > 4: json.dump({'rows': rows, 'verdict': verdict}, open(sys.argv[4], 'w'), ensure_ascii=False, indent=1)
