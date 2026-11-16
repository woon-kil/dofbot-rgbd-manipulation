#!/usr/bin/env python3
"""P2k 분석(촬영 전 사전 등록, 2026-11-17). 사용: .venv/bin/python -I evidence/G06/p2k_analyze_261117.py <결과.json> <사진 접두사 d1_<TS>_> <기준 사진 png> [출력.json]
규칙(결과를 보기 전 고정, 사후 조정 금지):
 - 사진 이름: <접두사>2501N.png, N = j6_points 라벨(0=시작 178, 1=140, 2=100, 3=140 닫는 쪽, 4=178 끝).
 - 예측: 그 점의 원시 각도(raw_pre_arm J1~J4)와 J6 목표(178/140/100)로 메시 투영(gemini_j6_visibility_check_261117.py와 같은 모델, UNVERIFIED)한 두 캡 끝 화소의 중점.
 - 검출: gemini_fingertip_detect_v2.py의 detect_v2(차분 > 45, 노출 방어 0.40, 2-means 군집 간격 ≥ 8 px), ROI = 예측 중점 ±60 px. 규칙 수정 없음.
 - **측정 가능성 판정:** 점 0(오므림 178)에서 두 캡이 검출되지 않으면(미검출·병합·노출) 이 실행 전체를 '판정 불가'로 둔다(규칙을 고치지 않는다).
 - 점이 '정지 중 J2~J4 처짐'(drift)으로 표시됐거나 J6 도착 실패면 그 점은 '판정 불가'.
 - 화소 Δv = v(점) − v(점 0). 예측 Δv는 같은 투영의 예측 중점 차. 허용 ±3.7 px(= 높이 ±2.5 mm × 약 1.5 px/mm, 근사; 정확한 px/mm는 height_scale로 보고에 함께 적는다).
 - 판정 역할: 점 2(J6 100)만 가설을 가린다: |Δv| < 3.7이면 '0 근처(J6 영향 없음)', 예측 ±3.7 이내면 '메시 예측과 일치', 그 외 '다른 값'. 점 1·3(140)은 값 보고용. 반복성 = 점 4 대 점 0의 중점 차(px).
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
if __name__ == '__main__':
    res = json.load(open(sys.argv[1])); pre = sys.argv[2]; ref = sys.argv[3]; rows = []
    pr = res['poses'][0]; pts = pr['j6_points']; drift = {d['point'] for d in pr.get('drift', [])}
    for pt in pts:
        lab = pt['label']; j6 = 178 if lab in ('start', '4') else pt['j6_target']; rawq = [pt['raw_pre_arm'][k]['deg'] for k in '1234']
        if any(v is None for v in rawq): rows.append({'label': lab, 'status': '판정 불가: 원시 각도 None'}); continue
        c = caps_px(rawq, j6); mid_pred = ((c[0][0] + c[1][0]) / 2, (c[0][1] + c[1][1]) / 2); idx = 0 if lab == 'start' else int(lab); img = '%s2501%d.png' % (pre, idx)
        row = {'label': lab, 'j6': j6, 'pred_mid_px': [round(float(mid_pred[0]), 1), round(float(mid_pred[1]), 1)], 'pred_caps_px': [[round(float(x), 1) for x in cc] for cc in c]}
        if lab in drift: row['status'] = '판정 불가: 정지 중 처짐'
        elif not os.path.exists(img): row['status'] = '사진 없음'
        else:
            mid, why = D2.detect_v2(img, ref, mid_pred); row['status'] = why
            if mid is not None: row['mid_px'] = [round(mid[0], 1), round(mid[1], 1)]; row['diff_px'] = [round(mid[0] - mid_pred[0], 1), round(mid[1] - mid_pred[1], 1)]
        rows.append(row); print(row)
    r0 = rows[0] if rows else None
    if r0 is None or 'mid_px' not in r0: print('판정 불가: 점 0(오므림 178)에서 두 캡이 검출되지 않았다(규칙 수정 없음).'); verdict = {'run': '판정 불가'}
    else:
        v0 = r0['mid_px'][1]; verdict = {'run': 'ok'}
        for r in rows[1:]:
            if 'mid_px' in r: r['dv_obs_px'] = round(r['mid_px'][1] - v0, 1); r['dv_pred_px'] = round(r['pred_mid_px'][1] - r0['pred_mid_px'][1], 1)
        r4 = next((r for r in rows if r['label'] == '4' and 'mid_px' in r), None)
        if r4: verdict['repeatability_px'] = [round(r4['mid_px'][0] - r0['mid_px'][0], 1), round(r4['mid_px'][1] - r0['mid_px'][1], 1)]
        r2 = next((r for r in rows if r['label'] == '2' and 'dv_obs_px' in r), None)
        if r2: verdict['point2'] = '0 근처(J6 영향 없음)' if abs(r2['dv_obs_px']) < 3.7 else ('메시 예측과 일치' if abs(r2['dv_obs_px'] - r2['dv_pred_px']) <= 3.7 else '다른 값')
    print(verdict)
    if len(sys.argv) > 4: json.dump({'rows': rows, 'verdict': verdict}, open(sys.argv[4], 'w'), ensure_ascii=False, indent=1)
