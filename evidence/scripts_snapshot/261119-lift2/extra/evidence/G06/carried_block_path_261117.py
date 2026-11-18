#!/usr/bin/env python3
"""쥔 블록 들어올림 경로 점검(오프라인, 하드웨어 없음, 값은 UNVERIFIED FK). 블록을 쥔 채 관절 이동 순서를 따라가며 **끝점뿐 아니라 모든 중간 상태**의
캡 끝 변위(Δz, Δy), tilt, 블록 바닥 종이 위 높이를 낸다. 블록은 캡 끝과 같이 움직인다고 본다(강체). 서보 오차(증가 방향 부족분, 감소 방향 넘침)를 넣을 수 있다.
규칙(제안, 승인 전): ① 첫 이동은 높이를 올려야 한다(Δz > 0). ② 모든 중간 상태에서 블록 바닥 높이 ≥ gate_mm(블록이 종이에 눌리지 않음). 위반이면 FAIL.
종이 위 높이 = FK z + 11 mm. 사용: .venv/bin/python -I evidence/G06/carried_block_path_261117.py"""
import sys; sys.path.insert(0, 'src'); sys.path.insert(0, 'scripts'); sys.path.insert(0, 'evidence/G06')
import numpy as np
import gemini_fingertip_detect_r15 as D

Q0 = [86.3, 68.32, 11.37, 31.42]   # Run 4 접촉 시 원시 J1~J4(UNVERIFIED 아님: 기록값)


def cap_and_tilt(q):
    o = D.fk.fk(np.array([list(q) + [90.0]])); t = o['tcp'][0][0] * 1000; R = o['tcp'][1][0]
    a = R[:, 1]; a = a if a[2] < 0 else -a
    return t + a * 24.4, float(np.degrees(np.arccos(-a[2])))


def follow(moves, q0=Q0, bottom0_mm=0.0, gate_mm=2.0, inc_shortfall_deg=0.0, dec_overshoot_deg=0.0):
    """moves: [(관절 번호 2~4, 각도 변화 deg), ...]. 증가 방향은 inc_shortfall_deg만큼 덜 가고, 감소 방향은 dec_overshoot_deg만큼 더 간다.
    반환: (상태 목록, 통과 여부). 상태: dict(move, dz, dy, tilt, bottom_mm)."""
    c0, t0 = cap_and_tilt(q0); q = list(q0); states = []; ok = True
    for i, (j, dq) in enumerate(moves):
        eff = dq - inc_shortfall_deg if dq > 0 else dq - dec_overshoot_deg
        q[j - 1] += eff; c, t = cap_and_tilt(q)
        dz = float(c[2] - c0[2]); dy = float(c[1] - c0[1]); bottom = bottom0_mm + dz
        states.append({'move': i + 1, 'joint': j, 'dq': dq, 'dz': dz, 'dy': dy, 'tilt': t, 'bottom_mm': bottom})
        if bottom < gate_mm: ok = False
    if states and states[0]['dz'] <= 0: ok = False
    return states, ok


if __name__ == '__main__':
    orders = {'J2 +4 → J4 −5 → J2 +4': [(2, 4), (4, -5), (2, 4)], 'J4 −5 → J2 +4 → J2 +4': [(4, -5), (2, 4), (2, 4)], 'J2 +4 → J2 +4 → J4 −5': [(2, 4), (2, 4), (4, -5)]}
    for name, mv in orders.items():
        for label, kw in (('명목', {}), ('최악(증가 2.0° 부족, 감소 0.8° 넘침)', {'inc_shortfall_deg': 2.0, 'dec_overshoot_deg': 0.8})):
            st, ok = follow(mv, **kw)
            print('%s [%s]: %s' % (name, label, '통과' if ok else 'FAIL'))
            for s in st: print('   이동 %d(J%d %+d°): Δz %+.1f mm, Δy %+.1f mm, tilt %.1f°, 블록 바닥 종이 위 %.1f mm' % (s['move'], s['joint'], s['dq'], s['dz'], s['dy'], s['tilt'], s['bottom_mm']))
