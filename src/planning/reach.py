"""G06 도달성(IK, 오프라인). 하드웨어에 연결하지 않는다.

5축 팔: J5(손목 회전)는 위치를 바꾸지 않으므로 90 고정. J1은 방위각, J2~J4는 팔 평면 안의 3링크.
목표는 (월드 x, y, z)의 캡 끝면 가운데(`tip`)와 접근 기울기 tilt(도, 부호 있음)다.
tilt = atan2(a·r̂, -a_z), a = R5·[0,1,0](손가락 축), r̂ = 팔 평면의 바깥 방향(수평). 0이면 수직 아래, + 이면 끝이 베이스에서 멀어지는 쪽.

모델(`Model`): J1~J4 이득, 월드 높이 오프셋 dz(실제 − FK), 캡 끝 오프셋 tip_y(TCP 기준 arm5 y, m).
"""
from dataclasses import dataclass, field

import numpy as np

from kinematics.urdf_fk import UrdfFK

SDK_MIN = np.array([0.0, 0.0, 5.0, 5.0])      # J3, J4 하한 5(G01.5b 후보값, 미검증)
SDK_MAX = np.array([180.0, 180.0, 180.0, 180.0])
TCP_ARM5_Y = 0.09                              # URDF Gripping_point (arm5 y, m)


@dataclass(frozen=True)
class Model:
    gains: tuple = (1.0, 1.0, 1.0, 1.0)
    dz: float = 0.0          # m, 실제 높이 − FK 높이 (음수 = 실제가 낮음)
    tip_y: float = -0.004    # m, 캡 끝면의 TCP 기준 arm5 y 오프셋 (S4-13: 약 −4 mm)

    def tool_offset(self):
        return np.array([0.0, self.tip_y, 0.0])


URDF_MODEL = Model(gains=(1, 1, 1, 1), dz=0.0, tip_y=0.0)


def tip_state(fk, q4, model, j5=90.0):
    """q4(SDK J1~J4) -> (tip 위치 m 월드(실제), 손가락 축 a, fk 결과)."""
    out = fk.fk(np.array([list(q4) + [j5]]), gains=model.gains, tool_offset=model.tool_offset())
    tip = out["tcp"][0][0] + np.array([0.0, 0.0, model.dz])
    a = out["tcp"][1][0] @ np.array([0.0, 1.0, 0.0])
    return tip, a, out


def tilt_deg(a, rhat):
    return float(np.degrees(np.arctan2(a @ rhat, -a[2])))


def _residual(fk, q234, j1, target, tilt, rhat, model):
    q4 = [j1, q234[0], q234[1], q234[2]]
    tip, a, _ = tip_state(fk, q4, model)
    r = np.array([(tip - target) @ rhat, tip[2] - target[2], np.radians(tilt_deg(a, rhat) - tilt)])
    return r


def solve_ik(fk, target, tilt, model, seeds=None, x0=None, tol=1e-5, max_iter=40):
    """외부 진입점: J1 방위각을 한 번 보정(팔 평면의 옆 오프셋)해 두 번 푼다."""
    first = _solve_ik(fk, target, tilt, model, seeds, x0, tol, max_iter, 0.0)
    if not first.get("q"):
        return first
    tip, _, _ = tip_state(fk, first["q"], model)
    x, y, _z = target
    r = float(np.hypot(x, y))
    lat = float((tip - np.array(target)) @ np.array([-y / r, x / r, 0.0]))
    second = _solve_ik(fk, target, tilt, model, seeds, first["q"][1:], tol, max_iter, -np.degrees(lat / r))
    return second if second.get("q") else first


def _solve_ik(fk, target, tilt, model, seeds, x0, tol, max_iter, j1_corr_deg):
    """목표 tip(월드 m, 실제 높이)와 tilt(도) -> dict(ok, q(SDK 4), residual_mm, limit_ok).
    J1은 방위각으로 정하고 J2~J4를 감쇠 뉴턴으로 푼다. 여러 시작점(팔꿈치 위·아래) 중 잔차가 가장 작고 한계 안인 해를 고른다."""
    x, y, z = target
    r = float(np.hypot(x, y))
    if r < 1e-9:
        return {"ok": False, "reason": "axis"}
    j1 = 90.0 + float(np.degrees(np.arctan2(-x, y))) + j1_corr_deg
    rhat = np.array([x / r, y / r, 0.0])
    if not (SDK_MIN[0] <= j1 <= SDK_MAX[0]):
        return {"ok": False, "reason": "j1_limit", "j1": j1}
    starts = [np.array(s, float) for s in (seeds if seeds is not None else [(95, 25, 40), (95, 60, 20), (110, 15, 15), (80, 70, 60), (120, 40, 40)])]
    if x0 is not None:
        starts = [np.array(x0, float)] + starts
    best = None
    for s in starts:
        q = s.copy()
        lam = 1e-3
        res = _residual(fk, q, j1, target, tilt, rhat, model)
        for _ in range(max_iter):
            if np.abs(res[:2]).max() < tol and abs(res[2]) < 1e-4:
                break
            J = np.empty((3, 3))
            for k in range(3):
                dq = np.zeros(3); dq[k] = 1e-3
                J[:, k] = (_residual(fk, q + dq, j1, target, tilt, rhat, model) - res) / 1e-3
            step = np.linalg.solve(J.T @ J + lam * np.eye(3), -J.T @ res)
            step = np.clip(step, -15.0, 15.0)
            q_new = np.clip(q + step, SDK_MIN[1:], SDK_MAX[1:])
            r_new = _residual(fk, q_new, j1, target, tilt, rhat, model)
            if np.linalg.norm(r_new) < np.linalg.norm(res):
                q, res, lam = q_new, r_new, max(lam / 3, 1e-9)
            else:
                lam *= 10
        tipf, _, _ = tip_state(fk, [j1, *q], model)
        lat = float((tipf - np.array(target)) @ np.array([-y / r, x / r, 0.0]))
        err_mm = float(np.sqrt((res[:2] * 1000) @ (res[:2] * 1000) + (lat * 1000) ** 2))
        tilt_err = float(np.degrees(res[2]))
        cand = {"q": [j1, *[float(v) for v in q]], "residual_mm": err_mm, "tilt_err_deg": tilt_err}
        if best is None or err_mm + abs(tilt_err) < best["residual_mm"] + abs(best["tilt_err_deg"]):
            best = cand
    ok = best["residual_mm"] < 0.05 and abs(best["tilt_err_deg"]) < 0.1
    lim = bool(np.all(np.array(best["q"]) >= SDK_MIN - 1e-6) and np.all(np.array(best["q"]) <= SDK_MAX + 1e-6))
    best.update(ok=bool(ok and lim), limit_ok=lim, reason=None if ok and lim else ("no_solution" if not ok else "joint_limit"))
    return best
