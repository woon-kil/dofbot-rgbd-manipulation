"""G06 관측 자세 후보(오프라인): 파지 목표가 DaBai 컬러 영상 안에 있고 깊이가 최소 거리 이상인 자세 중 tip이 목표에 가장 가까운 자세를 찾는다.

경로 길이 = 관측 자세 tip과 파지 tip 사이 직선거리(하한). 수직 후퇴선 위로 한정하지 않는다(그 선 위 자세는 대부분 관절 한계 밖이다).
카메라 내부 파라미터: `evidence/G02/dabai_color_camera_info.json`(640×480, fx 479.07, fy 478.98, cx 315.07, cy 237.41), 왜곡은 무시(주변부 여유는 margin으로).
hand-eye: `evidence/G02/g04_solve.json` fit.PARK(rot_deg, trans_mm). R_LC: URDF 링크 프레임 -> 카메라 광학 프레임(x 오른쪽, y 아래, z 앞).
"""
import numpy as np
from scipy.spatial.transform import Rotation as Rot

from planning.reach import tip_state

K = (479.07, 478.98, 315.07, 237.41)
W, H = 640, 480
HE_ROT_DEG = (-0.276, -1.161, -0.293)
HE_TRANS_M = (-0.0136, -0.00078, 0.00251)
R_LC = np.array([[1.0, 0, 0], [0, 0, 1], [0, -1.0, 0]])
RD = Rot.from_rotvec(np.radians(HE_ROT_DEG)).as_matrix()


def camera_pose(fk, q4, model, j5=90.0):
    out = fk.fk(np.array([list(q4) + [j5]]), gains=model.gains, tool_offset=model.tool_offset())
    dp, dR = out["dabai"][0][0], out["dabai"][1][0]
    cp = dp + dR @ np.array(HE_TRANS_M) + np.array([0.0, 0.0, model.dz])
    return cp, dR @ RD @ R_LC


def project(point, cp, Rc):
    """월드 점 -> (u, v, depth m)."""
    p = Rc.T @ (np.asarray(point, float) - cp)
    return K[0] * p[0] / p[2] + K[2], K[1] * p[1] / p[2] + K[3], float(p[2])


def visible(point, cp, Rc, dmin, margin=40):
    u, v, d = project(point, cp, Rc)
    return bool(d >= dmin and margin <= u <= W - margin and margin <= v <= H - margin), (u, v, d)


def axis_dir(fk, q4, model):
    return tip_state(fk, q4, model)[1]


class PoseGrid:
    """J2~J4 격자(J1=90, 팔 평면 x=0) 전 자세의 tip·손가락 축·tilt·카메라 자세. J1은 팔 전체를 z축으로 돌릴 뿐이라
    목표를 팔 평면으로 회전하면 가시성·거리가 같다(정확한 대칭, tip의 옆 오프셋 약 1 mm 이하는 무시)."""

    def __init__(self, fk, model, step=2.0, j234_min=(0.0, 5.0, 5.0), j234_max=(180.0, 180.0, 180.0)):
        axes = [np.arange(lo, hi + 1e-9, step) for lo, hi in zip(j234_min, j234_max)]
        J2, J3, J4 = np.meshgrid(*axes, indexing="ij")
        self.sdk = np.stack([np.full(J2.size, 90.0), J2.ravel(), J3.ravel(), J4.ravel()], 1)
        out = fk.fk(np.concatenate([self.sdk, np.full((len(self.sdk), 1), 90.0)], 1), gains=model.gains, tool_offset=model.tool_offset())
        off = np.array([0.0, 0.0, model.dz])
        self.tip = out["tcp"][0] + off
        self.p5 = out["arm5"][0] + off
        self.R5 = out["arm5"][1]
        self.axis = out["tcp"][1] @ np.array([0.0, 1.0, 0.0])
        self.tilt = np.degrees(np.arctan2(self.axis[:, 1], -self.axis[:, 2]))
        dp, dR = out["dabai"]
        self.cp = dp + dR @ np.array(HE_TRANS_M) + off
        self.Rc = dR @ RD @ R_LC

    def visible_mask(self, target, dmin, margin=40, max_tilt=45.0, max_joint=170.0):
        p = np.einsum("nji,nj->ni", self.Rc, np.asarray(target, float) - self.cp)
        depth = p[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = K[0] * p[:, 0] / depth + K[2]
            v = K[1] * p[:, 1] / depth + K[3]
        ok = (depth >= max(dmin, 1e-6)) & (u >= margin) & (u <= W - margin) & (v >= margin) & (v <= H - margin)
        ok &= (np.abs(self.tilt) <= max_tilt) & (self.sdk[:, 1:].max(axis=1) <= max_joint)   # 관측 자세 tilt·관절 여유(제안값, 미동결)
        return ok, u, v, depth

    def nearest_observations(self, target, dmin, margin=40, k=1):
        """목표가 보이는 자세 중 tip 거리(= 관측→파지 경로 길이의 하한)가 가장 작은 k개. 반환 [(index, dist_m, (u, v, depth))]."""
        ok, u, v, d = self.visible_mask(target, dmin, margin)
        idx = np.flatnonzero(ok)
        if len(idx) == 0:
            return []
        dist = np.linalg.norm(self.tip[idx] - np.asarray(target, float), axis=1)
        order = np.argsort(dist)[:k]
        return [(int(idx[o]), float(dist[o]), (float(u[idx[o]]), float(v[idx[o]]), float(d[idx[o]]))) for o in order]
