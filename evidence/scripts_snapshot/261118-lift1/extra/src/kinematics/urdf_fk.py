"""DOFBOT Pro URDF 순기구학(오프라인). 하드웨어에 연결하지 않는다.

입력은 SDK 각도(J1~J5, 도). SDK 90°가 URDF 0 rad다(`q = radians(sdk - 90)`).
보정 인자(`gains`, `zeros`, `tool_offset`)는 G06 비관적 모델용이다. 기본값은 URDF 그대로(명목).
- gains: J1~J4 각도 이득. 보정 각도 = 90 + gain·(sdk − 90) + zero
- zeros: J1~J4 영점 오프셋(도)
- tool_offset: Arm5 프레임에서 TCP(`Gripping_point_Joint`)에 더하는 이동(m)

기준 구현은 `scripts/g01_pose_generator.py`의 `fk()`(추적되지 않음)이며, 같은 값을 내는지는 `tests/test_urdf_fk.py`가 고정값으로 확인한다.
모델 값(URDF)은 실물 검증 전 UNVERIFIED다(G02 PASS 범위 안에서만 신뢰).
"""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

DEFAULT_URDF = Path(__file__).resolve().parents[2] / "data" / "refs" / "dofbot-pro-DOFBOT_Pro-V24.urdf"
ARM_JOINTS = ("Arm1_Joint", "Arm2_Joint", "Arm3_Joint", "Arm4_Joint")


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_rotation(axis, q):
    """축 방향 단위벡터 `axis`, 각도 q(rad, 길이 N) -> (N, 3, 3)."""
    x, y, z = axis
    c, s = np.cos(q), np.sin(q)
    t = 1 - c
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = c + x * x * t; R[:, 0, 1] = x * y * t - z * s; R[:, 0, 2] = x * z * t + y * s
    R[:, 1, 0] = y * x * t + z * s; R[:, 1, 1] = c + y * y * t; R[:, 1, 2] = y * z * t - x * s
    R[:, 2, 0] = z * x * t - y * s; R[:, 2, 1] = z * y * t + x * s; R[:, 2, 2] = c + z * z * t
    return R


class UrdfFK:
    def __init__(self, urdf_path=DEFAULT_URDF):
        # stdlib ElementTree는 외부 엔티티·팽창 공격에 약하다. 입력은 Jetson에서 읽어 `data/refs/`에 복사한
        # 프로젝트 소유 URDF 한 파일뿐이므로 허용한다. 신뢰할 수 없는 URDF를 받게 되면 defusedxml(현재 미설치)로 바꿀 것.
        joints = {j.get("name"): j for j in ET.parse(str(urdf_path)).getroot().findall("joint")}
        self._origin = {}
        self._axis = {}
        for name, j in joints.items():
            o = j.find("origin")
            if o is None:
                continue
            xyz = np.array([float(v) for v in o.get("xyz").split()])
            rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
            self._origin[name] = (xyz, rpy_matrix(*rpy))
            a = j.find("axis")
            if a is not None:
                self._axis[name] = [float(v) for v in a.get("xyz").split()]

    def fk(self, sdk, gains=None, zeros=None, tool_offset=None):
        """sdk: (N, 5) SDK 각도(도). 반환: dict(이름 -> (위치 (N,3) m, 회전 (N,3,3)))."""
        sdk = np.asarray(sdk, float)
        if sdk.ndim != 2 or sdk.shape[1] != 5:
            raise ValueError("sdk must be (N, 5)")
        n = len(sdk)
        g = np.ones(4) if gains is None else np.asarray(gains, float)
        z = np.zeros(4) if zeros is None else np.asarray(zeros, float)
        a = sdk.copy()
        a[:, :4] = 90.0 + g * (sdk[:, :4] - 90.0) + z
        q = np.radians(a - 90.0)
        Rm = np.broadcast_to(np.eye(3), (n, 3, 3)).copy()
        p = np.zeros((n, 3))
        out = {}
        for i, name in enumerate(ARM_JOINTS):
            xyz, R0 = self._origin[name]
            p = p + Rm @ xyz
            Rm = Rm @ R0
            Rm = Rm @ axis_rotation(self._axis[name], q[:, i])
            out[name] = (p.copy(), Rm.copy())
        xyz, R0 = self._origin["DaBai_DCW2_Joint"]
        out["dabai"] = (p + Rm @ xyz, Rm @ R0)
        xyz, R0 = self._origin["Arm5_Joint"]
        p5 = p + Rm @ xyz
        R5 = Rm @ R0 @ axis_rotation([0, 1, 0], q[:, 4])
        xyz, _ = self._origin["Gripping_point_Joint"]
        off = np.zeros(3) if tool_offset is None else np.asarray(tool_offset, float)
        out["tcp"] = (p5 + R5 @ (xyz + off), R5)
        out["arm5"] = (p5, R5)
        return out
