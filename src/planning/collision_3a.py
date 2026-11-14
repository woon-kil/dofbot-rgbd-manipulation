"""G06 3-A+ 간이 충돌 검사(오프라인). 하드웨어에 연결하지 않는다.

모델: 작업면(흰 보드) 평면 + 링크별 볼록껍질(URDF 메시) + 손가락 캡 캡슐(사진 실측).
- 링크 볼록껍질과 평면의 거리는 껍질 꼭짓점의 최저 높이로 정확히 구한다(선형 함수의 최솟값은 껍질 꼭짓점에서 난다).
- 링크 볼록껍질과 베이스 껍질 사이는 `max_i(n_i·x + d_i)`(점-볼록다면체 부호 거리의 하한)을 쓴다. 비볼록 부분을 껍질이 채우므로 보수적이다.
- 캡슐 치수는 `docs/261114-2055-S4-13-항b-점-TCP-계산-결과보고서.md`의 사진 실측(UNVERIFIED, J6=98 전제).
"""
import struct
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

MESH_DIR = Path(__file__).resolve().parents[2] / "data" / "refs" / "dofbot_meshes"

# 손가락 캡 (arm5 프레임, m). 안쪽 간격 48.7 mm, 캡 두께 약 9.7 mm, 길이 약 25 mm.
CAP_CENTER_X = 0.02435 + 0.00485
CAP_LENGTH = 0.025
CAP_RADIUS = 0.005
FINGER_BAR_LENGTH = 0.025   # 캡 아래 금속 링크 연장(가정, UNVERIFIED)
FINGER_BAR_RADIUS = 0.004


def read_stl_vertices(path):
    """바이너리 STL의 꼭짓점 (M,3) m. ASCII STL은 지원하지 않는다."""
    b = Path(path).read_bytes()
    n = struct.unpack("<I", b[80:84])[0]
    if 84 + 50 * n != len(b):
        raise ValueError("binary STL expected: %s" % path)
    a = np.frombuffer(b[84:], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]), count=n)
    return a["v"].reshape(-1, 3).astype(float)


class Hull:
    """볼록껍질: 꼭짓점(`vertices`)과 평면 방정식 `n·x + d <= 0`(안쪽)."""

    def __init__(self, points):
        h = ConvexHull(np.asarray(points, float))
        self.vertices = np.asarray(points, float)[h.vertices]
        self.equations = h.equations  # (F,4)

    def distance_lb(self, pts):
        """점들 (K,3)에서 껍질까지의 거리 하한(안쪽이면 음수). (K,)"""
        pts = np.asarray(pts, float)
        return (pts @ self.equations[:, :3].T + self.equations[:, 3]).max(axis=1)


def load_link_hull(name, mesh_dir=MESH_DIR):
    return Hull(read_stl_vertices(Path(mesh_dir) / ("%s.STL" % name)))


def finger_capsules(tip_y):
    """arm5 프레임 캡슐 목록 [(p0, p1, r)]. tip_y: 캡 끝면의 arm5 y(m). 캡 두 개 + 금속 링크 연장 두 개."""
    caps = []
    for sx in (-1.0, 1.0):
        x = sx * CAP_CENTER_X
        caps.append((np.array([x, tip_y, 0.0]), np.array([x, tip_y - CAP_LENGTH, 0.0]), CAP_RADIUS))
        caps.append((np.array([x, tip_y - CAP_LENGTH, 0.0]), np.array([x, tip_y - CAP_LENGTH - FINGER_BAR_LENGTH, 0.0]), FINGER_BAR_RADIUS))
    return caps


def capsule_lowest_z(caps, p5, R5):
    """캡슐들의 최저 높이(월드 z). p5, R5: arm5 위치·회전."""
    z = []
    for a, b, r in caps:
        wa = p5 + R5 @ a
        wb = p5 + R5 @ b
        z.append(min(wa[2], wb[2]) - r)
    return min(z)


def capsule_points(caps, p5, R5, n=5):
    """캡슐 축을 따라 월드 점 샘플(반지름 포함 안 함)과 반지름 배열."""
    pts, rad = [], []
    for a, b, r in caps:
        for t in np.linspace(0, 1, n):
            pts.append(p5 + R5 @ (a + (b - a) * t))
            rad.append(r)
    return np.array(pts), np.array(rad)


class Box:
    """월드 좌표 축 정렬 직육면체(블록·받침대). center, half: (3,) m."""

    def __init__(self, center, half):
        self.center = np.asarray(center, float)
        self.half = np.asarray(half, float)

    def signed_distance(self, pts):
        """점들 (...,3) -> 부호 거리(밖 양수, 안 음수). 유클리드 거리(밖), 안쪽은 가장 가까운 면까지의 음수 거리."""
        q = np.abs(np.asarray(pts, float) - self.center) - self.half
        outside = np.linalg.norm(np.maximum(q, 0.0), axis=-1)
        inside = np.max(q, axis=-1)
        return np.where(np.any(q > 0, axis=-1), outside, inside)


def link_world_vertices(hull, p, R):
    return hull.vertices @ R.T + p


def clearances(fkout, hulls, caps, table_z, dz=0.0, boxes=None):
    """한 자세의 최소 여유(m). fkout: `UrdfFK.fk` 결과, hulls: {'base_link', 'Arm2_Joint'...}, table_z: 작업면 월드 z,
    dz: 모델 월드 높이 오프셋(실제 − FK, 음수면 실제가 낮음, 모든 점에 더함).
    반환 dict: table(링크·손가락 중 최저), base(이동 링크·손가락과 베이스 껍질), 항목별 table_links."""
    off = np.array([0.0, 0.0, dz])
    out = {"table_links": {}}
    link_frames = {"Arm2_Joint": "Arm2_Link", "Arm3_Joint": "Arm3_Link", "Arm4_Joint": "Arm4_Link", "dabai": "DaBai_DCW2_Link", "arm5": "Arm5_Link"}
    for key, hull in hulls.items():
        if key == "base_link" or key not in fkout:
            continue
        p, R = fkout[key][0][0] + off, fkout[key][1][0]
        out["table_links"][key] = float(link_world_vertices(hull, p, R)[:, 2].min() - table_z)
    p5, R5 = fkout["arm5"][0][0] + off, fkout["arm5"][1][0]
    out["table_fingers"] = float(capsule_lowest_z(caps, p5, R5) - table_z)
    out["table"] = float(min(list(out["table_links"].values()) + [out["table_fingers"]]))
    base = hulls.get("base_link")
    if base is not None:
        vals = []
        for key, hull in hulls.items():
            if key in ("base_link", "Arm1_Joint", "Arm2_Joint") or key not in fkout:
                continue
            p, R = fkout[key][0][0] + off, fkout[key][1][0]
            vals.append(float(base.distance_lb(link_world_vertices(hull, p, R)).min()))
        pts, rad = capsule_points(caps, p5, R5)
        vals.append(float((base.distance_lb(pts) - rad).min()))
        out["base"] = min(vals)
    if boxes:
        vals = []
        for box in boxes:
            for key, hull in hulls.items():
                if key in ("base_link", "Arm1_Joint", "Arm2_Joint") or key not in fkout:
                    continue
                p, R = fkout[key][0][0] + off, fkout[key][1][0]
                vals.append(float(box.signed_distance(link_world_vertices(hull, p, R)).min()))
            pts, rad = capsule_points(caps, p5, R5)
            vals.append(float((box.signed_distance(pts) - rad).min()))
        out["box"] = min(vals)
    return out
