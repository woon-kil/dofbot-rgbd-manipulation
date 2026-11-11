"""Small dependency-free rigid-transform contract checker (not a robot driver)."""
import math

def identity():
    return [[float(i == j) for j in range(4)] for i in range(4)]

RIGID_TOL = 1e-6

def _check_rigid(t):
    """회전부가 직교(RtR=I)이고 det=+1인지 검사한다. 허용오차 RIGID_TOL."""
    r = [row[:3] for row in t[:3]]
    for i in range(3):
        for j in range(3):
            dot = sum(r[k][i] * r[k][j] for k in range(3))
            if abs(dot - float(i == j)) > RIGID_TOL:
                raise ValueError("non-orthonormal rotation")
    det = (r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
           - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
           + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]))
    if abs(det - 1.0) > RIGID_TOL:
        raise ValueError("rotation determinant is not +1")

def inverse(t):
    """강체 변환의 역: [R|p]^-1 = [R^T | -R^T p]."""
    _check_shape(t)
    _check_rigid(t)
    rt = [[t[j][i] for j in range(3)] for i in range(3)]
    p = [t[i][3] for i in range(3)]
    inv = [rt[i] + [-sum(rt[i][k] * p[k] for k in range(3))] for i in range(3)]
    inv.append([0.0, 0.0, 0.0, 1.0])
    return inv

def _check_shape(t):
    if len(t) != 4 or any(len(r) != 4 for r in t):
        raise ValueError("4x4 matrices required")
    if not all(math.isfinite(v) for r in t for v in r):
        raise ValueError("nonfinite transform")
    if any(abs(t[3][j] - float(j == 3)) > 1e-9 for j in range(4)):
        raise ValueError("invalid homogeneous row")

def multiply(a, b):
    if len(a) != 4 or len(b) != 4 or any(len(r) != 4 for r in a + b):
        raise ValueError("4x4 matrices required")
    if not all(math.isfinite(v) for r in a + b for v in r):
        raise ValueError("nonfinite transform")
    if any(abs(a[3][j] - float(j == 3)) > 1e-9 or abs(b[3][j] - float(j == 3)) > 1e-9 for j in range(4)):
        raise ValueError("invalid homogeneous row")
    _check_rigid(a)
    _check_rigid(b)
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]

def transform_point(t, xyz):
    if len(xyz) != 3 or not all(math.isfinite(x) for x in xyz):
        raise ValueError("3 finite coordinates required")
    return [sum(t[i][k] * xyz[k] for k in range(3)) + t[i][3] for i in range(3)]
