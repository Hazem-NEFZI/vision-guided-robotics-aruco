"""Validated SE(3) transforms. T_a_b maps b-frame coordinates into frame a."""

import numpy as np
from scipy.spatial.transform import Rotation


def validate_transform(T):
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError("Transform must be a finite 4x4 matrix")
    if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError("Invalid homogeneous last row")
    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-7) or not np.isclose(
        np.linalg.det(R), 1, atol=1e-7
    ):
        raise ValueError("Rotation must belong to SO(3)")
    return T


def rigid_transform(R, t):
    T = np.eye(4)
    R = np.asarray(R, float)
    t = np.asarray(t, float)
    if R.shape != (3, 3) or t.size != 3:
        raise ValueError("Expected 3x3 rotation and three translation values")
    T[:3, :3] = R
    T[:3, 3] = t.reshape(3)
    return validate_transform(T)


def inverse_transform(T):
    T = validate_transform(T)
    return rigid_transform(T[:3, :3].T, -T[:3, :3].T @ T[:3, 3])


def transform_xyz(T, position):
    T = validate_transform(T)
    p = np.asarray(position, float)
    if p.shape != (3,) or not np.isfinite(p).all():
        raise ValueError("Expected finite XYZ vector")
    return T[:3, :3] @ p + T[:3, 3]


def pose_to_json(T):
    T = validate_transform(T)
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    if q[3] < 0:
        q = -q
    return {"position_xyz_m": T[:3, 3].tolist(), "orientation_xyzw": q.tolist()}


def camera_to_base(T_base_camera, T_camera_marker, approach_m=0.1):
    if not np.isfinite(approach_m) or approach_m < 0:
        raise ValueError("Approach distance must be finite and nonnegative")
    target = validate_transform(T_base_camera) @ validate_transform(T_camera_marker)
    pre = target.copy()
    pre[2, 3] += approach_m
    return target, pre


def in_workspace(xyz, mins=(0.15, -0.50, 0.05), maxs=(0.8, 0.50, 1.0)):
    p, lo, hi = map(lambda x: np.asarray(x, float), (xyz, mins, maxs))
    if (
        any(x.shape != (3,) for x in (p, lo, hi))
        or not np.isfinite([p, lo, hi]).all()
        or np.any(lo > hi)
    ):
        raise ValueError("Invalid finite workspace vectors")
    return bool(np.all(p >= lo) and np.all(p <= hi))
