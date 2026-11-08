import numpy as np
import pytest
import cv2
from scipy.spatial.transform import Rotation
from robotics_vision.geometry import (
    rigid_transform,
    pose_to_json,
    inverse_transform,
    validate_transform,
    camera_to_base,
)
from robotics_vision.pose_quality import solve_square
from robotics_vision.aruco import marker_object_points


@pytest.mark.parametrize(
    "axis", [[1, -1, 0], [1, 0, -1], [0, 1, -1], [1, 2, -3], [0, 0, 1]]
)
def test_half_turn_round_trip(axis):
    axis = np.asarray(axis, float)
    axis /= np.linalg.norm(axis)
    R = Rotation.from_rotvec(np.pi * axis).as_matrix()
    T = rigid_transform(R, [1, 2, 3])
    reconstructed = Rotation.from_quat(pose_to_json(T)["orientation_xyzw"]).as_matrix()
    assert np.allclose(R, reconstructed, atol=1e-9)
    assert np.allclose(inverse_transform(T) @ T, np.eye(4))


@pytest.mark.parametrize(
    "bad", [np.zeros((4, 4)), np.full((4, 4), np.nan), np.diag([1, 1, -1, 1])]
)
def test_invalid_transform(bad):
    with pytest.raises(ValueError):
        validate_transform(bad)


def test_pose_with_known_projection():
    K = np.array([[800.0, 0, 320], [0, 800, 240], [0, 0, 1]])
    rv = np.array([2.9, 0.2, 0.1])
    tv = np.array([0.01, 0.02, 0.5])
    corners, _ = cv2.projectPoints(marker_object_points(0.06), rv, tv, K, np.zeros(5))
    result = solve_square(corners.reshape(4, 2), K, np.zeros(5), 0.06)
    assert result["reprojection_rms_px"] < 1e-3
    assert np.allclose(result["T_camera_marker"][:3, 3], tv, atol=1e-4)


def test_invalid_approach():
    with pytest.raises(ValueError):
        camera_to_base(np.eye(4), np.eye(4), np.nan)
