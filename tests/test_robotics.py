import numpy as np
from robotics_vision.geometry import (
    rigid_transform,
    camera_to_base,
    pose_to_json,
    in_workspace,
)
from robotics_vision.aruco import estimate_tag_poses
from robotics_vision.demo import synthetic_image


def test_marker_detection_and_distance():
    im, K = synthetic_image()
    tags = estimate_tag_poses(im, K, np.zeros(5), 0.055)
    assert len(tags) == 1
    assert tags[0]["id"] == 9
    assert 0.18 < tags[0]["T_camera_marker"][2, 3] < 0.35


def test_transform_chain_and_unit_quaternion():
    cam = rigid_transform(np.diag([1.0, -1.0, -1.0]), [0.42, 0.05, 0.85])
    marker = rigid_transform(np.eye(3), [0, 0, 0.27])
    base, pre = camera_to_base(cam, marker)
    assert np.allclose(base[:3, 3], [0.42, 0.05, 0.58])
    assert np.isclose(pre[2, 3], base[2, 3] + 0.1)
    assert np.isclose(np.linalg.norm(pose_to_json(base)["orientation_xyzw"]), 1)
    assert in_workspace(base[:3, 3])
