import cv2
import numpy as np


def marker_object_points(length_m):
    h = float(length_m) / 2
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)


def estimate_tag_poses(rgb, K, dist, marker_length_m=0.055):
    if marker_length_m <= 0:
        raise ValueError("Marker length must be positive meters")
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return []
    results = []
    for marker_id, c in zip(ids.flatten(), corners):
        from .pose_quality import solve_square

        try:
            quality = solve_square(
                np.asarray(c).reshape(4, 2), K, dist, marker_length_m
            )
        except ValueError:
            continue
        results.append(
            {"id": int(marker_id), "corners": np.asarray(c).reshape(4, 2), **quality}
        )
    return results
