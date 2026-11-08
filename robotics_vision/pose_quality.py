"""Planar PnP ambiguity inspection; no motion commands."""

import cv2
import numpy as np
from .aruco import marker_object_points
from .geometry import rigid_transform


def solve_square(corners, K, dist, length_m, max_error_px=2.0):
    corners = np.asarray(corners, np.float64)
    K = np.asarray(K, np.float64)
    dist = np.asarray(dist, np.float64)
    if corners.shape != (4, 2) or not np.isfinite(corners).all():
        raise ValueError("Four finite image corners required")
    if (
        K.shape != (3, 3)
        or not np.isfinite(K).all()
        or K[0, 0] <= 0
        or K[1, 1] <= 0
        or not np.allclose(K[2], [0, 0, 1])
    ):
        raise ValueError("Invalid camera intrinsics")
    if dist.size not in (4, 5, 8, 12, 14) or not np.isfinite(dist).all():
        raise ValueError("Invalid lens distortion coefficients")
    if (
        not np.isfinite(length_m)
        or length_m <= 0
        or not np.isfinite(max_error_px)
        or max_error_px <= 0
    ):
        raise ValueError("Positive length and error limit required")
    area = abs(cv2.contourArea(corners.astype(np.float32)))
    if area < 16:
        raise ValueError("Marker is degenerate or too small for reliable pose")
    obj = marker_object_points(length_m).astype(np.float64)
    result = cv2.solvePnPGeneric(obj, corners, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    candidates = []
    for rv, tv in zip(result[1], result[2]):
        rot, _ = cv2.Rodrigues(rv)
        if np.any((rot @ obj.T + tv.reshape(3, 1))[2] <= 0):
            continue
        points, _ = cv2.projectPoints(obj, rv, tv, K, dist)
        rms = float(
            np.sqrt(np.mean(np.sum((points.reshape(4, 2) - corners) ** 2, axis=1)))
        )
        candidates.append(
            {
                "T_camera_marker": rigid_transform(rot, tv),
                "reprojection_rms_px": rms,
                "rvec": rv,
                "tvec": tv,
            }
        )
    candidates.sort(key=lambda c: c["reprojection_rms_px"])
    if not candidates or candidates[0]["reprojection_rms_px"] > max_error_px:
        raise ValueError("No pose passes reprojection/cheirality gates")
    gap = (
        candidates[1]["reprojection_rms_px"] - candidates[0]["reprojection_rms_px"]
        if len(candidates) > 1
        else None
    )
    return {
        **candidates[0],
        "candidate_count": len(candidates),
        "ambiguity_gap_px": gap,
        "ambiguous": gap is not None and gap < 0.25,
    }
