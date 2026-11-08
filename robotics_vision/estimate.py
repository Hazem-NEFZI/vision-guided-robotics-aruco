"""Estimate targets from a real saved camera image and explicit calibration JSON."""

import argparse, json
from pathlib import Path
import cv2
import numpy as np
from .aruco import estimate_tag_poses
from .geometry import validate_transform, camera_to_base, pose_to_json


def estimate(image_path, calibration_path, output):
    cfg = json.loads(Path(calibration_path).read_text())
    K = np.asarray(cfg["K"], float)
    dist = np.asarray(cfg["distortion"], float)
    base_camera = validate_transform(cfg["T_base_camera"])
    image = cv2.imread(str(image_path))
    if image is None:
        raise ValueError("Cannot decode image")
    if list(image.shape[1::-1]) != cfg["image_size_wh"]:
        raise ValueError("Image dimensions differ from calibration resolution")
    tags = estimate_tag_poses(
        cv2.cvtColor(image, cv2.COLOR_BGR2RGB), K, dist, cfg["marker_length_m"]
    )
    rows = []
    for tag in tags:
        base, pre = camera_to_base(base_camera, tag["T_camera_marker"])
        rows.append(
            {
                "marker_id": tag["id"],
                "marker_in_robot_base": pose_to_json(base),
                "pregrasp_in_robot_base": pose_to_json(pre),
                "reprojection_rms_px": tag["reprojection_rms_px"],
                "ambiguous": tag["ambiguous"],
                "ambiguity_gap_px": tag["ambiguity_gap_px"],
            }
        )
    result = {
        "source": str(image_path),
        "poses": rows,
        "motion_authorized": False,
        "note": "Ambiguous planar poses require additional views or external constraints.",
    }
    target = Path(output)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(result, indent=2))
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument("image")
    p.add_argument("calibration")
    p.add_argument("--output", default="outputs/poses.json")
    a = p.parse_args()
    estimate(a.image, a.calibration, a.output)


if __name__ == "__main__":
    main()
