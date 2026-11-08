import argparse
import json
from pathlib import Path
import numpy as np
import cv2
from .geometry import rigid_transform, camera_to_base, pose_to_json, in_workspace
from .aruco import estimate_tag_poses


def synthetic_image():
    h, w = 500, 720
    canvas = np.full((h, w, 3), (45, 54, 73), np.uint8)
    cv2.rectangle(canvas, (135, 75), (600, 420), (68, 81, 102), -1)
    cv2.rectangle(canvas, (145, 85), (590, 410), (84, 99, 113), 3)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, 9, 180)
    marker = cv2.cvtColor(marker, cv2.COLOR_GRAY2RGB)
    canvas[165:345, 270:450] = marker
    cv2.putText(
        canvas,
        "SYNTHETIC CAMERA FRAME",
        (30, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.85,
        (235, 242, 248),
        2,
    )
    K = np.array([[790.0, 0.0, w / 2], [0.0, 790.0, h / 2], [0.0, 0.0, 1.0]])
    return canvas, K


def main(output="outputs"):
    out = Path(output)
    out.mkdir(parents=True, exist_ok=True)
    image, K = synthetic_image()
    distortion = np.zeros((5, 1))
    poses = estimate_tag_poses(image, K, distortion, marker_length_m=0.055)
    if not poses:
        raise RuntimeError("No ArUco marker detected in demo")
    # EXAMPLE CAMERA EXTRINSIC: not a real calibration.
    R = np.diag(
        [1.0, -1.0, -1.0]
    )  # camera points downward in this hypothetical base frame
    T_base_camera = rigid_transform(R, [0.42, 0.05, 0.85])
    records = []
    for tag in poses:
        Tbase, Tpre = camera_to_base(
            T_base_camera, tag["T_camera_marker"], approach_m=0.10
        )
        records.append(
            {
                "marker_id": tag["id"],
                "marker_in_robot_base": pose_to_json(Tbase),
                "pregrasp_in_robot_base": pose_to_json(Tpre),
                "marker_in_camera": pose_to_json(tag["T_camera_marker"]),
                "inside_demo_workspace": in_workspace(Tbase[:3, 3]),
            }
        )
        cv2.polylines(image, [tag["corners"].astype(np.int32)], True, (28, 237, 185), 3)
        cv2.drawFrameAxes(image, K, distortion, tag["rvec"], tag["tvec"], 0.03, 3)
        cv2.putText(
            image,
            f"ID {tag['id']} | z={tag['tvec'][2, 0]:.3f}m",
            (165, 385),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (239, 240, 244),
            2,
        )
    cv2.imwrite(str(out / "demo.png"), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    data = {
        "source": "synthetic",
        "camera_intrinsics": K.tolist(),
        "camera_extrinsic": "hypothetical only; replace with calibrated T_base_camera",
        "T_base_camera": T_base_camera.tolist(),
        "poses": records,
        "warning": "Visualization ONLY; no robot commands are issued.",
    }
    (out / "poses.json").write_text(json.dumps(data, indent=2))
    print(
        json.dumps(
            {
                "markers": len(records),
                "workspace_ok": records[0]["inside_demo_workspace"],
            },
            indent=2,
        )
    )
    return data


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--output", default="outputs")
    a = p.parse_args()
    main(a.output)
