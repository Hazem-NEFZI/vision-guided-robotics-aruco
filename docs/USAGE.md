# Usage

```bash
python -m robotics_vision.demo --output outputs
python -m robotics_vision.estimate image.png configs/calibration.example.json --output outputs/poses.json
```

The supplied calibration is hypothetical. Replace K, distortion coefficients, image resolution, marker side length and T_base_camera with your measured calibration. Input image dimensions must match calibration resolution. The marker dictionary is DICT_4X4_50.

`poses.json` includes marker ID, base-frame pose, pregrasp, reprojection RMS and planar ambiguity. No detected/accepted markers yields an empty list, which downstream code must handle explicitly.

With ROS 2 installed and sourced:

```bash
python ros2_publish_pose.py --pose outputs/poses.json
```

The ROS dependency is intentionally separate from the pip-installable perception package. Do not run example transforms on a physical robot as motion commands.
