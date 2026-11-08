# Design: perception before motion

`T_a_b` maps coordinates expressed in b into frame a. Lengths are meters, image coordinates are pixels, and quaternions are XYZW. Every composed transform must have finite entries, a proper SO(3) rotation and homogeneous last row. Quaternion conversion uses a rotation implementation tested at mixed-sign 180° axes.

Planar IPPE can return more than one solution. The estimator checks positive point depths, ranks candidates by pixel reprojection RMS, rejects candidates above 2 px and reports ambiguity when the residual gap is below 0.25 px. These numeric defaults are engineering heuristics, not uncertainty bounds. A small residual alone cannot resolve planar ambiguity.

The pregrasp offset is along BASE +Z; it is not automatically aligned with the marker normal or tool approach axis. The marker pose is not a calibrated tool grasp pose. Real use needs marker-to-grasp and flange-to-tool transforms, collision checking and a motion stack.

The pipeline does not command robot motion. The ROS adapter publishes a visualization pose and rejects explicitly flagged ambiguous poses. ROS 2 runtime and real robot tests remain external integration work.

Reference: https://docs.opencv.org/4.12.0/d5/d1f/calib3d_solvePnP.html
