"""Optional ROS 2 visualization publisher; never commands a robot."""

import argparse
import json
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--pose", default="outputs/poses.json")
    a = p.parse_args()
    with open(a.pose, encoding="utf-8") as source:
        poses = json.load(source)["poses"]
    if not poses:
        raise ValueError("No marker poses available")
    if poses[0].get("ambiguous", False):
        raise ValueError("Planar pose is ambiguous; obtain additional observations")
    record = poses[0]["marker_in_robot_base"]
    rclpy.init()
    node = Node("vision_target_visualizer")
    pub = node.create_publisher(PoseStamped, "/vision/target_pose", 10)
    try:
        msg = PoseStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = record[
            "position_xyz_m"
        ]
        (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ) = record["orientation_xyzw"]
        # Publish several times to allow subscribers to discover the topic.
        for _ in range(5):
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.2)
        node.get_logger().info(
            "Published perception-only target pose; no motion issued"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
