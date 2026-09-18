import json
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String


class LocalCameraGridNode(Node):
    """
    Converts normalized camera detections u/v into a local camera-footprint grid.

    This is dashboard/debug only:
      - u/v are normalized image coordinates in [0, 1]
      - x/y are local footprint coordinates in meters
      - x=0..footprint_width_m
      - y=0..footprint_height_m

    It does NOT create global robot-navigation coordinates.
    """

    def __init__(self):
        super().__init__("local_camera_grid_node")

        self.declare_parameter("input_topic", "/detector_node/confirmed")
        self.declare_parameter("output_topic", "/octopus/local_camera_grid_patch")
        self.declare_parameter("frame_id", "camera_footprint")

        self.declare_parameter("footprint_width_m", 4.46)
        self.declare_parameter("footprint_height_m", 3.34)
        self.declare_parameter("resolution_m", 0.10)
        self.declare_parameter("confidence", 0.8)
        self.declare_parameter("source_id", "local_camera_grid")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.pub = self.create_publisher(String, output_topic, 10)
        self.sub = self.create_subscription(
            PoseArray,
            input_topic,
            self._on_detections,
            10,
        )

        self.get_logger().info("Local camera grid node started")
        self.get_logger().info(f"Input topic: {input_topic}")
        self.get_logger().info(f"Output topic: {output_topic}")

    def _on_detections(self, msg: PoseArray):
        width = float(self.get_parameter("footprint_width_m").value)
        height = float(self.get_parameter("footprint_height_m").value)
        resolution = float(self.get_parameter("resolution_m").value)
        confidence = float(self.get_parameter("confidence").value)
        frame_id = str(self.get_parameter("frame_id").value)
        source_id = str(self.get_parameter("source_id").value)

        now = time.time()
        updated_cells = []

        for pose in msg.poses:
            u = float(pose.position.x)
            v = float(pose.position.y)

            # Clamp debug input so a noisy detector cannot break the grid.
            u = max(0.0, min(1.0, u))
            v = max(0.0, min(1.0, v))

            x = u * width
            y = v * height

            col = int(x / resolution)
            row = int(y / resolution)

            updated_cells.append({
                "row": row,
                "col": col,
                "x": x,
                "y": y,
                "u": u,
                "v": v,
                "coverage": 1.0,
                "trash_probability": 0.8,
                "obstacle_probability": 0.0,
                "confidence": confidence,
                "source_id": source_id,
                "last_observed_time": now,
            })

        patch = {
            "timestamp": now,
            "frame_id": frame_id,
            "grid_type": "local_camera_grid",
            "footprint_width_m": width,
            "footprint_height_m": height,
            "resolution_m": resolution,
            "updated_cells": updated_cells,
        }

        out = String()
        out.data = json.dumps(patch)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LocalCameraGridNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
