#!/usr/bin/env python3

import json
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String


class WorldPoseArrayToJsonBridgeNode(Node):
    def __init__(self):
        super().__init__("world_posearray_to_json_bridge_node")

        self.declare_parameter("input_topic", "/octopus/detections_world_pose")
        self.declare_parameter("output_topic", "/octopus/detections_world")
        self.declare_parameter("source_id", "flight_camera_transform")
        self.declare_parameter("class_name", "trash")
        self.declare_parameter("confidence", 0.8)

        # Indoor/debug mode:
        # If true, the first received world detection becomes a temporary local origin.
        # Output points are shifted into the small 5 m x 3 m prototype grid.
        self.declare_parameter("relative_mode", False)
        self.declare_parameter("relative_origin_x", 2.5)
        self.declare_parameter("relative_origin_y", 1.5)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)

        self.reference_x = None
        self.reference_y = None

        self.pub = self.create_publisher(String, self.output_topic, 10)
        self.sub = self.create_subscription(
            PoseArray,
            self.input_topic,
            self.on_pose_array,
            10,
        )

        self.get_logger().info("World PoseArray to Octopus JSON bridge started")
        self.get_logger().info(f"Input topic: {self.input_topic}")
        self.get_logger().info(f"Output topic: {self.output_topic}")

    def on_pose_array(self, msg: PoseArray):
        source_id = str(self.get_parameter("source_id").value)
        class_name = str(self.get_parameter("class_name").value)
        confidence = float(self.get_parameter("confidence").value)

        relative_mode = bool(self.get_parameter("relative_mode").value)
        relative_origin_x = float(self.get_parameter("relative_origin_x").value)
        relative_origin_y = float(self.get_parameter("relative_origin_y").value)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            stamp = time.time()

        frame_id = msg.header.frame_id if msg.header.frame_id else "map"

        detections = []

        for pose in msg.poses:
            raw_x = float(pose.position.x)
            raw_y = float(pose.position.y)
            raw_z = float(pose.position.z)

            x = raw_x
            y = raw_y

            if relative_mode:
                if self.reference_x is None or self.reference_y is None:
                    self.reference_x = raw_x
                    self.reference_y = raw_y
                    self.get_logger().warn(
                        "Relative debug mode active. "
                        f"Using first detection as reference: x={raw_x:.3f}, y={raw_y:.3f}"
                    )

                x = raw_x - self.reference_x + relative_origin_x
                y = raw_y - self.reference_y + relative_origin_y

            detections.append(
                {
                    "class_name": class_name,
                    "x": x,
                    "y": y,
                    "z": raw_z,
                    "confidence": confidence,
                    "raw_x": raw_x,
                    "raw_y": raw_y,
                }
            )

        payload = {
            "source_id": source_id,
            "frame_id": frame_id,
            "timestamp": stamp,
            "relative_mode": relative_mode,
            "detections": detections,
        }

        out = String()
        out.data = json.dumps(payload)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = WorldPoseArrayToJsonBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
