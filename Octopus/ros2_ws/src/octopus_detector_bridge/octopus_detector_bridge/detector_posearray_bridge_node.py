#!/usr/bin/env python3

import json
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String


class DetectorPoseArrayBridgeNode(Node):
    def __init__(self):
        super().__init__("detector_posearray_bridge_node")

        self.declare_parameter("input_topic", "/detector_node/confirmed")
        self.declare_parameter("output_topic", "/octopus/detections_world")
        self.declare_parameter("input_coordinate_mode", "map")
        self.declare_parameter("source_id", "detector_node")
        self.declare_parameter("default_class_name", "trash")
        self.declare_parameter("default_confidence", 1.0)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.input_coordinate_mode = self.get_parameter("input_coordinate_mode").value
        self.source_id = self.get_parameter("source_id").value
        self.default_class_name = self.get_parameter("default_class_name").value
        self.default_confidence = float(self.get_parameter("default_confidence").value)

        if self.input_coordinate_mode not in ["map", "normalized_image"]:
            raise ValueError(
                "input_coordinate_mode must be either 'map' or 'normalized_image'"
            )

        self.publisher = self.create_publisher(
            String,
            self.output_topic,
            10,
        )

        self.subscription = self.create_subscription(
            PoseArray,
            self.input_topic,
            self.posearray_callback,
            10,
        )

        self.get_logger().info("Detector PoseArray bridge started")
        self.get_logger().info(f"Input topic: {self.input_topic}")
        self.get_logger().info(f"Output topic: {self.output_topic}")
        self.get_logger().info(f"Input coordinate mode: {self.input_coordinate_mode}")

        if self.input_coordinate_mode == "normalized_image":
            self.get_logger().warn(
                "normalized_image mode does not publish map detections yet. "
                "A camera/marker or Pixhawk transform is needed first."
            )

    def posearray_callback(self, msg: PoseArray):
        if self.input_coordinate_mode == "normalized_image":
            self.get_logger().warn(
                "Received PoseArray in normalized_image mode. "
                "Skipping /octopus/detections_world publish to avoid wrong map updates."
            )
            return

        timestamp = self.header_stamp_to_float(msg)

        detections = []
        for pose in msg.poses:
            detections.append(
                {
                    "class_name": self.default_class_name,
                    "x": float(pose.position.x),
                    "y": float(pose.position.y),
                    "confidence": self.default_confidence,
                }
            )

        payload = {
            "source_id": self.source_id,
            "frame_id": msg.header.frame_id if msg.header.frame_id else "map",
            "timestamp": timestamp,
            "detections": detections,
        }

        out = String()
        out.data = json.dumps(payload)
        self.publisher.publish(out)

        self.get_logger().info(
            f"Published {len(detections)} detection(s) to {self.output_topic}"
        )

    @staticmethod
    def header_stamp_to_float(msg: PoseArray) -> float:
        sec = int(msg.header.stamp.sec)
        nanosec = int(msg.header.stamp.nanosec)

        if sec == 0 and nanosec == 0:
            return time.time()

        return float(sec) + float(nanosec) * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = DetectorPoseArrayBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
