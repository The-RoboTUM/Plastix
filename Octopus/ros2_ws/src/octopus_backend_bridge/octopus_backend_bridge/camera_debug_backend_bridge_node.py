#!/usr/bin/env python3

import base64
import json
import time
import urllib.request

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class CameraDebugBackendBridgeNode(Node):
    def __init__(self):
        super().__init__("camera_debug_backend_bridge_node")

        self.declare_parameter("image_topic", "/detector_node/debug_image/compressed")
        self.declare_parameter("detections_topic", "/detector_node/detections_debug")
        self.declare_parameter(
            "frame_backend_url",
            "http://127.0.0.1:8000/api/camera_debug/frame",
        )
        self.declare_parameter(
            "detections_backend_url",
            "http://127.0.0.1:8000/api/camera_debug/detections",
        )
        self.declare_parameter("request_timeout_sec", 1.0)
        self.declare_parameter("image_post_period_sec", 0.5)
        self.declare_parameter("detections_post_period_sec", 0.2)
        self.declare_parameter("log_period_sec", 5.0)

        self.image_topic = self.get_parameter("image_topic").value
        self.detections_topic = self.get_parameter("detections_topic").value
        self.frame_backend_url = self.get_parameter("frame_backend_url").value
        self.detections_backend_url = self.get_parameter("detections_backend_url").value
        self.request_timeout_sec = float(self.get_parameter("request_timeout_sec").value)
        self.image_post_period_sec = float(self.get_parameter("image_post_period_sec").value)
        self.detections_post_period_sec = float(
            self.get_parameter("detections_post_period_sec").value
        )
        self.log_period_sec = float(self.get_parameter("log_period_sec").value)

        self.last_image_post_time = 0.0
        self.last_detections_post_time = 0.0
        self.last_error_log_time = 0.0
        self.last_success_log_time = 0.0
        self.image_count = 0
        self.detections_count = 0

        self.image_subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            10,
        )
        self.detections_subscription = self.create_subscription(
            String,
            self.detections_topic,
            self.detections_callback,
            10,
        )

        self.get_logger().info("Camera debug backend bridge started")
        self.get_logger().info(f"Image topic: {self.image_topic}")
        self.get_logger().info(f"Detections topic: {self.detections_topic}")
        self.get_logger().info(f"Frame backend URL: {self.frame_backend_url}")
        self.get_logger().info(f"Detections backend URL: {self.detections_backend_url}")

    @staticmethod
    def stamp_to_float(stamp):
        try:
            value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        except Exception:
            value = 0.0
        return value if value > 0.0 else time.time()

    def image_callback(self, msg: CompressedImage):
        now = time.time()
        if now - self.last_image_post_time < self.image_post_period_sec:
            return
        self.last_image_post_time = now

        payload = {
            "format": "jpeg",
            "stamp": self.stamp_to_float(msg.header.stamp),
            "frame_id": msg.header.frame_id or "camera",
            "data_base64": base64.b64encode(bytes(msg.data)).decode("ascii"),
        }

        try:
            self.post_json(self.frame_backend_url, payload)
        except Exception as exc:
            self.log_error_throttled(f"Failed to POST camera debug frame: {exc}")
            return

        self.image_count += 1
        self.log_success_throttled()

    def detections_callback(self, msg: String):
        now = time.time()
        if now - self.last_detections_post_time < self.detections_post_period_sec:
            return
        self.last_detections_post_time = now

        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.log_error_throttled(f"Invalid detector debug JSON: {exc}")
            return

        payload["bridge_received_at"] = now

        try:
            self.post_json(self.detections_backend_url, payload)
        except Exception as exc:
            self.log_error_throttled(f"Failed to POST camera debug detections: {exc}")
            return

        self.detections_count += 1
        self.log_success_throttled(payload)

    def post_json(self, url, payload):
        data = json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            url,
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=self.request_timeout_sec) as response:
            if response.status < 200 or response.status >= 300:
                raise RuntimeError(f"Backend returned HTTP {response.status}")

    def log_error_throttled(self, message):
        now = time.time()
        if now - self.last_error_log_time >= self.log_period_sec:
            self.get_logger().warn(message)
            self.last_error_log_time = now

    def log_success_throttled(self, payload=None):
        now = time.time()
        if now - self.last_success_log_time < self.log_period_sec:
            return
        detection_count = 0
        if payload:
            detection_count = len(payload.get("detections", []))
        self.get_logger().info(
            "Forwarded camera debug data "
            f"images={self.image_count}, detections_msgs={self.detections_count}, "
            f"latest_detection_count={detection_count}"
        )
        self.last_success_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = CameraDebugBackendBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
