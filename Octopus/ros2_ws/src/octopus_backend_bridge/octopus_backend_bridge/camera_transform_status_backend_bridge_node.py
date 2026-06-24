#!/usr/bin/env python3

import json
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CameraTransformStatusBackendBridgeNode(Node):
    def __init__(self):
        super().__init__("camera_transform_status_backend_bridge_node")

        self.declare_parameter("status_topic", "/octopus/camera_transform/status")
        self.declare_parameter(
            "backend_url",
            "http://127.0.0.1:8000/api/camera_transform/status",
        )
        self.declare_parameter("request_timeout_sec", 1.0)
        self.declare_parameter("log_period_sec", 5.0)

        self.status_topic = self.get_parameter("status_topic").value
        self.backend_url = self.get_parameter("backend_url").value
        self.request_timeout_sec = float(
            self.get_parameter("request_timeout_sec").value
        )
        self.log_period_sec = float(self.get_parameter("log_period_sec").value)

        self.last_error_log_time = 0.0
        self.last_success_log_time = 0.0
        self.forwarded_count = 0

        self.subscription = self.create_subscription(
            String,
            self.status_topic,
            self.status_callback,
            10,
        )

        self.get_logger().info("Camera transform status backend bridge started")
        self.get_logger().info(f"Status topic: {self.status_topic}")
        self.get_logger().info(f"Backend URL: {self.backend_url}")

    def status_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.log_error_throttled(f"Invalid camera transform status JSON: {exc}")
            return

        payload["bridge_received_at"] = time.time()

        try:
            self.post_json(payload)
        except Exception as exc:
            self.log_error_throttled(
                f"Failed to forward camera transform status to backend: {exc}"
            )
            return

        self.forwarded_count += 1
        now = time.time()
        if now - self.last_success_log_time >= self.log_period_sec:
            state = payload.get("state", "unknown")
            markers = payload.get("detected_marker_ids", [])
            missing = payload.get("missing_marker_ids", [])
            self.get_logger().info(
                "Forwarded camera transform status "
                f"count={self.forwarded_count}, "
                f"state={state}, markers={markers}, missing={missing}"
            )
            self.last_success_log_time = now

    def post_json(self, payload):
        data = json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            self.backend_url,
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )

        with urllib.request.urlopen(
            request,
            timeout=self.request_timeout_sec,
        ) as response:
            if response.status < 200 or response.status >= 300:
                raise RuntimeError(f"Backend returned HTTP {response.status}")

    def log_error_throttled(self, message):
        now = time.time()
        if now - self.last_error_log_time >= self.log_period_sec:
            self.get_logger().warn(message)
            self.last_error_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = CameraTransformStatusBackendBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
