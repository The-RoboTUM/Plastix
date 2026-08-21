#!/usr/bin/env python3

"""Forward per-device status topics to the dashboard backend.

Ground robots publish their own state as a JSON string, e.g. GripperX on
``/octopus/devices/gripperx/status``:

    {"source_id": "gripperx_demo", "robot_id": "gripperx", "timestamp": 1787234926.3,
     "pose": {"status": "ok", "frame_id": "map", "x": 1.2, "y": 0.4,
              "yaw_deg": 35.0, "lat": 48.2513, "lon": 11.6359},
     "nav": {"status": "idle", "active_goal_id": null, "distance_remaining_m": null},
     "armed": false,
     "battery": {"status": "unavailable", "reason": "NO_SENSOR_INSTALLED",
                 "percent": null, "voltage_v": null},
     "link": {"connected": true, "last_rx_age_sec": 0.7}}

``pose.lat``/``pose.lon`` are what puts the robot on the mission map. They are null
until the robot has a GPS datum (``pose.status == "no_datum"``); the payload is
forwarded either way so the dashboard can say *why* there is no marker.

The topic list is a parameter, so a second robot only needs its topic added -
no code change.
"""

import json
import time
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DeviceStatusBackendBridgeNode(Node):
    def __init__(self):
        super().__init__("device_status_backend_bridge_node")

        self.declare_parameter("status_topics", ["/octopus/devices/gripperx/status"])
        self.declare_parameter(
            "backend_url_template",
            "http://127.0.0.1:8000/api/devices/{device_id}/status",
        )
        self.declare_parameter("request_timeout_sec", 1.0)
        self.declare_parameter("post_period_sec", 0.5)
        self.declare_parameter("log_period_sec", 5.0)

        self.status_topics = list(self.get_parameter("status_topics").value or [])
        self.backend_url_template = self.get_parameter("backend_url_template").value
        self.request_timeout_sec = float(self.get_parameter("request_timeout_sec").value)
        self.post_period_sec = float(self.get_parameter("post_period_sec").value)
        self.log_period_sec = float(self.get_parameter("log_period_sec").value)

        self.last_post_time = {}
        self.post_count = {}
        self.last_error_log_time = 0.0
        self.last_success_log_time = 0.0

        self.subscriptions_by_topic = {}
        for topic in self.status_topics:
            device_id = self.device_id_from_topic(topic)
            self.subscriptions_by_topic[topic] = self.create_subscription(
                String,
                topic,
                self.make_callback(topic, device_id),
                10,
            )
            self.get_logger().info(f"Device status: {topic} -> device_id '{device_id}'")

        self.get_logger().info("Device status backend bridge started")
        if not self.status_topics:
            self.get_logger().warn("No status_topics configured - nothing is forwarded")

    @staticmethod
    def device_id_from_topic(topic):
        """'/octopus/devices/gripperx/status' -> 'gripperx'."""
        parts = [part for part in str(topic).split("/") if part]
        if len(parts) >= 2 and parts[-1] == "status":
            return parts[-2]
        return parts[-1] if parts else "unknown"

    def make_callback(self, topic, fallback_device_id):
        def callback(msg: String):
            self.status_callback(topic, fallback_device_id, msg)
        return callback

    def status_callback(self, topic, fallback_device_id, msg: String):
        now = time.time()
        if now - self.last_post_time.get(topic, 0.0) < self.post_period_sec:
            return
        self.last_post_time[topic] = now

        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.log_error_throttled(f"Invalid JSON on {topic}: {exc}")
            return

        if not isinstance(payload, dict):
            self.log_error_throttled(f"Payload on {topic} is not a JSON object")
            return

        # The robot names itself; the topic name is only the fallback.
        device_id = str(payload.get("robot_id") or fallback_device_id).strip().lower()
        if not device_id:
            device_id = fallback_device_id

        payload["bridge_received_at"] = now
        payload["source_topic"] = topic

        url = self.backend_url_template.format(device_id=device_id)
        try:
            self.post_json(url, payload)
        except Exception as exc:
            self.log_error_throttled(f"Failed to POST {device_id} status: {exc}")
            return

        self.post_count[device_id] = self.post_count.get(device_id, 0) + 1
        self.log_success_throttled(device_id, payload)

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

    def log_success_throttled(self, device_id, payload):
        now = time.time()
        if now - self.last_success_log_time < self.log_period_sec:
            return
        pose = payload.get("pose") or {}
        pose_status = pose.get("status", "unknown")
        lat = pose.get("lat")
        lon = pose.get("lon")
        where = (
            f"lat={lat:.7f} lon={lon:.7f}"
            if isinstance(lat, (int, float)) and isinstance(lon, (int, float))
            else f"no position (pose.status={pose_status})"
        )
        self.get_logger().info(
            f"Forwarded {device_id} status posts={self.post_count.get(device_id, 0)}, {where}"
        )
        self.last_success_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = DeviceStatusBackendBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
