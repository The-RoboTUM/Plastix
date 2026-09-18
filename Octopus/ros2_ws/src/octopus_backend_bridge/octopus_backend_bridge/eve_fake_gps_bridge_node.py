#!/usr/bin/env python3
"""Publish Eve's fake GPS start coordinate from the dashboard into ROS.

Eve's placement on the mission map is browser state: the operator drags the
marker and it lands in localStorage. The collector robot is started at that same
physical spot, so this coordinate is the datum every trash GPS goal is relative
to — which means ROS needs it.

The dashboard POSTs the coordinate to the backend on every placement; this node
polls the backend and republishes it as a NavSatFix, so dragging the marker moves
the datum for the whole robot fleet.

This is the only bridge that runs backend -> ROS. All others go ROS -> backend,
which is why it polls instead of subscribing: the FastAPI backend holds the value
but cannot push into the ROS graph.

Publishes ``/octopus/fake_eve_gps_start`` (sensor_msgs/NavSatFix, latched).
"""

import json
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus


class EveFakeGpsBridgeNode(Node):
    def __init__(self):
        super().__init__("eve_fake_gps_bridge_node")

        self.declare_parameter("backend_url", "http://127.0.0.1:8000/api/eve/fake_gps")
        self.declare_parameter("output_topic", "/octopus/fake_eve_gps_start")
        self.declare_parameter("poll_period_sec", 1.0)
        self.declare_parameter("request_timeout_sec", 1.0)
        self.declare_parameter("log_period_sec", 10.0)
        self.declare_parameter("frame_id", "map")

        # Used until the dashboard has posted anything, so a robot starting before
        # the operator opens the dashboard still gets a usable datum. Matches
        # DEMO_MAP_ORIGIN in live_data.js.
        self.declare_parameter("fallback_lat", 48.2513611)
        self.declare_parameter("fallback_lon", 11.6359722)
        self.declare_parameter("publish_fallback", True)

        self.backend_url = str(self.get_parameter("backend_url").value)
        self.request_timeout_sec = float(self.get_parameter("request_timeout_sec").value)
        self.log_period_sec = float(self.get_parameter("log_period_sec").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.fallback_lat = float(self.get_parameter("fallback_lat").value)
        self.fallback_lon = float(self.get_parameter("fallback_lon").value)
        self.publish_fallback = bool(self.get_parameter("publish_fallback").value)

        self.last_published = None
        self.last_error_log_time = 0.0
        self.using_fallback = False

        latched_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            NavSatFix, str(self.get_parameter("output_topic").value), latched_qos
        )

        self.timer = self.create_timer(
            float(self.get_parameter("poll_period_sec").value), self.poll
        )

        self.get_logger().info("Eve fake GPS bridge started")
        self.get_logger().info(f"Backend URL: {self.backend_url}")
        self.get_logger().info(
            f"Publishing: {self.get_parameter('output_topic').value}"
        )

    def poll(self):
        coordinate = self.fetch()

        if coordinate is None:
            if not self.publish_fallback or self.last_published is not None:
                return
            # Nothing from the dashboard yet and nothing published before.
            if not self.using_fallback:
                self.get_logger().warn(
                    "No Eve position from the dashboard yet, publishing the "
                    f"fallback datum {self.fallback_lat:.7f}, {self.fallback_lon:.7f}"
                )
                self.using_fallback = True
            self.publish(self.fallback_lat, self.fallback_lon)
            return

        lat, lon = coordinate
        moved = (
            self.last_published is None
            or abs(self.last_published[0] - lat) > 1e-9
            or abs(self.last_published[1] - lon) > 1e-9
        )
        if moved:
            self.get_logger().info(
                f"Eve start coordinate: {lat:.7f}, {lon:.7f}"
                + (" (was fallback)" if self.using_fallback else "")
            )
            self.using_fallback = False

        self.publish(lat, lon)
        self.last_published = (lat, lon)

    def fetch(self):
        try:
            with urllib.request.urlopen(
                self.backend_url, timeout=self.request_timeout_sec
            ) as response:
                body = response.read().decode("utf-8", errors="replace")
            payload = json.loads(body)
        except Exception as exc:
            self.log_error_throttled(f"Cannot read Eve position from backend: {exc}")
            return None

        coordinate = payload.get("eve_fake_gps")
        if not isinstance(coordinate, dict):
            return None

        try:
            lat = float(coordinate["lat"])
            lon = float(coordinate["lon"])
        except (KeyError, TypeError, ValueError):
            self.log_error_throttled(f"Backend returned no usable lat/lon: {coordinate}")
            return None

        return lat, lon

    def publish(self, lat, lon):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = 0.0
        msg.position_covariance = [
            0.25, 0.0, 0.0,
            0.0, 0.25, 0.0,
            0.0, 0.0, 1.0,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.publisher.publish(msg)

    def log_error_throttled(self, message):
        now = time.time()
        if now - self.last_error_log_time >= self.log_period_sec:
            self.get_logger().warn(message)
            self.last_error_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = EveFakeGpsBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
