#!/usr/bin/env python3

import json
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MapPatchBackendBridgeNode(Node):
    def __init__(self):
        super().__init__("map_patch_backend_bridge_node")

        self.declare_parameter(
            "backend_url",
            "http://127.0.0.1:8000/api/map_patch",
        )
        self.declare_parameter("timeout_sec", 2.0)

        self.backend_url = self.get_parameter("backend_url").value
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)

        self.subscription = self.create_subscription(
            String,
            "/octopus/map_patch",
            self.map_patch_callback,
            10,
        )

        self.get_logger().info("Map patch backend bridge started")
        self.get_logger().info(f"Posting map patches to: {self.backend_url}")

    def map_patch_callback(self, msg: String):
        try:
            patch = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid map patch JSON: {exc}")
            return

        try:
            self.post_patch(patch)
        except Exception as exc:
            self.get_logger().error(f"Failed to POST map patch: {exc}")
            return

        updated_cells = patch.get("updated_cells", [])
        self.get_logger().info(
            f"Posted map patch to backend with {len(updated_cells)} updated cell(s)"
        )

    def post_patch(self, patch: dict):
        body = json.dumps(patch).encode("utf-8")

        request = urllib.request.Request(
            self.backend_url,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )

        with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
            if response.status < 200 or response.status >= 300:
                raise RuntimeError(f"Backend returned HTTP {response.status}")


def main(args=None):
    rclpy.init(args=args)
    node = MapPatchBackendBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
