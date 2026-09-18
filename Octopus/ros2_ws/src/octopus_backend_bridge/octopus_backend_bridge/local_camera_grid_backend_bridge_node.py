import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LocalCameraGridBackendBridgeNode(Node):
    def __init__(self):
        super().__init__("local_camera_grid_backend_bridge_node")

        self.declare_parameter("input_topic", "/octopus/local_camera_grid_patch")
        self.declare_parameter("backend_url", "http://127.0.0.1:8000/api/local_camera_grid")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.backend_url = str(self.get_parameter("backend_url").value)

        self.sub = self.create_subscription(
            String,
            self.input_topic,
            self._on_patch,
            10,
        )

        self.get_logger().info("Local camera grid backend bridge started")
        self.get_logger().info(f"Input topic: {self.input_topic}")
        self.get_logger().info(f"Posting local camera grid to: {self.backend_url}")

    def _on_patch(self, msg: String):
        try:
            data = msg.data.encode("utf-8")
            req = urllib.request.Request(
                self.backend_url,
                data=data,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=1.0) as response:
                response.read()
        except Exception as exc:
            self.get_logger().warn(f"Failed to post local camera grid patch: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = LocalCameraGridBackendBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
