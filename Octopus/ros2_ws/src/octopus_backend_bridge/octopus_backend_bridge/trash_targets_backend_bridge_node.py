import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TrashTargetsBackendBridgeNode(Node):
    """Carries the trash target list from ROS to the dashboard backend.

    trash_gps_goal_node owns the list and publishes it on /octopus/trash_gps for
    the collecting robot. The dashboard cannot read ROS - it only polls HTTP - so
    without this bridge the Tasks panel has no source at all.
    """

    def __init__(self):
        super().__init__("trash_targets_backend_bridge_node")

        self.declare_parameter("input_topic", "/octopus/trash_gps")
        self.declare_parameter("backend_url", "http://127.0.0.1:8000/api/trash_targets")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.backend_url = str(self.get_parameter("backend_url").value)

        self.sub = self.create_subscription(
            String,
            self.input_topic,
            self._on_targets,
            10,
        )

        self.get_logger().info("Trash targets backend bridge started")
        self.get_logger().info(f"Input topic: {self.input_topic}")
        self.get_logger().info(f"Posting trash targets to: {self.backend_url}")

    def _on_targets(self, msg: String):
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
            self.get_logger().warn(f"Failed to post trash targets: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = TrashTargetsBackendBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
