#!/usr/bin/env python3

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid


class GridMapBuilderNode(Node):
    def __init__(self):
        super().__init__("grid_map_builder_node")

        # Map size: first prototype field
        self.declare_parameter("width_m", 5.0)
        self.declare_parameter("height_m", 3.0)
        self.declare_parameter("resolution", 0.10)
        self.declare_parameter("origin_x", 0.0)
        self.declare_parameter("origin_y", 0.0)

        self.width_m = float(self.get_parameter("width_m").value)
        self.height_m = float(self.get_parameter("height_m").value)
        self.resolution = float(self.get_parameter("resolution").value)
        self.origin_x = float(self.get_parameter("origin_x").value)
        self.origin_y = float(self.get_parameter("origin_y").value)

        self.cols = int(math.ceil(self.width_m / self.resolution))
        self.rows = int(math.ceil(self.height_m / self.resolution))

        self.coverage = self.make_grid(0.0)
        self.trash_probability = self.make_grid(0.0)
        self.obstacle_probability = self.make_grid(0.0)
        self.confidence = self.make_grid(0.0)
        self.last_observed_time = self.make_grid(0.0)
        self.source_id = [["" for _ in range(self.cols)] for _ in range(self.rows)]

        self.detection_sub = self.create_subscription(
            String,
            "/octopus/detections_world",
            self.detections_callback,
            10,
        )

        self.map_patch_pub = self.create_publisher(
            String,
            "/octopus/map_patch",
            10,
        )

        self.global_map_pub = self.create_publisher(
            String,
            "/octopus/global_map",
            10,
        )

        self.coverage_grid_pub = self.create_publisher(
            OccupancyGrid,
            "/octopus/coverage_grid",
            10,
        )

        self.trash_grid_pub = self.create_publisher(
            OccupancyGrid,
            "/octopus/trash_grid",
            10,
        )

        self.timer = self.create_timer(1.0, self.publish_periodic_maps)

        self.get_logger().info(
            f"octopus_mapping started: {self.cols}x{self.rows} cells, "
            f"resolution={self.resolution} m/cell"
        )

    def make_grid(self, value):
        return [[value for _ in range(self.cols)] for _ in range(self.rows)]

    def detections_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid JSON: {exc}")
            return

        source_id = payload.get("source_id", "unknown_source")
        frame_id = payload.get("frame_id", "map")
        detections = payload.get("detections", [])

        if frame_id != "map":
            self.get_logger().warn(
                f"Expected frame_id='map', got frame_id='{frame_id}'"
            )

        updated_cells = []

        for detection in detections:
            updated_cell = self.apply_detection(detection, source_id)
            if updated_cell is not None:
                updated_cells.append(updated_cell)

        if not updated_cells:
            return

        patch = {
            "timestamp": time.time(),
            "frame_id": "map",
            "updated_cells": updated_cells,
        }

        out = String()
        out.data = json.dumps(patch)
        self.map_patch_pub.publish(out)

        self.get_logger().info(
            f"Published map patch with {len(updated_cells)} updated cell(s)"
        )

    def apply_detection(self, detection, source_id):
        try:
            x = float(detection["x"])
            y = float(detection["y"])
        except KeyError:
            self.get_logger().warn(f"Detection missing x/y: {detection}")
            return None

        class_name = detection.get("class_name", "trash")
        confidence = float(detection.get("confidence", 0.8))
        detection_source = detection.get("source_id", source_id)

        cell = self.map_to_cell(x, y)

        if cell is None:
            self.get_logger().warn(f"Detection outside map: x={x:.2f}, y={y:.2f}")
            return None

        row, col = cell
        now = time.time()

        self.coverage[row][col] = 1.0
        self.confidence[row][col] = max(self.confidence[row][col], confidence)
        self.last_observed_time[row][col] = now
        self.source_id[row][col] = detection_source

        if class_name in ["trash", "plastic", "bottle", "cup", "bag"]:
            self.trash_probability[row][col] = max(
                self.trash_probability[row][col],
                confidence,
            )

        if class_name in ["obstacle", "tree", "rock", "wall"]:
            self.obstacle_probability[row][col] = max(
                self.obstacle_probability[row][col],
                confidence,
            )

        cell_x, cell_y = self.cell_center(row, col)

        return {
            "row": row,
            "col": col,
            "x": cell_x,
            "y": cell_y,
            "coverage": self.coverage[row][col],
            "trash_probability": self.trash_probability[row][col],
            "obstacle_probability": self.obstacle_probability[row][col],
            "confidence": self.confidence[row][col],
            "source_id": self.source_id[row][col],
            "last_observed_time": self.last_observed_time[row][col],
        }

    def map_to_cell(self, x, y):
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)

        if row < 0 or row >= self.rows:
            return None
        if col < 0 or col >= self.cols:
            return None

        return row, col

    def cell_center(self, row, col):
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    def publish_periodic_maps(self):
        self.publish_global_map()
        self.publish_occupancy_grid(
            publisher=self.coverage_grid_pub,
            layer=self.coverage,
            mode="coverage",
        )
        self.publish_occupancy_grid(
            publisher=self.trash_grid_pub,
            layer=self.trash_probability,
            mode="probability",
        )

    def publish_global_map(self):
        payload = {
            "frame_id": "map",
            "width_m": self.width_m,
            "height_m": self.height_m,
            "resolution": self.resolution,
            "rows": self.rows,
            "cols": self.cols,
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "layers": {
                "coverage": self.coverage,
                "trash_probability": self.trash_probability,
                "obstacle_probability": self.obstacle_probability,
                "confidence": self.confidence,
                "source_id": self.source_id,
                "last_observed_time": self.last_observed_time,
            },
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.global_map_pub.publish(msg)

    def publish_occupancy_grid(self, publisher, layer, mode):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"

        grid.info.resolution = self.resolution
        grid.info.width = self.cols
        grid.info.height = self.rows
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        data = []

        for row in range(self.rows):
            for col in range(self.cols):
                value = layer[row][col]

                if mode == "coverage":
                    data.append(0 if value > 0.5 else -1)
                else:
                    data.append(max(0, min(100, int(value * 100))))

        grid.data = data
        publisher.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = GridMapBuilderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
