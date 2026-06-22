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

        # Local prototype map geometry.
        self.declare_parameter("width_m", 5.0)
        self.declare_parameter("height_m", 3.0)
        self.declare_parameter("resolution", 0.10)
        self.declare_parameter("origin_x", 0.0)
        self.declare_parameter("origin_y", 0.0)
        self.declare_parameter("default_coverage_confidence", 0.8)

        self.width_m = float(self.get_parameter("width_m").value)
        self.height_m = float(self.get_parameter("height_m").value)
        self.resolution = float(self.get_parameter("resolution").value)
        self.origin_x = float(self.get_parameter("origin_x").value)
        self.origin_y = float(self.get_parameter("origin_y").value)
        self.default_coverage_confidence = float(
            self.get_parameter("default_coverage_confidence").value
        )

        self.cols = int(math.ceil(self.width_m / self.resolution))
        self.rows = int(math.ceil(self.height_m / self.resolution))

        self.layers = {
            "coverage": self.make_grid(0.0),
            "trash_probability": self.make_grid(0.0),
            "obstacle_probability": self.make_grid(0.0),
            "confidence": self.make_grid(0.0),
            "last_observed_time": self.make_grid(0.0),
            "source_id": self.make_grid(""),
        }

        self.subscription = self.create_subscription(
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

        self.get_logger().info(
            f"octopus_mapping started: {self.cols}x{self.rows} cells, "
            f"resolution={self.resolution} m/cell"
        )

    def make_grid(self, default_value):
        return [
            [default_value for _ in range(self.cols)]
            for _ in range(self.rows)
        ]

    def detections_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid JSON on /octopus/detections_world: {exc}")
            return

        frame_id = payload.get("frame_id", "map")
        if frame_id != "map":
            self.get_logger().warn(
                f"Received frame_id='{frame_id}'. Prototype expects frame_id='map'."
            )

        timestamp = float(payload.get("timestamp", time.time()))
        source_id = str(payload.get("source_id", "unknown_source"))

        updated_keys = set()

        coverage_polygon = payload.get("coverage_polygon")
        if coverage_polygon:
            coverage_confidence = float(
                payload.get("coverage_confidence", self.default_coverage_confidence)
            )
            updated_keys.update(
                self.apply_coverage_polygon(
                    coverage_polygon=coverage_polygon,
                    source_id=source_id,
                    timestamp=timestamp,
                    confidence=coverage_confidence,
                )
            )

        detections = payload.get("detections", [])
        for detection in detections:
            key = self.apply_detection(
                detection=detection,
                source_id=source_id,
                timestamp=timestamp,
            )
            if key is not None:
                updated_keys.add(key)

        if not updated_keys:
            self.get_logger().warn(
                "Received message but no valid map cells were updated."
            )
            return

        updated_cells = [
            self.cell_to_dict(row, col)
            for row, col in sorted(updated_keys)
        ]

        patch = {
            "timestamp": timestamp,
            "frame_id": "map",
            "updated_cells": updated_cells,
        }

        self.publish_json(self.map_patch_pub, patch)
        self.publish_global_map()
        self.publish_occupancy_grids()

        self.get_logger().info(
            f"Published map patch with {len(updated_cells)} updated cell(s)"
        )

    def apply_detection(self, detection, source_id: str, timestamp: float):
        try:
            x = float(detection["x"])
            y = float(detection["y"])
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn(f"Skipping invalid detection: {detection}")
            return None

        cell = self.xy_to_cell(x, y)
        if cell is None:
            self.get_logger().warn(
                f"Detection outside map bounds: x={x}, y={y}"
            )
            return None

        row, col = cell
        class_name = str(detection.get("class_name", "trash"))
        confidence = float(detection.get("confidence", 1.0))

        self.layers["coverage"][row][col] = 1.0
        self.layers["confidence"][row][col] = max(
            self.layers["confidence"][row][col],
            confidence,
        )
        self.layers["last_observed_time"][row][col] = timestamp
        self.layers["source_id"][row][col] = source_id

        if class_name in ["trash", "plastic", "bottle", "cup", "bag"]:
            self.layers["trash_probability"][row][col] = max(
                self.layers["trash_probability"][row][col],
                confidence,
            )

        if class_name in ["obstacle", "tree", "rock", "wall"]:
            self.layers["obstacle_probability"][row][col] = max(
                self.layers["obstacle_probability"][row][col],
                confidence,
            )

        return (row, col)

    def apply_coverage_polygon(
        self,
        coverage_polygon,
        source_id: str,
        timestamp: float,
        confidence: float,
    ):
        polygon = self.parse_polygon(coverage_polygon)
        if len(polygon) < 3:
            self.get_logger().warn(
                f"Skipping invalid coverage polygon: {coverage_polygon}"
            )
            return set()

        updated_keys = set()

        min_x = max(min(p[0] for p in polygon), self.origin_x)
        max_x = min(max(p[0] for p in polygon), self.origin_x + self.width_m)
        min_y = max(min(p[1] for p in polygon), self.origin_y)
        max_y = min(max(p[1] for p in polygon), self.origin_y + self.height_m)

        min_cell = self.xy_to_cell(min_x, min_y)
        max_cell = self.xy_to_cell(
            max_x - 1e-9,
            max_y - 1e-9,
        )

        if min_cell is None or max_cell is None:
            self.get_logger().warn(
                "Coverage polygon does not overlap map bounds."
            )
            return set()

        min_row, min_col = min_cell
        max_row, max_col = max_cell

        for row in range(min_row, max_row + 1):
            for col in range(min_col, max_col + 1):
                x, y = self.cell_center(row, col)

                if not self.point_in_polygon(x, y, polygon):
                    continue

                self.layers["coverage"][row][col] = 1.0
                self.layers["confidence"][row][col] = max(
                    self.layers["confidence"][row][col],
                    confidence,
                )
                self.layers["last_observed_time"][row][col] = timestamp
                self.layers["source_id"][row][col] = source_id

                updated_keys.add((row, col))

        return updated_keys

    @staticmethod
    def parse_polygon(raw_polygon):
        polygon = []

        for point in raw_polygon:
            try:
                if isinstance(point, dict):
                    x = float(point["x"])
                    y = float(point["y"])
                else:
                    x = float(point[0])
                    y = float(point[1])
                polygon.append((x, y))
            except (KeyError, TypeError, ValueError, IndexError):
                continue

        return polygon

    @staticmethod
    def point_in_polygon(x, y, polygon):
        inside = False
        n = len(polygon)

        j = n - 1
        for i in range(n):
            xi, yi = polygon[i]
            xj, yj = polygon[j]

            intersects = ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi
            )

            if intersects:
                inside = not inside

            j = i

        return inside

    def xy_to_cell(self, x, y):
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

    def cell_to_dict(self, row, col):
        x, y = self.cell_center(row, col)

        return {
            "row": row,
            "col": col,
            "x": x,
            "y": y,
            "coverage": self.layers["coverage"][row][col],
            "trash_probability": self.layers["trash_probability"][row][col],
            "obstacle_probability": self.layers["obstacle_probability"][row][col],
            "confidence": self.layers["confidence"][row][col],
            "source_id": self.layers["source_id"][row][col],
            "last_observed_time": self.layers["last_observed_time"][row][col],
        }

    def publish_json(self, publisher, payload):
        msg = String()
        msg.data = json.dumps(payload)
        publisher.publish(msg)

    def publish_global_map(self):
        cells = {}

        for row in range(self.rows):
            for col in range(self.cols):
                if self.layers["coverage"][row][col] <= 0.0:
                    continue

                key = f"{row},{col}"
                cells[key] = self.cell_to_dict(row, col)

        payload = {
            "timestamp": time.time(),
            "frame_id": "map",
            "width_m": self.width_m,
            "height_m": self.height_m,
            "resolution": self.resolution,
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "rows": self.rows,
            "cols": self.cols,
            "cells": cells,
        }

        self.publish_json(self.global_map_pub, payload)

    def publish_occupancy_grids(self):
        self.coverage_grid_pub.publish(
            self.layer_to_occupancy_grid("coverage")
        )
        self.trash_grid_pub.publish(
            self.layer_to_occupancy_grid("trash_probability")
        )

    def layer_to_occupancy_grid(self, layer_name):
        grid = OccupancyGrid()

        now = self.get_clock().now().to_msg()
        grid.header.stamp = now
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
                coverage = self.layers["coverage"][row][col]
                value = self.layers[layer_name][row][col]

                if coverage <= 0.0:
                    data.append(-1)
                else:
                    data.append(int(max(0.0, min(1.0, value)) * 100.0))

        grid.data = data
        return grid


def main(args=None):
    rclpy.init(args=args)
    node = GridMapBuilderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
