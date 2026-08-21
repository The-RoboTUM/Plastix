#!/usr/bin/env python3
"""Publish confirmed trash as WGS84 goals a collector robot can drive to.

The rest of the Octopus stack works in the local ``map`` frame in meters. A robot
navigating with Nav2 over GPS wants latitude/longitude instead, so this node is
the single place where map meters become geographic coordinates.

Indoor fake-GPS demo: the collector robot is started at the same physical spot as
Eve, so both share one fake datum. That datum is Eve's position on the mission
map, published by eve_fake_gps_bridge_node — drag Eve in the dashboard and every
trash coordinate here follows, because they are all expressed relative to her.
None of it is a real satellite fix; the shared reference is what makes it usable.

Topics published:
  * ``/octopus/trash_goal``   NavSatFix, latched. The next target to drive to.
  * ``/octopus/trash_gps``    String (JSON). Every known target with id/lat/lon,
    so the robot (or the dashboard) can plan over the full set instead of one goal.

Topics subscribed besides the detections:
  * ``/octopus/fake_eve_gps_start``  NavSatFix. Eve's start coordinate = the datum.
  * ``/octopus/trash_goal_done``     String. The robot reports the id it finished;
    that target is marked collected and the goal advances to the next one.

The lat/lon math deliberately mirrors ``localToLatLng()`` in the dashboard's
live_data.js (same flat-earth constant), so a target shown on the Mission Map and
the goal sent to the robot are the same coordinate.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String


# Flat-earth approximation, identical to METERS_PER_DEGREE_LAT in live_data.js.
# Over a few dozen meters of indoor demo area the error is far below the
# detector's own accuracy, and matching the dashboard matters more than rigor.
METERS_PER_DEGREE_LAT = 111320.0


class TrashGpsGoalNode(Node):
    def __init__(self):
        super().__init__("trash_gps_goal_node")

        self.declare_parameter("input_topic", "/octopus/detections_world")
        self.declare_parameter("datum_topic", "/octopus/fake_eve_gps_start")
        self.declare_parameter("goal_topic", "/octopus/trash_goal")
        self.declare_parameter("targets_topic", "/octopus/trash_gps")
        self.declare_parameter("goal_done_topic", "/octopus/trash_goal_done")

        # Shared fake start coordinate, normally taken from `datum_topic`. These
        # values are only the bootstrap until the first message arrives; they
        # match DEMO_MAP_ORIGIN in the dashboard.
        self.declare_parameter("datum_lat", 48.2513611)
        self.declare_parameter("datum_lon", 11.6359722)
        self.declare_parameter("altitude_m", 0.0)

        # Two detections closer than this are the same piece of trash. The
        # detector's tracker already settles positions, this only keeps ids
        # stable across messages so "collected" can refer to something.
        self.declare_parameter("merge_radius_m", 0.25)
        self.declare_parameter("min_confidence", 0.0)
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("frame_id", "map")
        # "nearest" = closest to the datum, i.e. closest to where the robot
        # started. "first" = oldest detection first.
        self.declare_parameter("goal_selection", "nearest")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.datum_lat = float(self.get_parameter("datum_lat").value)
        self.datum_lon = float(self.get_parameter("datum_lon").value)
        self.altitude_m = float(self.get_parameter("altitude_m").value)
        self.merge_radius_m = float(self.get_parameter("merge_radius_m").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.goal_selection = str(self.get_parameter("goal_selection").value)

        self.datum_from_topic = False
        self.update_datum(self.datum_lat, self.datum_lon)

        self.targets = []  # ordered by first detection
        self.next_target_id = 1
        self.last_goal_id = None

        latched_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.goal_pub = self.create_publisher(
            NavSatFix, str(self.get_parameter("goal_topic").value), latched_qos
        )
        self.targets_pub = self.create_publisher(
            String, str(self.get_parameter("targets_topic").value), 10
        )

        self.create_subscription(String, self.input_topic, self.detections_callback, 10)
        self.create_subscription(
            String,
            str(self.get_parameter("goal_done_topic").value),
            self.goal_done_callback,
            10,
        )
        # The datum is latched by the publisher, so this arrives right after the
        # subscription is up even if Eve was placed long ago.
        self.create_subscription(
            NavSatFix,
            str(self.get_parameter("datum_topic").value),
            self.datum_callback,
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self.timer = self.create_timer(
            float(self.get_parameter("publish_period_sec").value), self.publish_all
        )

        self.get_logger().info("Trash GPS goal node started")
        self.get_logger().info(f"Input topic: {self.input_topic}")
        self.get_logger().info(
            f"Datum topic: {self.get_parameter('datum_topic').value} "
            f"(bootstrap {self.datum_lat:.7f}, {self.datum_lon:.7f})"
        )
        self.get_logger().info(f"Goal selection: {self.goal_selection}")

    # --- coordinate conversion -------------------------------------------------

    def update_datum(self, lat, lon):
        self.datum_lat = float(lat)
        self.datum_lon = float(lon)
        self.meters_per_degree_lon = METERS_PER_DEGREE_LAT * math.cos(
            math.radians(self.datum_lat)
        )

    def datum_callback(self, msg: NavSatFix):
        lat = self.finite(msg.latitude)
        lon = self.finite(msg.longitude)
        if lat is None or lon is None:
            return

        moved = abs(lat - self.datum_lat) > 1e-9 or abs(lon - self.datum_lon) > 1e-9
        if not moved and self.datum_from_topic:
            return

        self.update_datum(lat, lon)
        self.datum_from_topic = True
        self.get_logger().info(f"Datum is now {lat:.7f}, {lon:.7f}")
        # Targets are stored in map meters, so moving the datum moves every goal
        # with it. Republish immediately instead of waiting for the next tick.
        self.publish_all()

    def local_to_latlon(self, x_m, y_m):
        """Map meters (x = east, y = north) to WGS84 around the datum."""
        lat = self.datum_lat + y_m / METERS_PER_DEGREE_LAT
        lon = self.datum_lon + x_m / self.meters_per_degree_lon
        return lat, lon

    # --- target registry -------------------------------------------------------

    def detections_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Invalid JSON on {self.input_topic}: {exc}")
            return

        now = time.time()
        for detection in payload.get("detections", []):
            x = self.finite(detection.get("x"))
            y = self.finite(detection.get("y"))
            if x is None or y is None:
                continue

            confidence = self.finite(detection.get("confidence"))
            if confidence is not None and confidence < self.min_confidence:
                continue

            self.register(x, y, confidence, detection.get("class_name"), now)

    def register(self, x, y, confidence, class_name, now):
        existing = self.nearest_target(x, y, self.merge_radius_m)
        if existing is not None:
            # Trash does not move: average the position instead of jumping to the
            # newest reading, so the goal stops jittering after a few frames.
            hits = existing["hits"] + 1
            existing["x"] += (x - existing["x"]) / hits
            existing["y"] += (y - existing["y"]) / hits
            existing["hits"] = hits
            existing["last_seen"] = now
            if confidence is not None:
                existing["confidence"] = max(existing["confidence"] or 0.0, confidence)
            return

        self.targets.append({
            "id": self.next_target_id,
            "class_name": str(class_name or "trash"),
            "x": x,
            "y": y,
            "confidence": confidence,
            "hits": 1,
            "first_seen": now,
            "last_seen": now,
            "collected": False,
        })
        self.next_target_id += 1
        self.get_logger().info(
            f"New trash target #{self.targets[-1]['id']} at map ({x:.2f}, {y:.2f})"
        )

    def nearest_target(self, x, y, radius):
        best = None
        best_dist = radius
        for target in self.targets:
            dist = math.hypot(target["x"] - x, target["y"] - y)
            if dist <= best_dist:
                best = target
                best_dist = dist
        return best

    def goal_done_callback(self, msg: String):
        """The robot reports a finished target, by id or as {"id": N} JSON."""
        raw = msg.data.strip()
        target_id = None
        try:
            payload = json.loads(raw)
            target_id = payload.get("id") if isinstance(payload, dict) else payload
        except json.JSONDecodeError:
            target_id = raw

        try:
            target_id = int(target_id)
        except (TypeError, ValueError):
            self.get_logger().warn(f"Cannot read a target id from '{raw}'")
            return

        for target in self.targets:
            if target["id"] == target_id:
                target["collected"] = True
                self.get_logger().info(f"Target #{target_id} marked collected")
                self.publish_all()
                return

        self.get_logger().warn(f"Unknown target id {target_id} reported as done")

    # --- publishing ------------------------------------------------------------

    def open_targets(self):
        return [t for t in self.targets if not t["collected"]]

    def select_goal(self):
        candidates = self.open_targets()
        if not candidates:
            return None
        if self.goal_selection == "first":
            return candidates[0]
        # Map (0, 0) is by construction the datum, so distance from the map origin
        # is distance from where Eve — and with her the robot — started.
        return min(candidates, key=lambda t: math.hypot(t["x"], t["y"]))

    def navsatfix(self, lat, lon):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = self.altitude_m
        # These are demo coordinates derived from a camera, not a receiver. The
        # covariance says "roughly half a meter" so consumers have something
        # sane to weigh instead of an unknown.
        msg.position_covariance = [
            0.25, 0.0, 0.0,
            0.0, 0.25, 0.0,
            0.0, 0.0, 1.0,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        return msg

    def publish_all(self):
        goal = self.select_goal()

        entries = []
        for target in self.targets:
            lat, lon = self.local_to_latlon(target["x"], target["y"])
            entries.append({
                "id": target["id"],
                "class_name": target["class_name"],
                "lat": lat,
                "lon": lon,
                "x": target["x"],
                "y": target["y"],
                "confidence": target["confidence"],
                "collected": target["collected"],
                "is_goal": bool(goal and goal["id"] == target["id"]),
                "last_seen": target["last_seen"],
            })

        self.targets_pub.publish(String(data=json.dumps({
            "source_id": "trash_gps_goal_node",
            "frame_id": self.frame_id,
            "timestamp": time.time(),
            "datum": {
                "lat": self.datum_lat,
                "lon": self.datum_lon,
                # Our own position in the local map frame. Always (0, 0): the
                # frame is anchored on this point, so it cannot be anything else.
                # Stated instead of implied, so a consumer never has to guess
                # where "we" are in the same meters the targets are given in.
                "x": 0.0,
                "y": 0.0,
                "from_topic": self.datum_from_topic,
            },
            "goal_id": goal["id"] if goal else None,
            "open_count": len(self.open_targets()),
            "targets": entries,
        }, separators=(",", ":"))))

        if goal is None:
            if self.last_goal_id is not None:
                self.get_logger().info("No open trash targets left, goal cleared")
                self.last_goal_id = None
            return

        lat, lon = self.local_to_latlon(goal["x"], goal["y"])
        self.goal_pub.publish(self.navsatfix(lat, lon))

        if goal["id"] != self.last_goal_id:
            self.get_logger().info(
                f"Goal is target #{goal['id']}: {lat:.7f}, {lon:.7f} "
                f"(map {goal['x']:.2f}, {goal['y']:.2f})"
            )
            self.last_goal_id = goal["id"]

    @staticmethod
    def finite(value):
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if math.isfinite(number) else None


def main(args=None):
    rclpy.init(args=args)
    node = TrashGpsGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
