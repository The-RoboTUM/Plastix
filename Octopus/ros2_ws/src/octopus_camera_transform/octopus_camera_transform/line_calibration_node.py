#!/usr/bin/env python3
"""Hold and publish the `octopus_line` reference-line calibration.

Spec: `Octopus/docs/line_calibration.md`. This node covers sections 1-5 of the
GripperX draft — the frame, the pixel-to-metre transform and the scale L. It
deliberately stops there:

**It does not feed the projection.** `flight_camera_transform_node` keeps using
`manual_height_above_ground_m` and the PX4 startup yaw exactly as before, and
the datum keeps coming from the Eve marker. Section 4 of the spec moves the
datum to the line midpoint, which is a contract change affecting GripperX, the
dashboard and `trash_gps_goal_node` at once — not something to switch on while
the document still says "Draft" and the demo runs on the current behaviour.

What it does instead is make the calibration observable, so the numbers can be
compared against what the running system assumes before anything is rewired.
The comparison worth having is `implied_camera_height_m` against the configured
height: the map scales linearly with that constant, and the line measures it.

Input, in order of precedence:
  1. the backend (`/api/line_calibration`), where the dashboard will POST the
     operator's two marks once the marking UI exists (spec section 7, step 6);
  2. ROS parameters, so the node is usable today without that UI.

Publishes ``/octopus/line_calibration/status`` (std_msgs/String, JSON) and POSTs
the same payload to the backend. Under ``/octopus/*``, so the rosbridge glob
already covers it and GripperX can read it without a new topic being allowed.
"""

import hashlib
import json
import time
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from octopus_camera_transform.line_frame import (
    DEFAULT_MAX_LENGTH_M,
    DEFAULT_MIN_LENGTH_M,
    LineCalibration,
    LineCalibrationError,
)


class LineCalibrationNode(Node):

    def __init__(self):
        super().__init__("line_calibration_node")

        self.declare_parameter("output_topic", "/octopus/line_calibration/status")
        self.declare_parameter("input_url", "http://127.0.0.1:8000/api/line_calibration")
        self.declare_parameter("status_url", "http://127.0.0.1:8000/api/line_calibration/status")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("request_timeout_sec", 1.0)
        self.declare_parameter("log_period_sec", 10.0)

        # Manual input, used when the backend has nothing. -1 means "unset";
        # a pixel coordinate is never negative.
        self.declare_parameter("pixel_a", [-1.0, -1.0])
        self.declare_parameter("pixel_b", [-1.0, -1.0])
        self.declare_parameter("length_m", 0.0)
        self.declare_parameter("mirrored", False)

        # Section 5/6: GripperX reports the live calibration's id and its L in
        # its telemetry so the value our operator typed in can be checked
        # against the one their LiDAR measured. The manual entry stays - this is
        # the cross-check, not a replacement for it.
        self.declare_parameter("devices_url", "http://127.0.0.1:8000/api/devices/status")
        self.declare_parameter("gripperx_device_id", "gripperx")
        # They report millimetre precision, so a centimetre is a generous
        # "the operator typed the right number".
        self.declare_parameter("length_tolerance_m", 0.01)

        self.declare_parameter("min_length_m", DEFAULT_MIN_LENGTH_M)
        self.declare_parameter("max_length_m", DEFAULT_MAX_LENGTH_M)

        # For the scale cross-check. Defaults match OCTOPUS_HBVCAM_640X480 in
        # live_data.js and manual_height_above_ground_m in the start script; if
        # either moves, pass the new value rather than trusting these.
        self.declare_parameter("fx", 359.3292231592479)
        self.declare_parameter("configured_height_m", 2.5)

        self.input_url = str(self.get_parameter("input_url").value)
        self.status_url = str(self.get_parameter("status_url").value)
        self.request_timeout_sec = float(self.get_parameter("request_timeout_sec").value)
        self.log_period_sec = float(self.get_parameter("log_period_sec").value)
        self.fx = float(self.get_parameter("fx").value)
        self.configured_height_m = float(self.get_parameter("configured_height_m").value)
        self.devices_url = str(self.get_parameter("devices_url").value)
        self.gripperx_device_id = str(self.get_parameter("gripperx_device_id").value)
        self.length_tolerance_m = float(self.get_parameter("length_tolerance_m").value)
        self.min_length_m = float(self.get_parameter("min_length_m").value)
        self.max_length_m = float(self.get_parameter("max_length_m").value)

        self.publisher = self.create_publisher(
            String, str(self.get_parameter("output_topic").value), 10
        )
        self.timer = self.create_timer(
            float(self.get_parameter("publish_period_sec").value), self.tick
        )

        self.last_error_log_time = 0.0
        self.last_signature = None
        self.last_gripperx_signature = None

        self.get_logger().info("Line calibration node started")
        self.get_logger().info(f"Reading operator marks from: {self.input_url}")
        self.get_logger().info(
            f"Scale cross-check against configured height "
            f"{self.configured_height_m:.3f} m, fx {self.fx:.3f}"
        )

    # --- input -------------------------------------------------------------

    def read_backend(self):
        try:
            with urllib.request.urlopen(
                self.input_url, timeout=self.request_timeout_sec
            ) as response:
                payload = json.loads(response.read().decode("utf-8", errors="replace"))
        except Exception as exc:
            self.log_throttled(f"Cannot read line calibration from backend: {exc}")
            return None

        marks = payload.get("line_calibration")
        return marks if isinstance(marks, dict) else None

    def read_gripperx(self):
        """GripperX's own view of the calibration, from its telemetry.

        Read through the backend rather than the ROS graph because that is where
        the robot's status already lands (device_status_backend_bridge_node),
        and because GripperX speaks to us over rosbridge, not DDS.
        """
        try:
            with urllib.request.urlopen(
                self.devices_url, timeout=self.request_timeout_sec
            ) as response:
                payload = json.loads(response.read().decode("utf-8", errors="replace"))
        except Exception:
            # Not an error worth logging every second: no robot connected is the
            # normal state for most of the setup.
            return None

        # Read at call time, not at construction: the robot id is the one
        # parameter an operator plausibly retunes on a running system.
        device_id = str(self.get_parameter("gripperx_device_id").value)
        device = (payload.get("devices") or {}).get(device_id)
        if not isinstance(device, dict):
            return None
        block = device.get("line_calibration")
        return block if isinstance(block, dict) else None

    def read_parameters(self):
        pixel_a = [float(v) for v in self.get_parameter("pixel_a").value]
        pixel_b = [float(v) for v in self.get_parameter("pixel_b").value]
        length_m = float(self.get_parameter("length_m").value)
        if min(pixel_a) < 0.0 or min(pixel_b) < 0.0 or length_m <= 0.0:
            return None
        return {
            "pixel_a": pixel_a,
            "pixel_b": pixel_b,
            "length_m": length_m,
            "mirrored": bool(self.get_parameter("mirrored").value),
            "source": "ros_parameters",
        }

    # --- output ------------------------------------------------------------

    def tick(self):
        marks = self.read_backend() or self.read_parameters()
        payload = self.solve(marks)

        message = String()
        message.data = json.dumps(payload)
        self.publisher.publish(message)
        self.post_status(message.data)

        signature = (payload.get("state"), payload.get("reason"))
        if signature != self.last_signature:
            if payload.get("calibrated"):
                self.get_logger().info(
                    "LINE CALIBRATED  L = {length_m:.3f} m  "
                    "{metres_per_pixel:.6f} m/px  "
                    "implied height {implied_camera_height_m:.3f} m "
                    "(configured {configured:.3f} m, {delta:+.1f} %)".format(
                        configured=self.configured_height_m,
                        delta=payload["scale_deviation_percent"],
                        **{k: payload[k] for k in (
                            "length_m", "metres_per_pixel", "implied_camera_height_m")}
                    )
                )
            elif payload.get("state") == "awaiting_length":
                self.get_logger().info(
                    "Post marks set, geofence available. "
                    "Waiting for L from GripperX before the scale is known."
                )
            else:
                self.get_logger().warn(f"No line calibration: {payload.get('reason')}")
            self.last_signature = signature

        gripperx = payload.get("gripperx") or {}
        mismatch_signature = (gripperx.get("id"), gripperx.get("length_match"))
        if gripperx.get("length_match") is False and mismatch_signature != self.last_gripperx_signature:
            self.get_logger().warn(
                "L disagrees with GripperX: entered {ours:.3f} m, they measured "
                "{theirs:.3f} m (delta {delta:+.3f} m, calibration {cid}). "
                "Re-enter L or re-click on their side - the two frames are not "
                "the same size.".format(
                    ours=payload.get("length_m", float("nan")),
                    theirs=gripperx.get("length_m", float("nan")),
                    delta=gripperx.get("length_delta_m", float("nan")),
                    cid=gripperx.get("id"),
                )
            )
        self.last_gripperx_signature = mismatch_signature

    def solve(self, marks):
        base = {
            "source_id": "line_calibration_node",
            "frame_id": "octopus_line",
            "timestamp": time.time(),
            "spec": "OCTOPUS_LINE_CALIBRATION.md draft 2026-09-24, sections 1-5",
            # Stated plainly so nobody reads this topic as proof the pipeline
            # uses the line. It does not - see the module docstring.
            "applied_to_projection": False,
            "configured_height_m": self.configured_height_m,
            "fx": self.fx,
            "min_length_m": self.min_length_m,
            "max_length_m": self.max_length_m,
        }

        if not marks:
            base.update({
                "calibrated": False,
                "marks_set": False,
                "state": "not_calibrated",
                "reason": "no post marks from the dashboard or parameters yet",
            })
            return base

        # The marks travel even when L does not. They alone fix the origin, the
        # direction and the demo-area square, and flight_camera_transform_node
        # builds the geofence from them; L only adds the metric scale. Waiting
        # for GripperX to read out L would hold up the geofence for no reason.
        base["marks_set"] = True
        base["pixel_a"] = [float(v) for v in marks.get("pixel_a", [])] or None
        base["pixel_b"] = [float(v) for v in marks.get("pixel_b", [])] or None
        base["mirrored"] = bool(marks.get("mirrored", False))
        base["input_source"] = marks.get("source", "backend")

        if not float(marks.get("length_m") or 0.0) > 0.0:
            base.update({
                "calibrated": False,
                "state": "awaiting_length",
                "reason": "marks set, waiting for L from GripperX's LiDAR",
            })
            return base

        try:
            calibration = LineCalibration(
                marks["pixel_a"],
                marks["pixel_b"],
                marks["length_m"],
                mirrored=bool(marks.get("mirrored", False)),
                min_length_m=self.min_length_m,
                max_length_m=self.max_length_m,
            )
        except (LineCalibrationError, KeyError, TypeError, ValueError) as exc:
            base.update({
                "calibrated": False,
                "state": "rejected",
                "reason": str(exc),
            })
            return base

        implied = calibration.implied_camera_height_m(self.fx)
        base.update(calibration.as_dict())
        base["calibration_id"] = self.calibration_id(calibration)
        base.update({
            "state": "calibrated",
            "reason": None,
            "input_source": marks.get("source", "backend"),
            "implied_camera_height_m": implied,
            # Positive means the line says the camera hangs higher than the
            # projection assumes, so the pipeline currently reports distances
            # that are too short by this fraction.
            "scale_deviation_percent": (implied / self.configured_height_m - 1.0) * 100.0,
        })
        base["gripperx"] = self.compare_with_gripperx(calibration.length_m)
        return base

    @staticmethod
    def calibration_id(calibration):
        """A short id that changes whenever the frame changes.

        Section 8 lists "Octopus-side recalibration is not observable by
        GripperX" as an open point for both teams. This is our half of it: the
        id rides on /octopus/line_calibration/status, which is under /octopus/*
        and therefore already inside their rosbridge glob - so they can detect a
        new frame without us inventing a signalling channel.
        """
        seed = json.dumps([
            calibration.pixel_a, calibration.pixel_b,
            round(calibration.length_m, 6), calibration.mirrored,
        ], sort_keys=True)
        return hashlib.sha1(seed.encode("utf-8")).hexdigest()[:12]

    def compare_with_gripperx(self, our_length_m):
        """Did the operator type in the L that GripperX actually measured?"""
        block = self.read_gripperx()
        if block is None:
            return {"reported": False, "reason": "no line_calibration in GripperX telemetry"}

        status = block.get("status")
        their_length = block.get("length_m")
        result = {
            "reported": True,
            "status": status,
            "reason": block.get("reason"),
            "id": block.get("id"),
            "length_m": their_length,
        }

        if status != "ok" or not isinstance(their_length, (int, float)):
            result["length_match"] = None
            return result

        delta = float(our_length_m) - float(their_length)
        result["length_delta_m"] = delta
        result["length_match"] = abs(delta) <= self.length_tolerance_m
        return result

    def post_status(self, body):
        try:
            request = urllib.request.Request(
                self.status_url,
                data=body.encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(request, timeout=self.request_timeout_sec) as response:
                response.read()
        except Exception as exc:
            self.log_throttled(f"Failed to post line calibration status: {exc}")

    def log_throttled(self, message):
        now = time.time()
        if now - self.last_error_log_time >= self.log_period_sec:
            self.get_logger().warn(message)
            self.last_error_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = LineCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
