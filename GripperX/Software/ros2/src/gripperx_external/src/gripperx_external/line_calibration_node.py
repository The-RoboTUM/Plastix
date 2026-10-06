#!/usr/bin/env python3
"""Two clicks in RViz -> the shared Octopus reference line, as ``map -> octopus_line``.

WHAT THE OPERATOR DOES
======================
At every start of the stack: in RViz (fixed frame ``map``) take the "Publish
Point" tool, click the scan-point cluster of post A, then of post B. The node
computes the line frame (convention: :mod:`line_frame`), logs the measured
length L to the millimetre - the number the operator types in on the drone
side - and publishes it. A pair whose L lies outside ``expected_length_min_m``
.. ``expected_length_max_m`` (user-stated post spacing, in the config) is
REFUSED and shown in red; the operator re-clicks. A third click starts a new
pair (recalibration); the ``reset`` service discards everything.

WHAT IT PUBLISHES (relative to its namespace, ``/gripperx/external``)
====================================================================
* ``line_calibration``          ``std_msgs/String`` JSON, latched. The status
  the gateway gates on - built and parsed by :mod:`line_frame` only.
* ``line_calibration_markers``  ``MarkerArray``, latched. Posts A/B, the line,
  an axis triad at the origin, the text "L = x.xxx m".
* ``/tf``                       ``map -> octopus_line`` while calibrated, at
  ``tf_rate_hz``. DELIBERATELY NOT ``/tf_static``: a static transform cannot be
  withdrawn - it stays latched in every listener's buffer for the life of that
  listener - and this transform has a lifetime (the SLAM session, the next
  reset). A periodic ``/tf`` stops when the calibration dies, so a stale line
  frame times out in RViz instead of lingering as if it were still true. The
  gateway does NOT read this transform; it reads the status topic, which
  carries the identity and the reason a TF cannot express.

WHY THE CALIBRATION DIES WITH THE MAP SESSION
=============================================
The clicks are coordinates in slam_toolbox's ``map`` frame, and a restarted
slam_toolbox starts a NEW map frame (anchored wherever the robot stands). The
first line of defence is WHERE this node runs: it is launched by
``gripperx_localization/launch/localization.launch.py`` next to slam_toolbox,
so restarting the mapping service restarts it too. Within one launch,
slam_toolbox can still die or be replaced while this node lives on, so it also
records the DDS endpoint GIDs (global identifiers) of the map topic's
publishers when the pair is clicked and invalidates the calibration when that
set changes (a new process has new GIDs) or stays empty longer than
``map_session_grace_sec``. The gateway, in another service, runs the same check
independently. See ``line_frame.map_session_verdict`` for the limits.

IT MOVES NOTHING. It has no action client, no service client and no publisher
on the motion command chain - the same post-construction sweeps as the other
two nodes of this package prove it at startup.
"""

from __future__ import annotations

import math
import time
import uuid
from typing import List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point, PointStamped, TransformStamped
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from . import line_frame as lf
from .domain_guard import enforce_domain
from .grasp import parse_measured_param
from .octopus_link_node import (
    assert_no_chain_publishers,
    assert_no_command_clients,
    guarded_publisher,
)


def _latched(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _colour(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


class LineCalibrationNode(Node):
    def __init__(self, **node_kwargs) -> None:
        # `node_kwargs` (e.g. parameter_overrides) exist for the checks, which
        # build this node with the values read from the config files.
        super().__init__("line_calibration_node", **node_kwargs)

        self.declare_parameter("expected_domain_id", -1)
        # `line_frame_id`, `map_topic`, `expected_length_*` and
        # `map_session_grace_sec` come from the `/**` section of the config,
        # which the gateway loads too - one source for both (audit H1).
        self.declare_parameter("line_frame_id", lf.DEFAULT_FRAME_ID)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        # The topic whose publishers define the map session. slam_toolbox's
        # occupancy grid: its publisher lives exactly as long as the map frame.
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_session_check_period_sec", 1.0)
        # How long the map topic may show NO publisher before the calibration
        # is dropped (audit M2). `TO-VERIFY`: while unmeasured, an empty read
        # drops it at once - the fail-closed reading. A DIFFERENT publisher set
        # drops it at once whatever this says.
        self.declare_parameter(
            "map_session_grace_sec",
            "TO-VERIFY",
            ParameterDescriptor(
                dynamic_typing=True,
                description="Seconds, or 'TO-VERIFY' (then an empty read drops at once).",
            ),
        )
        self.declare_parameter("tf_rate_hz", 10.0)
        # PLAUSIBILITY RANGE for |AB|, inclusive: a pair outside it is a wrong
        # post or a stray click and is REFUSED. The values live in the config
        # (user-stated range, not a measurement). The defaults are NaN on
        # purpose - no number is invented here, and a node started without the
        # config refuses every pair with NO_LENGTH_RANGE instead of accepting
        # anything.
        for name in ("expected_length_min_m", "expected_length_max_m"):
            self.declare_parameter(
                name,
                float("nan"),
                ParameterDescriptor(
                    description=(
                        "Plausibility bound on the post spacing |AB| in metres "
                        "(user-stated). Unset (NaN) refuses every pair."
                    )
                ),
            )

        enforce_domain(self, int(self.get_parameter("expected_domain_id").value))

        self._frame_id = str(self.get_parameter("line_frame_id").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._map_topic = str(self.get_parameter("map_topic").value)
        self._map_grace_sec = parse_measured_param(
            self.get_parameter("map_session_grace_sec").value
        )
        self._map_empty_since: Optional[float] = None
        #: The identity of THIS process. The calibration counter restarts with
        #: the process; the pair (session, id) does not repeat.
        self._session = uuid.uuid4().hex[:12]
        self._count = 0
        self._calibration: Optional[lf.LineCalibration] = None
        self._pending_a: Optional[Tuple[float, float]] = None
        #: The last refused pair, shown in RViz until the next click.
        self._rejected: Optional[dict] = None
        self._state = lf.STATE_WAITING_FOR_A
        self._reason = lf.NOT_CALIBRATED_YET
        self._detail = "click post A, then post B, with RViz 'Publish Point'"

        self._status_pub = guarded_publisher(self, String, "line_calibration", _latched())
        self._marker_pub = guarded_publisher(
            self, MarkerArray, "line_calibration_markers", _latched()
        )
        self._tf_pub = guarded_publisher(
            self,
            TFMessage,
            "/tf",
            QoSProfile(
                depth=100,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("clicked_point_topic").value),
            self._on_clicked_point,
            10,
        )
        self.create_service(Trigger, "~/reset", self._on_reset)
        self.create_timer(
            max(0.1, float(self.get_parameter("map_session_check_period_sec").value)),
            self._check_map_session,
        )
        self.create_timer(
            1.0 / max(1.0, float(self.get_parameter("tf_rate_hz").value)), self._publish_tf
        )

        assert_no_chain_publishers(self)
        assert_no_command_clients(self, ())

        self._publish()
        self.get_logger().info(
            f"line calibration up (session {self._session}): waiting for post A. "
            f"Clicks are taken in '{self._map_frame}' from "
            f"{self.get_parameter('clicked_point_topic').value}; the frame "
            f"'{self._frame_id}' dies with the '{self._map_topic}' publisher it "
            "was clicked against. NOTHING is persisted - recalibrate at every start."
        )
        low, high = self._length_range()
        if not (math.isfinite(low) and math.isfinite(high) and 0.0 < low <= high):
            self.get_logger().error(
                f"expected_length_min_m/max_m = {low}/{high} is not a usable range: "
                "EVERY clicked pair will be refused (NO_LENGTH_RANGE). Set both in "
                "the config."
            )
        else:
            self.get_logger().info(
                f"a pair is accepted only if L is within {low:.3f}-{high:.3f} m "
                "(user-stated post spacing, a plausibility bound)"
            )

    # ------------------------------------------------------------------
    def _length_range(self) -> Tuple[float, float]:
        return (
            float(self.get_parameter("expected_length_min_m").value),
            float(self.get_parameter("expected_length_max_m").value),
        )

    def _current_map_session(self) -> Tuple[str, ...]:
        infos = self.get_publishers_info_by_topic(self._map_topic)
        return lf.map_session_key([info.endpoint_gid for info in infos])

    def _invalidate(self, reason: str, detail: str, severity: str = "warn") -> None:
        had = self._calibration
        self._calibration = None
        self._state = lf.STATE_WAITING_FOR_A if self._pending_a is None else lf.STATE_WAITING_FOR_B
        self._reason = reason
        self._detail = detail
        if had is not None:
            message = (
                f"LINE CALIBRATION #{had.calibration_id} DISCARDED ({reason}): {detail}. "
                "Dispatch is refused until posts A and B are clicked again."
            )
            if severity == "error":
                self.get_logger().error(message)
            else:
                self.get_logger().warn(message)
        self._publish()

    # -- inputs ---------------------------------------------------------
    def _on_clicked_point(self, msg: PointStamped) -> None:
        frame = msg.header.frame_id.lstrip("/")
        if frame != self._map_frame:
            self.get_logger().error(
                f"click ignored: it is in frame '{msg.header.frame_id}', the posts "
                f"must be clicked in '{self._map_frame}'. Set RViz's fixed frame "
                f"to '{self._map_frame}'."
            )
            self._detail = f"last click was in '{msg.header.frame_id}', not '{self._map_frame}'"
            self._publish()
            return
        x, y, z = float(msg.point.x), float(msg.point.y), float(msg.point.z)
        if not (math.isfinite(x) and math.isfinite(y)):
            self.get_logger().error(f"click ignored: non-finite point ({x}, {y})")
            return

        if self._pending_a is None:
            self._pending_a = (x, y)
            self._rejected = None
            if self._calibration is not None:
                # A new pair has begun. Keeping the old calibration live while
                # the operator is visibly replacing it would leave the robot
                # driving on a line its operator has just declared wrong.
                self._invalidate(
                    lf.RECALIBRATING,
                    f"post A re-clicked at ({x:.3f}, {y:.3f}); click post B to finish",
                )
            else:
                self._state = lf.STATE_WAITING_FOR_B
                self._reason = lf.NOT_CALIBRATED_YET
                self._detail = f"post A at ({x:.3f}, {y:.3f}); click post B"
                self._publish()
            self.get_logger().info(
                f"post A = ({x:.3f}, {y:.3f}) in '{self._map_frame}' "
                f"(clicked z = {z:.3f} m, ignored - the line frame has z = 0). "
                "Now click post B."
            )
            return

        a = self._pending_a
        self._pending_a = None
        self.get_logger().info(
            f"post B = ({x:.3f}, {y:.3f}) in '{self._map_frame}' "
            f"(clicked z = {z:.3f} m, ignored)"
        )
        map_session = self._current_map_session()
        if not map_session:
            self._state = lf.STATE_WAITING_FOR_A
            self._reason = lf.NO_MAP_SESSION
            self._detail = (
                f"no publisher on {self._map_topic}: without it the map frame's "
                "lifetime cannot be tracked, so no calibration is made. Click A "
                "and B again once SLAM is running."
            )
            self.get_logger().error(f"calibration REFUSED: {self._detail}")
            self._publish()
            return
        try:
            cal = lf.compute_calibration(
                a,
                (x, y),
                self._length_range(),
                frame_id=self._frame_id,
                parent_frame_id=self._map_frame,
                session=self._session,
                calibration_id=self._count + 1,
                stamp_sec=time.time(),
                map_session=map_session,
            )
        except lf.LineCalibrationError as exc:
            self._rejected = {
                "a": {"x": a[0], "y": a[1]},
                "b": {"x": x, "y": y},
                "length_m": math.hypot(x - a[0], y - a[1]),
                "reason": exc.reason,
            }
            self._state = lf.STATE_WAITING_FOR_A
            self._reason = exc.reason
            self._detail = f"{exc.detail}. Click post A again."
            if exc.reason == lf.LENGTH_OUT_OF_RANGE:
                # The same string as the red RViz text (audit L6).
                self.get_logger().error(
                    lf.rejection_text(exc.reason, self._rejected["length_m"], self._length_range())
                )
            else:
                self.get_logger().error(f"calibration REFUSED ({exc.reason}): {exc.detail}")
            self._publish()
            return

        self._count += 1
        self._calibration = cal
        self._state = lf.STATE_CALIBRATED
        self._reason = ""
        self._detail = ""
        self.get_logger().warn(
            "LINE CALIBRATED - " + lf.describe(cal) + ". "
            f"ENTER L = {cal.length_m:.3f} m ON THE DRONE SIDE."
        )
        self._publish()

    def _on_reset(self, _request: Trigger.Request, response: Trigger.Response):
        had = self._calibration
        self._pending_a = None
        self._invalidate(lf.RESET, "operator reset")
        response.success = True
        response.message = (
            f"discarded calibration #{had.calibration_id}" if had is not None
            else "nothing was calibrated"
        ) + "; click post A, then post B"
        self.get_logger().info(f"reset: {response.message}")
        return response

    def _check_map_session(self) -> None:
        current = self._current_map_session()
        mono = time.monotonic()
        if current:
            self._map_empty_since = None
        elif self._map_empty_since is None:
            self._map_empty_since = mono
        cal = self._calibration
        if cal is None:
            return
        verdict = lf.map_session_verdict(cal.map_session, current)
        if verdict == lf.MAP_SESSION_CHANGED:
            self._invalidate(
                verdict,
                f"a DIFFERENT publisher is on {self._map_topic} than when the posts "
                "were clicked - slam_toolbox restarted, so the map frame the clicks "
                "were taken in no longer exists",
                severity="error",
            )
        elif verdict == lf.NO_MAP_SESSION:
            empty_for = mono - (self._map_empty_since or mono)
            if self._map_grace_sec is not None and empty_for <= self._map_grace_sec:
                return
            # Fail closed while the grace is unmeasured: one empty read is a
            # loss. This node saw the publisher when the pair was clicked, so
            # it is not startup discovery lag.
            self._invalidate(
                lf.MAP_SESSION_LOST,
                f"no publisher visible on {self._map_topic} for {empty_for:.1f}s "
                + (
                    "(map_session_grace_sec is TO-VERIFY, so at once)"
                    if self._map_grace_sec is None
                    else f"(more than map_session_grace_sec={self._map_grace_sec:.1f}s)"
                )
                + " - the map frame the clicks were taken in is not shown to exist",
                severity="error",
            )

    # -- outputs --------------------------------------------------------
    def _publish(self) -> None:
        msg = String()
        msg.data = lf.build_status(
            state=self._state,
            session=self._session,
            frame_id=self._frame_id,
            parent_frame_id=self._map_frame,
            stamp_sec=time.time(),
            calibration=self._calibration,
            reason=self._reason,
            detail=self._detail,
            pending_a=self._pending_a,
            calibration_count=self._count,
            expected_length_m=self._length_range(),
            rejected=self._rejected,
            map_topic=self._map_topic,
        )
        self._status_pub.publish(msg)
        self._marker_pub.publish(self._markers())

    def _publish_tf(self) -> None:
        cal = self._calibration
        if cal is None:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._map_frame
        t.child_frame_id = self._frame_id
        t.transform.translation.x, t.transform.translation.y = cal.origin
        t.transform.rotation.z = math.sin(cal.yaw_rad * 0.5)
        t.transform.rotation.w = math.cos(cal.yaw_rad * 0.5)
        self._tf_pub.publish(TFMessage(transforms=[t]))

    def _marker(self, ns: str, marker_id: int, kind: int) -> Marker:
        m = Marker()
        m.header.frame_id = self._map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = marker_id
        m.type = kind
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _post_markers(self, label: str, x: float, y: float, index: int) -> List[Marker]:
        post = self._marker("line_cal/posts", index, Marker.CYLINDER)
        post.pose.position.x, post.pose.position.y, post.pose.position.z = x, y, 0.15
        post.scale.x = post.scale.y = 0.06
        post.scale.z = 0.3
        post.color = _colour(1.0, 0.55, 0.0)
        text = self._marker("line_cal/post_labels", index, Marker.TEXT_VIEW_FACING)
        text.pose.position.x, text.pose.position.y, text.pose.position.z = x, y, 0.45
        text.scale.z = 0.25
        text.color = _colour(1.0, 1.0, 1.0)
        text.text = label
        return [post, text]

    def _markers(self) -> MarkerArray:
        out = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        out.markers.append(clear)
        if self._pending_a is not None:
            out.markers.extend(self._post_markers("A", *self._pending_a, index=0))
        rej = self._rejected
        if rej is not None:
            ra, rb = rej["a"], rej["b"]
            line = self._marker("line_cal/rejected", 0, Marker.LINE_STRIP)
            line.scale.x = 0.02
            line.color = _colour(1.0, 0.1, 0.1)
            line.points = [Point(x=ra["x"], y=ra["y"], z=0.0), Point(x=rb["x"], y=rb["y"], z=0.0)]
            out.markers.append(line)
            low, high = self._length_range()
            text = self._marker("line_cal/rejected", 1, Marker.TEXT_VIEW_FACING)
            text.pose.position.x = 0.5 * (ra["x"] + rb["x"])
            text.pose.position.y = 0.5 * (ra["y"] + rb["y"])
            text.pose.position.z = 0.3
            text.scale.z = 0.25
            text.color = _colour(1.0, 0.2, 0.2)
            text.text = lf.rejection_text(rej["reason"], rej["length_m"], (low, high))
            out.markers.append(text)
        cal = self._calibration
        if cal is None:
            return out
        if self._pending_a is None:
            out.markers.extend(self._post_markers("A", cal.ax, cal.ay, index=0))
        out.markers.extend(self._post_markers("B", cal.bx, cal.by, index=1))

        line = self._marker("line_cal/line", 0, Marker.LINE_STRIP)
        line.scale.x = 0.02
        line.color = _colour(1.0, 0.55, 0.0)
        line.points = [Point(x=cal.ax, y=cal.ay, z=0.0), Point(x=cal.bx, y=cal.by, z=0.0)]
        out.markers.append(line)

        # The geofence: the square of side L around the midpoint, from the
        # SAME helper the validation uses, so what is drawn is what is gated.
        fence = self._marker("line_cal/geofence", 0, Marker.LINE_STRIP)
        fence.scale.x = 0.015
        fence.color = _colour(0.3, 0.8, 1.0, 0.8)
        fence.points = [Point(x=x, y=y, z=0.0) for x, y in lf.geofence_corners_map(cal)]
        out.markers.append(fence)

        ox, oy = cal.origin
        # The triad is drawn from the transform itself (line_to_map of the unit
        # vectors), so what the operator sees IS the convention the gateway uses.
        for index, ((lx, ly, lz), colour) in enumerate(
            (((0.5, 0.0, 0.0), (1, 0, 0)), ((0.0, 0.5, 0.0), (0, 1, 0)), ((0.0, 0.0, 0.5), (0, 0, 1)))
        ):
            arrow = self._marker("line_cal/axes", index, Marker.ARROW)
            tip_x, tip_y = cal.line_to_map(lx, ly)
            arrow.points = [Point(x=ox, y=oy, z=0.0), Point(x=tip_x, y=tip_y, z=lz)]
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.03, 0.06, 0.08
            arrow.color = _colour(*colour)
            out.markers.append(arrow)

        text = self._marker("line_cal/length", 0, Marker.TEXT_VIEW_FACING)
        tx, ty = cal.line_to_map(0.0, -0.35)
        text.pose.position.x, text.pose.position.y, text.pose.position.z = tx, ty, 0.3
        text.scale.z = 0.3
        text.color = _colour(1.0, 1.0, 0.2)
        text.text = f"L = {cal.length_m:.3f} m  (#{cal.calibration_id})"
        out.markers.append(text)
        return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = LineCalibrationNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except SystemExit as exc:
        # The SR-8 domain guard exits this way; the code must survive so a
        # supervisor can tell a refused start from a clean one.
        code = int(exc.code) if isinstance(exc.code, int) else 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if code:
        raise SystemExit(code)


if __name__ == "__main__":
    main()
