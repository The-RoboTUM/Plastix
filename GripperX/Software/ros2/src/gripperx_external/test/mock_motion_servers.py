#!/usr/bin/env python3
"""The world the gateway dispatches into, mocked. Twin domain only.

    ROS_DOMAIN_ID=221 python3 src/gripperx_external/test/mock_motion_servers.py

WHAT THIS IS FOR
================
Stage 3's acceptance (SAFETY.md C-8) asks for the auto-disarm triggers exercised
**with a goal in flight** - the seven of SR-15 rule 7 that do not need a
misbehaving clock; the two that do, `CLOCK_STALLED` (8) and `CLOCK_JUMPED_BACK`
(9), are exercised in the `clock` scenario - and for the SR-9 publisher diff
taken in the **armed** state. Neither was possible before stage 3 existed, and neither is
possible against a Nav2 that reaches its goal in two seconds: a trigger has to
arrive while something is genuinely running. This node provides a
``NavigateToPose`` that takes as long as it is told to, a ``PickPlastic`` that
succeeds or fails on command, and the four observations the validation pipeline
needs (TF, odometry, costmap, teleop mode).

WHAT IT IS NOT
==============
It is **not** a simulator and it makes no claim about Nav2's real behaviour.
Timings, path quality and arrival accuracy here are fixtures, not measurements,
and nothing measured against this file may be described as measured against
Nav2. What it does verify is the gateway's DECISION LOGIC, which is exactly what
FR-12 acceptance item 5 asks stage 3 to verify in the twin.

IT MOVES NOTHING, AND IT CANNOT
===============================
There is no hardware behind it and it publishes on no topic of the motion
command chain. ``/teleop/active_mode`` is the one chain-adjacent topic it
writes, and that is the mux's *output* - the state the gateway OBSERVES - never
``/teleop/set_mode``. It refuses to start on the real robot's domain: this file
exists to be driven by kill-based tests, and a kill-based test near domain 20 is
not a test anyone should be able to start by accident.

CONTROL
=======
Everything is a ROS parameter, so a scenario is driven with ``ros2 param set``:

    nav_outcome        succeed | abort | reject | hang
    nav_duration_sec   how long a goal stays in flight
    nav_available      false destroys the action server (NAV2_UNAVAILABLE)
    pick_outcome       succeed | fail | reject
    pick_available     false destroys the pick server
    teleop_mode        the string published; empty STOPS publishing (dead mux)
    teleport_on_arrival place the published TF at the goal pose on success
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from gripperx_arm_msgs.action import PickPlastic
from nav2_msgs.action import NavigateToPose

#: The real robot's domain. This file must never run there - see the docstring.
FORBIDDEN_DOMAIN = 20


def _latched(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class MockMotionServers(Node):
    def __init__(self) -> None:
        super().__init__("mock_motion_servers")
        self.declare_parameter("nav_outcome", "succeed")
        self.declare_parameter("nav_duration_sec", 30.0)
        self.declare_parameter("nav_available", True)
        self.declare_parameter("pick_outcome", "succeed")
        self.declare_parameter("pick_available", True)
        self.declare_parameter("pick_duration_sec", 1.0)
        self.declare_parameter("teleop_mode", "autonomous")
        self.declare_parameter("teleport_on_arrival", True)
        self.declare_parameter("robot_x", 0.0)
        self.declare_parameter("robot_y", 0.0)
        self.declare_parameter("robot_yaw", 0.0)

        self._group = ReentrantCallbackGroup()
        self._tf = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self._mode_pub = self.create_publisher(String, "/teleop/active_mode", 10)
        self._costmap_pub = self.create_publisher(
            OccupancyGrid, "/global_costmap/costmap", _latched()
        )

        self._nav_server: Optional[ActionServer] = None
        self._pick_server: Optional[ActionServer] = None
        self._nav_goals = 0
        self._nav_cancels = 0
        self._pick_goals = 0
        self._lock = threading.Lock()

        self._sync_servers()
        self._publish_costmap()
        # 20 Hz, like teleop_mux: the gateway's mode-age check only means
        # something against a producer that republishes rather than latches.
        self.create_timer(0.05, self._tick, callback_group=self._group)
        self.create_timer(0.1, self._publish_odom, callback_group=self._group)
        self.create_timer(1.0, self._sync_servers, callback_group=self._group)
        self.get_logger().info("mock motion servers up (nothing here can move anything)")

    # -- server lifecycle, so availability itself can be a scenario -------
    def _sync_servers(self) -> None:
        want_nav = bool(self.get_parameter("nav_available").value)
        if want_nav and self._nav_server is None:
            self._nav_server = ActionServer(
                self,
                NavigateToPose,
                "/navigate_to_pose",
                execute_callback=self._execute_nav,
                goal_callback=self._accept_nav,
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=self._group,
            )
            self.get_logger().info("navigate_to_pose server CREATED")
        elif not want_nav and self._nav_server is not None:
            self._nav_server.destroy()
            self._nav_server = None
            self.get_logger().warn("navigate_to_pose server DESTROYED")

        want_pick = bool(self.get_parameter("pick_available").value)
        if want_pick and self._pick_server is None:
            self._pick_server = ActionServer(
                self,
                PickPlastic,
                "/pick_plastic",
                execute_callback=self._execute_pick,
                goal_callback=lambda _: GoalResponse.ACCEPT
                if str(self.get_parameter("pick_outcome").value) != "reject"
                else GoalResponse.REJECT,
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=self._group,
            )
            self.get_logger().info("pick_plastic server CREATED")
        elif not want_pick and self._pick_server is not None:
            self._pick_server.destroy()
            self._pick_server = None
            self.get_logger().warn("pick_plastic server DESTROYED")

    # -- observations the validation pipeline needs -----------------------
    def _tick(self) -> None:
        stamp = self.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "map"
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x = float(self.get_parameter("robot_x").value)
        tf.transform.translation.y = float(self.get_parameter("robot_y").value)
        yaw = float(self.get_parameter("robot_yaw").value)
        tf.transform.rotation.z = math.sin(yaw * 0.5)
        tf.transform.rotation.w = math.cos(yaw * 0.5)
        self._tf.sendTransform(tf)

        mode = str(self.get_parameter("teleop_mode").value)
        if mode:
            # Empty means the mux DIED: no publication at all, which is what the
            # gateway's mode-age check exists to catch (SAFETY.md F-9).
            msg = String()
            msg.data = mode
            self._mode_pub.publish(msg)

    def _publish_odom(self) -> None:
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        self._odom_pub.publish(odom)

    def _publish_costmap(self) -> None:
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"
        grid.info.resolution = 0.1
        grid.info.width = 200
        grid.info.height = 200
        grid.info.origin.position.x = -10.0
        grid.info.origin.position.y = -10.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (200 * 200)
        self._costmap_pub.publish(grid)

    # -- NavigateToPose ---------------------------------------------------
    def _accept_nav(self, _goal) -> GoalResponse:
        if str(self.get_parameter("nav_outcome").value) == "reject":
            self.get_logger().warn("rejecting the goal (nav_outcome=reject)")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute_nav(self, goal_handle):
        with self._lock:
            self._nav_goals += 1
        target = goal_handle.request.pose.pose.position
        duration = float(self.get_parameter("nav_duration_sec").value)
        outcome = str(self.get_parameter("nav_outcome").value)
        self.get_logger().info(
            f"navigating to ({target.x:.3f}, {target.y:.3f}) for {duration:.1f}s "
            f"[{outcome}]"
        )
        deadline = time.time() + duration
        while time.time() < deadline:
            if goal_handle.is_cancel_requested:
                with self._lock:
                    self._nav_cancels += 1
                goal_handle.canceled()
                self.get_logger().warn("navigation CANCELED on request")
                return NavigateToPose.Result()
            if outcome == "hang":
                # Never finishes and never notices the deadline: the "server is
                # alive but the goal is stuck" case.
                deadline = time.time() + 1.0
            time.sleep(0.05)

        if outcome == "abort":
            goal_handle.abort()
            self.get_logger().warn("navigation ABORTED")
            return NavigateToPose.Result()

        if bool(self.get_parameter("teleport_on_arrival").value):
            # Arrive exactly on the pose. A real Nav2 stops within
            # xy_goal_tolerance / yaw_goal_tolerance; this fixture removes that
            # variable so the reached check is testing the gateway, not the mock.
            quat = goal_handle.request.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (quat.w * quat.z), 1.0 - 2.0 * (quat.z * quat.z)
            )
            self.set_parameters(
                [
                    rclpy.parameter.Parameter("robot_x", value=float(target.x)),
                    rclpy.parameter.Parameter("robot_y", value=float(target.y)),
                    rclpy.parameter.Parameter("robot_yaw", value=float(yaw)),
                ]
            )
        goal_handle.succeed()
        self.get_logger().info("navigation SUCCEEDED")
        return NavigateToPose.Result()

    # -- PickPlastic ------------------------------------------------------
    def _execute_pick(self, goal_handle):
        with self._lock:
            self._pick_goals += 1
        outcome = str(self.get_parameter("pick_outcome").value)
        deadline = time.time() + float(self.get_parameter("pick_duration_sec").value)
        while time.time() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn("pick CANCELED on request")
                return PickPlastic.Result()
            time.sleep(0.05)
        result = PickPlastic.Result()
        if outcome == "succeed":
            goal_handle.succeed()
            result.success = True
            result.message = "mock pick sequence complete"
        else:
            goal_handle.abort()
            result.success = False
            result.message = "mock pick failed on purpose"
        self.get_logger().info(f"pick finished: success={result.success}")
        return result


def main() -> int:
    domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    if domain == FORBIDDEN_DOMAIN:
        print(
            f"refusing to start on ROS_DOMAIN_ID={FORBIDDEN_DOMAIN}: that is the "
            "real robot. This file is driven by kill-based tests.",
            file=sys.stderr,
        )
        return 2
    rclpy.init()
    node = MockMotionServers()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
