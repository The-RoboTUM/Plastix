// Copyright (c) 2026 GripperX
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GRIPPERX_BEHAVIORS__CRAB_WALK_HPP_
#define GRIPPERX_BEHAVIORS__CRAB_WALK_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"

namespace gripperx_behaviors
{

/**
 * @class gripperx_behaviors::CrabWalk
 * @brief Bounded PURE LATERAL recovery for the 4WIS/4WID GripperX chassis.
 *
 * Nav2 has no lateral behavior: nav2_behaviors::DriveOnHeading rejects any goal
 * with target.y != 0 ("DrivingOnHeading in Y and Z not supported"), and its
 * collision check simulates strictly along the robot's heading
 * (cos/sin of theta times linear.x), so it could not have been reused by simply
 * feeding it a sideways velocity even if the input check were relaxed.
 *
 * WHY EXACTLY 90 deg AND NOT "SOMEWHERE SIDEWAYS". For a pure translation every
 * point of the body moves in the same direction, so every wheel must stand
 * perpendicular to it; the lateral lever arm between king pin and tyre changes
 * nothing about that. 90 deg is the only solution, not a choice. Each wheel
 * reaches it through the +-180 deg module equivalence (a wheel at -90 deg spun
 * backwards is the same motion as +90 deg spun forwards), which leaves 10 deg
 * to the 100 deg outward stop and nothing at all towards the 35 deg inward stop
 * (gripperx_control/config/steer_servo.yaml, FL [-100,+35] FR [-35,+100]
 * BL [-35,+100] BR [-100,+35] deg). Any vx or wz mixed in moves the required
 * angle off 90 deg and eats into that 10 deg, so this behavior commands vy
 * alone. Measured on the twin over 16 runs: the joints settle at -+89.95..89.99
 * deg, outward on all four.
 *
 * THE COLLISION GUARD IS STRONGER HERE THAN FOR THE REVERSE. Sideways the LD06
 * sees live (its blind wedge is +-20 deg around -x, at the rear), so costmap
 * memory is a second line rather than the only one. Demonstrated at a pose the
 * robot had just been teleported to: nav2_behaviors::BackUp refused with
 * COLLISION_AHEAD because the ground behind was never observed, while this
 * behavior executed 0.21 m in the same instant.
 *
 * WHAT IS WEAKER IS THE STATE ESTIMATE, and it is small but real. A crab leaves
 * a heading error the wheel odometry cannot observe at all, because the pure
 * lateral IK solution has omega = 0 by construction — /wheel/odom reported
 * exactly 0.0 deg of yaw change in every measured run. Ground truth over 16 runs
 * of the configured 0.20 m at 0.10 m/s: -0.0062 rad mean leftward, +0.0089 rad
 * mean rightward, 0.0138 rad worst case. It follows the direction of travel, so
 * it is a bias and it accumulates over repeated same-side recoveries. Nearly all
 * of it is produced by the two steering transients rather than by the lateral
 * roll itself, which is why the distance limit is NOT derived from it — the full
 * argument and the numbers are in config/nav2.yaml at behavior_server.
 *
 * The action type is nav2_msgs/action/DriveOnHeading, deliberately reused
 * instead of defining a new one, so that the stock nav2_behavior_tree
 * DriveOnHeading BT node can drive this server through its `server_name` port
 * without a custom BT plugin (which would force bt_navigator.plugin_lib_names
 * to be overridden with a full copy of the upstream default list). The goal's
 * `target` is read as a displacement in base_footprint with the crab convention
 * documented at onRun().
 */
class CrabWalk : public nav2_behaviors::TimedBehavior<nav2_msgs::action::DriveOnHeading>
{
  using ActionT = nav2_msgs::action::DriveOnHeading;
  using CostmapInfoType = nav2_core::CostmapInfoType;

public:
  CrabWalk();
  ~CrabWalk() override = default;

  /**
   * @brief Validate and latch the goal.
   *
   * Goal convention (see the class comment for why the action type is reused):
   *   - target.z must be 0.
   *   - EXACTLY ONE of target.x / target.y may be non-zero. target.y is the
   *     native field (signed lateral displacement, + = left = +y in
   *     base_footprint). target.x is accepted as the same quantity because the
   *     stock DriveOnHeading BT node can only populate x; on a server whose
   *     every motion is lateral there is no second axis it could mean.
   *   - sign(distance) == sign(speed), same rule the upstream behavior applies,
   *     so a half-edited goal aborts instead of driving the wrong way.
   *   - |distance| <= max_distance (server-side parameter). This is a THIRD,
   *     strongest bound that the reverse recovery could not have: BackUp is
   *     upstream code with no server-side distance parameter, this behavior is
   *     ours. It binds every client, including one that bypasses the BT.
   */
  nav2_behaviors::ResultStatus onRun(const std::shared_ptr<const ActionT::Goal> command) override;

  /**
   * @brief One control cycle: publish the lateral twist or finish.
   */
  nav2_behaviors::ResultStatus onCycleUpdate() override;

  /**
   * @brief Local costmap only - same resource the reverse recovery guards with.
   */
  CostmapInfoType getResourceInfo() override {return CostmapInfoType::LOCAL;}

protected:
  void onConfigure() override;

  /**
   * @brief Roll the footprint sideways through the costmap and report whether
   *        the whole remaining manoeuvre is free.
   * @param travelled distance already covered
   * @param lateral_speed signed vy that would be commanded
   * @param pose2d current pose (modified in place, as upstream does)
   * @param fetch_data fetch costmap+footprint on the first probe of a batch
   */
  bool isCollisionFree(
    const double & travelled,
    double lateral_speed,
    geometry_msgs::msg::Pose2D & pose2d,
    bool fetch_data);

  ActionT::Feedback::SharedPtr feedback_;

  geometry_msgs::msg::PoseStamped initial_pose_;
  double command_y_;
  double command_speed_;
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
  double simulate_ahead_time_;
  double max_distance_;
  bool allow_mirrored_fallback_;

  // --- alignment grace (the steering transient) -----------------------------
  // The controller may WITHHOLD DRIVE while the four modules swing into the
  // +-90 deg crab pose (swerve_controller's alignment gate). During that time
  // the robot correctly does not move, and without this the clock the goal is
  // judged against would be spent standing still: 0.17 m at 0.10 m/s is 1.70 s
  // of travel against a 2.8 s allowance, so a worst-case 1.5 s alignment would
  // make a CORRECT crab fail with TIMEOUT.
  //
  // WHY A SECOND CLOCK RATHER THAN A BIGGER time_allowance, and this is the
  // whole point: time_allowance is also a DISTANCE BOUND. The BT comment calls
  // it bound (c) of three -- speed x time, 0.10 x 2.8 = 0.28 m -- the backstop
  // that holds if max_distance and dist_to_travel are both wrong. Simply raising
  // it to 4.3 s would quietly relax that bound to 0.43 m, a 54 % loosening of a
  // documented safety property, to pay for time in which the robot is standing
  // still by construction.
  //
  // So the allowance is spent only once the robot is MEASURABLY MOVING. The
  // pre-motion phase gets its own bound, and bound (c) survives at
  //     0.10 x 2.8 + one cycle of travel = 0.28 + 0.01 = 0.29 m,
  // a 3.5 % loosening instead of 54 %.
  double alignment_grace_sec_{0.0};
  /// Whether the robot has covered at least one cycle's worth of travel. Not a
  /// tuned threshold: it is `|speed| / cycle_frequency_`, i.e. the smallest
  /// displacement this behavior could possibly observe between two of its own
  /// cycles, so anything at or above it is motion rather than transform noise.
  bool motion_started_{false};
  /// Bound on the pre-motion phase. Reaching it means the robot never started
  /// moving at all, which is a different failure from "moved too slowly" and is
  /// reported as such.
  rclcpp::Time grace_deadline_;
};

}  // namespace gripperx_behaviors

#endif  // GRIPPERX_BEHAVIORS__CRAB_WALK_HPP_
