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

#include "gripperx_behaviors/crab_walk.hpp"

#include <cmath>
#include <memory>
#include <utility>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace gripperx_behaviors
{

using nav2_behaviors::ResultStatus;
using nav2_behaviors::Status;

CrabWalk::CrabWalk()
: nav2_behaviors::TimedBehavior<nav2_msgs::action::DriveOnHeading>(),
  feedback_(std::make_shared<ActionT::Feedback>()),
  command_y_(0.0),
  command_speed_(0.0),
  simulate_ahead_time_(0.0),
  max_distance_(0.0),
  allow_mirrored_fallback_(true)
{
}

void CrabWalk::onConfigure()
{
  auto node = this->node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Shared with the upstream behaviors on the same server; declare_if_not_declared
  // because drive_on_heading/backup declare the same key.
  nav2_util::declare_parameter_if_not_declared(
    node, "simulate_ahead_time", rclcpp::ParameterValue(2.0));
  node->get_parameter("simulate_ahead_time", simulate_ahead_time_);

  // Behavior-scoped, hence prefixed with the plugin name.
  nav2_util::declare_parameter_if_not_declared(
    node, this->behavior_name_ + ".max_distance", rclcpp::ParameterValue(0.20));
  node->get_parameter(this->behavior_name_ + ".max_distance", max_distance_);

  nav2_util::declare_parameter_if_not_declared(
    node, this->behavior_name_ + ".allow_mirrored_fallback", rclcpp::ParameterValue(true));
  node->get_parameter(
    this->behavior_name_ + ".allow_mirrored_fallback", allow_mirrored_fallback_);

  // DEFAULT 0.0 = the pre-2026-08-24 behaviour EXACTLY: with no grace the
  // pre-motion phase is bounded by the same instant the motion phase would have
  // started at, so end_time_ lands where it always did. It only needs a non-zero
  // value once swerve_controller's alignment gate is enabled. See the header.
  nav2_util::declare_parameter_if_not_declared(
    node, this->behavior_name_ + ".alignment_grace_sec", rclcpp::ParameterValue(0.0));
  node->get_parameter(this->behavior_name_ + ".alignment_grace_sec", alignment_grace_sec_);
  if (alignment_grace_sec_ < 0.0) {
    RCLCPP_WARN(
      this->logger_, "%s.alignment_grace_sec %.2f is negative; clamped to 0.0.",
      this->behavior_name_.c_str(), alignment_grace_sec_);
    alignment_grace_sec_ = 0.0;
  }

  RCLCPP_INFO(
    this->logger_,
    "%s configured: max_distance %.3f m, allow_mirrored_fallback %s, simulate_ahead_time %.2f s, "
    "alignment_grace %.2f s",
    this->behavior_name_.c_str(), max_distance_,
    allow_mirrored_fallback_ ? "true" : "false", simulate_ahead_time_, alignment_grace_sec_);
}

ResultStatus CrabWalk::onRun(const std::shared_ptr<const ActionT::Goal> command)
{
  if (command->target.z != 0.0) {
    RCLCPP_ERROR(this->logger_, "CrabWalk: target.z must be zero.");
    return ResultStatus{Status::FAILED, ActionT::Result::INVALID_INPUT};
  }
  if (command->target.x != 0.0 && command->target.y != 0.0) {
    RCLCPP_ERROR(
      this->logger_,
      "CrabWalk: exactly one of target.x / target.y may be non-zero (got %.3f / %.3f).",
      command->target.x, command->target.y);
    return ResultStatus{Status::FAILED, ActionT::Result::INVALID_INPUT};
  }

  const double requested = (command->target.y != 0.0) ? command->target.y : command->target.x;
  if (requested == 0.0) {
    RCLCPP_ERROR(this->logger_, "CrabWalk: zero distance requested.");
    return ResultStatus{Status::FAILED, ActionT::Result::INVALID_INPUT};
  }
  if ((requested > 0.0) != (command->speed > 0.0)) {
    RCLCPP_ERROR(this->logger_, "CrabWalk: speed and distance sign did not match.");
    return ResultStatus{Status::FAILED, ActionT::Result::INVALID_INPUT};
  }

  // Strongest of the three distance bounds: config-side, applies to every
  // client. Rejecting rather than silently clamping keeps a wrong BT port
  // visible instead of turning it into a shorter, still-executed motion.
  if (std::fabs(requested) > max_distance_) {
    RCLCPP_ERROR(
      this->logger_, "CrabWalk: %.3f m exceeds %s.max_distance %.3f m - refusing.",
      std::fabs(requested), this->behavior_name_.c_str(), max_distance_);
    return ResultStatus{Status::FAILED, ActionT::Result::INVALID_INPUT};
  }

  command_y_ = requested;
  command_speed_ = command->speed;
  command_time_allowance_ = command->time_allowance;
  // TWO CLOCKS. `grace_deadline_` bounds the phase before the robot is
  // measurably moving — the steering transient, during which the controller's
  // alignment gate correctly holds the drive at zero. `end_time_` is the goal's
  // own allowance and is (re)started when motion begins, so it keeps meaning
  // "speed x time" as a distance bound. With alignment_grace_sec_ == 0.0 the
  // grace deadline is now, motion_started_ flips on the first cycle that sees
  // any travel, and the arithmetic collapses to what it was before.
  motion_started_ = false;
  grace_deadline_ = this->clock_->now() + rclcpp::Duration::from_seconds(alignment_grace_sec_);
  end_time_ = this->clock_->now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(
      initial_pose_, *this->tf_, this->local_frame_, this->robot_base_frame_,
      this->transform_tolerance_))
  {
    RCLCPP_ERROR(this->logger_, "CrabWalk: initial robot pose is not available.");
    return ResultStatus{Status::FAILED, ActionT::Result::TF_ERROR};
  }

  // Pre-flight over the WHOLE manoeuvre, and the side choice.
  //
  // The BT port carries the PREFERRED side (sign of dist_to_travel / speed);
  // the mirrored side is used only when the preferred one is blocked. Rationale
  // is in config/nav2.yaml at behavior_server: a fixed side would burn a whole
  // RoundRobin entry whenever it happens to be the blocked one, and both sides
  // are checked against the same costmap with the same footprint, so the
  // fallback adds no new failure surface - only a second, equally guarded
  // option. Set <name>.allow_mirrored_fallback:=false to get a strictly
  // BT-dictated direction back without a rebuild.
  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = initial_pose_.pose.position.x;
  pose2d.y = initial_pose_.pose.position.y;
  pose2d.theta = tf2::getYaw(initial_pose_.pose.orientation);

  if (!isCollisionFree(0.0, command_speed_, pose2d, true)) {
    if (!allow_mirrored_fallback_) {
      RCLCPP_WARN(
        this->logger_, "CrabWalk: preferred side blocked and fallback disabled - refusing.");
      return ResultStatus{Status::FAILED, ActionT::Result::COLLISION_AHEAD};
    }
    pose2d.x = initial_pose_.pose.position.x;
    pose2d.y = initial_pose_.pose.position.y;
    if (!isCollisionFree(0.0, -command_speed_, pose2d, false)) {
      RCLCPP_WARN(this->logger_, "CrabWalk: both sides blocked - refusing.");
      return ResultStatus{Status::FAILED, ActionT::Result::COLLISION_AHEAD};
    }
    RCLCPP_INFO(
      this->logger_, "CrabWalk: preferred side (%s) blocked, mirroring to %s.",
      command_y_ > 0.0 ? "left" : "right", command_y_ > 0.0 ? "right" : "left");
    command_y_ = -command_y_;
    command_speed_ = -command_speed_;
  }

  RCLCPP_INFO(
    this->logger_, "CrabWalk: %.3f m to the %s at %.3f m/s, allowance %.2f s.",
    std::fabs(command_y_), command_y_ > 0.0 ? "left" : "right",
    std::fabs(command_speed_), command_time_allowance_.seconds());

  return ResultStatus{Status::SUCCEEDED, ActionT::Result::NONE};
}

ResultStatus CrabWalk::onCycleUpdate()
{
  // The allowance is only spent while the robot is actually moving — see the
  // alignment_grace_sec_ block in the header for why this is not simply a larger
  // time_allowance. Before motion starts the pre-motion bound applies instead,
  // and running out of THAT is a different failure ("never started") from
  // running out of the allowance ("too slow"), so it is reported separately.
  if (motion_started_ || alignment_grace_sec_ <= 0.0) {
    const rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
    if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
      this->stopRobot();
      RCLCPP_WARN(this->logger_, "Exceeded time allowance before reaching the CrabWalk goal.");
      return ResultStatus{Status::FAILED, ActionT::Result::TIMEOUT};
    }
  } else if (this->clock_->now() > grace_deadline_) {
    this->stopRobot();
    RCLCPP_WARN(
      this->logger_,
      "CrabWalk never started moving within the %.2f s alignment grace. Either the steering "
      "modules did not reach the crab pose, or the drive is being withheld for another reason "
      "-- check alignment_status on /swerve_controller/wheel_velocities.",
      alignment_grace_sec_);
    return ResultStatus{Status::FAILED, ActionT::Result::TIMEOUT};
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *this->tf_, this->local_frame_, this->robot_base_frame_,
      this->transform_tolerance_))
  {
    RCLCPP_ERROR(this->logger_, "CrabWalk: current robot pose is not available.");
    return ResultStatus{Status::FAILED, ActionT::Result::TF_ERROR};
  }

  // Straight-line distance from the start, exactly as DriveOnHeading measures
  // it. NOTE what this does NOT measure: the heading the robot picked up on the
  // way. A crab carries a small yaw bias (measured on the twin: 0.0138 rad worst
  // case over the configured 0.20 m, numbers at behavior_server in
  // config/nav2.yaml) which the wheel odometry cannot see. In the twin SLAM
  // absorbs it; on the real robot, with all three EKF inputs dead, nothing does.
  //
  // This check is also why the executed distance runs slightly long: it fires at
  // cycle_frequency (10 Hz), so up to one cycle of travel passes before the stop
  // is issued, and the plant then coasts. Measured 0.197..0.222 m for a 0.20 m
  // goal.
  const double diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
  const double diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
  const double distance = std::hypot(diff_x, diff_y);

  feedback_->distance_traveled = distance;
  this->action_server_->publish_feedback(feedback_);

  // MOTION IS DECLARED FROM A DERIVED FIGURE, NOT A TUNED ONE: one cycle's worth
  // of travel at the commanded speed is the smallest displacement this behavior
  // could observe between two of its own cycles, so at or above it the robot is
  // moving rather than the transform jittering. At 0.10 m/s and 10 Hz that is
  // 0.01 m — which is also the entire amount by which this can loosen the
  // speed x time distance bound, because everything after it is on the clock.
  if (!motion_started_ &&
    distance >= std::fabs(command_speed_) / std::max(1.0, this->cycle_frequency_))
  {
    motion_started_ = true;
    end_time_ = this->clock_->now() + command_time_allowance_;
  }

  if (distance >= std::fabs(command_y_)) {
    this->stopRobot();
    return ResultStatus{Status::SUCCEEDED, ActionT::Result::NONE};
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.stamp = this->clock_->now();
  cmd_vel->header.frame_id = this->robot_base_frame_;
  cmd_vel->twist.linear.x = 0.0;
  cmd_vel->twist.linear.y = command_speed_;
  cmd_vel->twist.angular.z = 0.0;

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  if (!isCollisionFree(distance, cmd_vel->twist.linear.y, pose2d, true)) {
    this->stopRobot();
    RCLCPP_WARN(this->logger_, "Collision Ahead - Exiting CrabWalk");
    return ResultStatus{Status::FAILED, ActionT::Result::COLLISION_AHEAD};
  }

  this->vel_pub_->publish(std::move(cmd_vel));

  return ResultStatus{Status::RUNNING, ActionT::Result::NONE};
}

bool CrabWalk::isCollisionFree(
  const double & travelled,
  double lateral_speed,
  geometry_msgs::msg::Pose2D & pose2d,
  bool fetch_data)
{
  // Same structure as nav2_behaviors::DriveOnHeading::isCollisionFree, with one
  // substantive change: the probe walks along the LATERAL unit vector
  // (-sin theta, +cos theta) instead of the heading (cos theta, sin theta).
  // The body does not rotate during a crab, so pose2d.theta stays put and the
  // footprint is swept sideways - which is exactly the swept volume of this
  // manoeuvre.
  //
  // Unknown ground blocks by itself: local_costmap.track_unknown_space is true,
  // so never-observed cells are NO_INFORMATION (255) >= LETHAL_OBSTACLE (254)
  // and FootprintCollisionChecker scores them as a collision. That is the same
  // guard the reverse recovery relies on; laterally it is a second line rather
  // than the only one, because the LD06's blind wedge is at the rear.
  int cycle_count = 0;
  double sim_position_change;
  const double diff_dist = std::fabs(command_y_) - travelled;
  const int max_cycle_count = static_cast<int>(this->cycle_frequency_ * simulate_ahead_time_);
  const geometry_msgs::msg::Pose2D init_pose = pose2d;

  while (cycle_count < max_cycle_count) {
    sim_position_change = lateral_speed * (cycle_count / this->cycle_frequency_);
    pose2d.x = init_pose.x - sim_position_change * std::sin(init_pose.theta);
    pose2d.y = init_pose.y + sim_position_change * std::cos(init_pose.theta);
    cycle_count++;

    if (diff_dist - std::fabs(sim_position_change) <= 0.0) {
      break;
    }

    if (!this->local_collision_checker_->isCollisionFree(pose2d, fetch_data)) {
      return false;
    }
    fetch_data = false;
  }
  return true;
}

}  // namespace gripperx_behaviors

PLUGINLIB_EXPORT_CLASS(gripperx_behaviors::CrabWalk, nav2_core::Behavior)
