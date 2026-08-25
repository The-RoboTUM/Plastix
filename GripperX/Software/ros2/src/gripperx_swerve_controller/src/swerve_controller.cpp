#include "gripperx_swerve_controller/swerve_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace gripperx_swerve_controller
{

namespace
{
constexpr double kRadToDeg = 180.0 / M_PI;

// The regulator status codes exist in TWO places — the rclcpp-free enum in
// wheel_regulator.hpp and the constants in WheelVelocityReport.msg — because the
// control law may not depend on a generated message type. That is exactly the
// arrangement in which two enumerations silently drift apart, so the agreement
// is checked HERE, at compile time, rather than trusted.
using ReportMsg = gripperx_control_msgs::msg::WheelVelocityReport;
/// Name the manoeuvre the executed twist describes. DERIVED, NEVER COMMANDED —
/// see the `manoeuvre` field in WheelVelocityReport.msg for why there is no
/// mode topic. Nothing in the control path reads the answer.
///
/// The test is EXACT against zero, deliberately and for the same reason
/// OP-24/S1's zero-twist test is: an epsilon here would be a second, unmeasured
/// threshold sitting next to the one at stage 2, and the twists this classifies
/// are produced by the same sources whose exact zeros that test already relies on.
/// A near-zero-but-not-zero Nav2 twist therefore reads as GENERAL rather than as
/// a named manoeuvre, which is the honest answer: it IS a general twist.
uint8_t classify_manoeuvre(const BodyTwist & twist)
{
  const bool moving_x = twist.vx != 0.0;
  const bool moving_y = twist.vy != 0.0;
  const bool turning = twist.omega != 0.0;

  if (!moving_x && !moving_y && !turning) {
    return ReportMsg::MANOEUVRE_IDLE;
  }
  if (!turning && moving_y) {
    // Every module parallel: a pure translation. Includes the DIAGONAL case
    // (vx and vy both non-zero), which is still one translation direction and
    // still one pose — the teleop's steered crab produces exactly this.
    return ReportMsg::MANOEUVRE_CRAB;
  }
  if (turning && !moving_x && !moving_y) {
    return ReportMsg::MANOEUVRE_SPIN;
  }
  if (!moving_y) {
    // vx with or without omega: the ordinary cornering family, which is what
    // W/S and A/D have always produced.
    return ReportMsg::MANOEUVRE_CORNERING;
  }
  return ReportMsg::MANOEUVRE_GENERAL;
}

static_assert(kAlignDisabled == ReportMsg::ALIGNMENT_DISABLED, "alignment status mismatch");
static_assert(kAlignPassing == ReportMsg::ALIGNMENT_PASSING, "alignment status mismatch");
static_assert(kAlignSlewing == ReportMsg::ALIGNMENT_SLEWING, "alignment status mismatch");
static_assert(kAlignTimedOut == ReportMsg::ALIGNMENT_TIMED_OUT, "alignment status mismatch");
static_assert(kRegulatorDisabled == ReportMsg::REGULATOR_DISABLED, "status mismatch");
static_assert(kRegulatorActive == ReportMsg::REGULATOR_ACTIVE, "status mismatch");
static_assert(
  kRegulatorAtAuthorityLimit == ReportMsg::REGULATOR_AT_AUTHORITY_LIMIT, "status mismatch");
static_assert(kRegulatorOffProvenance == ReportMsg::REGULATOR_OFF_PROVENANCE, "status mismatch");
static_assert(
  kRegulatorOffNoMeasurement == ReportMsg::REGULATOR_OFF_NO_MEASUREMENT, "status mismatch");
static_assert(
  kRegulatorOffStaleFeedback == ReportMsg::REGULATOR_OFF_STALE_FEEDBACK, "status mismatch");
static_assert(
  kRegulatorOffStallLatched == ReportMsg::REGULATOR_OFF_STALL_LATCHED, "status mismatch");
static_assert(
  kRegulatorOffBelowFloor == ReportMsg::REGULATOR_OFF_BELOW_FLOOR, "status mismatch");

/// Reorder a joint-order (FL, FR, BL, BR) array into model order (FL, BL, BR, FR).
std::array<double, kNumWheels> to_model_order(const std::array<double, kNumWheels> & joint_order)
{
  std::array<double, kNumWheels> model_order{};
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    model_order[i] = joint_order[kModelToJointIndex[i]];
  }
  return model_order;
}

/// Reorder a model-order (FL, BL, BR, FR) array into joint order (FL, FR, BL, BR).
std::array<double, kNumWheels> to_joint_order(const std::array<double, kNumWheels> & model_order)
{
  std::array<double, kNumWheels> joint_order{};
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    joint_order[i] = model_order[kJointToModelIndex[i]];
  }
  return joint_order;
}

bool all_finite(const std::array<double, kNumWheels> & values)
{
  return std::all_of(
    values.begin(), values.end(), [](double v) { return std::isfinite(v); });
}
}  // namespace

controller_interface::CallbackReturn SwerveController::on_init()
{
  try {
    // Geometry — swerve_cmd.yaml, all three TO-VERIFY there (hand measurements
    // that contradict the CAD-derived URDF; the effective ROLLING radius, not
    // half the geometric diameter, is what odometry needs). Carried over with
    // the values the active config has today, unchanged.
    a_ = auto_declare<double>("a", a_);
    b_ = auto_declare<double>("b", b_);
    wheel_radius_ = auto_declare<double>("wheel_radius", wheel_radius_);

    // CONTACT-POINT CORRECTION (measured 2026-08-21: one commanded nominal
    // revolution in place turned the chassis ~270 deg instead of 360 — the
    // controller was commanding each wheel the speed its KING PIN needs while
    // the tyre runs on a 26 % larger circle). Joint order FL, FR, BL, BR,
    // signed in base_link y, positive = left.
    //
    // DEFAULT ZERO IS LOAD-BEARING, not a nicety: on a config without the key
    // the model reduces to the king-pin model bit for bit, so an un-migrated
    // machine behaves exactly as it did rather than differently-but-plausibly.
    // Which is why the resolved values are LOGGED below.
    const auto lateral_offsets =
      auto_declare<std::vector<double>>("wheel_lateral_offset", {0.0, 0.0, 0.0, 0.0});
    if (lateral_offsets.size() != kNumWheels) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "wheel_lateral_offset must have exactly %zu entries", kNumWheels);
      return controller_interface::CallbackReturn::ERROR;
    }
    std::copy(lateral_offsets.begin(), lateral_offsets.end(), wheel_lateral_offset_.begin());

    steering_joint_names_ = auto_declare<std::vector<std::string>>(
      "steering_joint_names", {"f_left_steer", "f_right_steer", "b_leftsteer", "b_rightsteer"});
    wheel_joint_names_ = auto_declare<std::vector<std::string>>(
      "wheel_joint_names", {"f_leftwheel", "f_rightwheel", "b_leftwheel", "b_rightwheel"});

    // §3.1.6 item 3 — the per-side wheel sign that joint_command_bridge.sim.yaml
    // carries today. The sim URDF gives the right-side wheel joints axis
    // "0 0 -1", so the sim needs [1, -1, 1, -1] where the real robot needs
    // [1, 1, 1, 1] (there the motor wiring compensates). It is a PARAMETER with
    // the same two value sets on purpose: hard-coding or dropping it makes the
    // twin drive sideways on a straight-ahead command.
    const auto multipliers =
      auto_declare<std::vector<double>>("wheel_command_multipliers", {1.0, 1.0, 1.0, 1.0});
    if (multipliers.size() != kNumWheels) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "wheel_command_multipliers must have exactly %zu entries",
        kNumWheels);
      return controller_interface::CallbackReturn::ERROR;
    }
    std::copy(multipliers.begin(), multipliers.end(), wheel_command_multipliers_.begin());

    // Per-wheel, per-direction steering window — FR-5 / SR-6, SAFETY-RELEVANT.
    // Mirrors gripperx_control/config/steer_servo.yaml, which is the source of
    // truth and whose clamp in steer_servo_node stays the last line of defence.
    const double outward_deg =
      auto_declare<double>("steering_outward_limit_deg", kDefaultOutwardLimitDeg);
    const double inward_deg =
      auto_declare<double>("steering_inward_limit_deg", kDefaultInwardLimitDeg);
    const auto outward_sign_param = auto_declare<std::vector<int64_t>>(
      "steering_outward_sign",
      {kDefaultOutwardSign[0], kDefaultOutwardSign[1], kDefaultOutwardSign[2],
       kDefaultOutwardSign[3]});
    if (outward_sign_param.size() != kNumWheels) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "steering_outward_sign must have exactly %zu entries",
        kNumWheels);
      return controller_interface::CallbackReturn::ERROR;
    }
    std::array<int, kNumWheels> outward_sign{};
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      outward_sign[i] = static_cast<int>(outward_sign_param[i]);
    }

    steer_alignment_min_scale_ =
      auto_declare<double>("steer_alignment_min_scale", steer_alignment_min_scale_);
    steer_alignment_deadband_rad_ =
      auto_declare<double>("steer_alignment_deadband_rad", steer_alignment_deadband_rad_);
    steer_alignment_reference_rad_ =
      auto_declare<double>("steer_alignment_reference_rad", steer_alignment_reference_rad_);
    max_wheel_angular_speed_ =
      auto_declare<double>("max_wheel_angular_speed", max_wheel_angular_speed_);
    cmd_vel_timeout_sec_ = auto_declare<double>("cmd_vel_timeout_sec", cmd_vel_timeout_sec_);
    enforce_front_forward_ = auto_declare<bool>("enforce_front_forward", enforce_front_forward_);
    allow_reverse_ = auto_declare<bool>("allow_reverse", allow_reverse_);

    // Task #21 steering differential. Node default stays FALSE exactly as in
    // swerve_cmd_node; swerve_cmd.yaml turns it ON (a29e181, "a/d wheel
    // differential on") and the ported config does the same. It is NOT on
    // §3.1.4 (a)'s carry-over list — that list is not exhaustive, see the
    // findings in this change's report — but it is ACTIVE today, so dropping it
    // would be a functional regression against NFR-10 acceptance 1.
    enable_steer_feedback_differential_ =
      auto_declare<bool>("enable_steer_feedback_differential", enable_steer_feedback_differential_);
    steer_diff_omega_gate_ = auto_declare<double>("steer_diff_omega_gate", steer_diff_omega_gate_);
    steer_diff_min_speed_mps_ =
      auto_declare<double>("steer_diff_min_speed_mps", steer_diff_min_speed_mps_);
    steer_diff_time_constant_sec_ =
      auto_declare<double>("steer_diff_time_constant_sec", steer_diff_time_constant_sec_);
    steer_diff_max_omega_ = auto_declare<double>("steer_diff_max_omega", steer_diff_max_omega_);
    steer_diff_min_ratio_ = auto_declare<double>("steer_diff_min_ratio", steer_diff_min_ratio_);
    steer_diff_max_ratio_ = auto_declare<double>("steer_diff_max_ratio", steer_diff_max_ratio_);

    cmd_vel_topic_ = auto_declare<std::string>("cmd_vel_topic", cmd_vel_topic_);
    // Arbitration point A2 moves in here — OP-23 / A2-b. Values carried over
    // from steer_servo_node, which drops this subscription.
    direct_steer_topic_ = auto_declare<std::string>("direct_steer_topic", direct_steer_topic_);
    direct_timeout_sec_ = auto_declare<double>("direct_timeout_sec", direct_timeout_sec_);
    active_mode_topic_ = auto_declare<std::string>("active_mode_topic", active_mode_topic_);
    autonomous_mode_name_ =
      auto_declare<std::string>("autonomous_mode_name", autonomous_mode_name_);

    intent_echo_topic_ = auto_declare<std::string>("intent_echo_topic", intent_echo_topic_);
    wheel_report_topic_ = auto_declare<std::string>("wheel_report_topic", wheel_report_topic_);

    // --- HWR-30a: encoder-based stall detection + tier-1 response ------------
    // EVERY THRESHOLD HERE IS `TO-VERIFY`, and for one shared reason: the window
    // this protects is a THERMAL property of the deliberately thin motor lead
    // (WIRING_PLAN §8.2 Option B) at STALL CURRENT, and the GB37-50 stall-current
    // measurement (WIRING_PLAN §8.5) DOES NOT EXIST. Until it does, no threshold
    // here is derived; they are chosen to be safe against FALSE trips, because a
    // false trip stops a working wheel mid-drive and that is itself a hazard.
    // The per-value reasoning is in ros2_controllers.yaml, not duplicated here.
    stall_config_.enabled = auto_declare<bool>("stall_detection_enabled", stall_config_.enabled);
    stall_config_.window_sec = auto_declare<double>("stall_window_sec", stall_config_.window_sec);
    stall_config_.min_command_rad_s =
      auto_declare<double>("stall_min_command_rad_s", stall_config_.min_command_rad_s);
    stall_config_.min_position_delta_rad =
      auto_declare<double>("stall_min_position_delta_rad", stall_config_.min_position_delta_rad);
    stall_config_.release_command_rad_s =
      auto_declare<double>("stall_release_command_rad_s", stall_config_.release_command_rad_s);
    // int64_t, not int: ROS integer parameters ARE int64_t, and the file already
    // declares steering_outward_sign that way.
    const auto max_latched = auto_declare<int64_t>(
      "stall_max_latched_wheels", static_cast<int64_t>(stall_config_.max_latched_wheels));
    stall_config_.max_latched_wheels =
      max_latched > 0 ? static_cast<std::size_t>(max_latched) : 0;
    // FALSE on the real robot, TRUE only in swerve_controller.sim.yaml: nothing
    // publishes /hw/wheel_feedback_valid in the twin, so a missing provenance
    // topic MUST disarm the detector rather than be assumed away.
    stall_config_.assume_live_provenance =
      auto_declare<bool>("assume_live_provenance", stall_config_.assume_live_provenance);
    wheel_feedback_valid_topic_ =
      auto_declare<std::string>("wheel_feedback_valid_topic", wheel_feedback_valid_topic_);
    stall_state_topic_ = auto_declare<std::string>("stall_state_topic", stall_state_topic_);
    stall_log_period_sec_ = auto_declare<double>("stall_log_period_sec", stall_log_period_sec_);

    std::string stall_error;
    if (!StallDetector::validate(stall_config_, stall_error)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Stall detection (HWR-30a): %s", stall_error.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    stall_detector_.configure(stall_config_);

    // --- STEERING ALIGNMENT GATE --------------------------------------------
    // DEFAULT FALSE, IN THE STRUCT AND IN ros2_controllers.yaml. While it is
    // false the gate is never entered and the wheel command passes through
    // bit-identically, so this whole feature is inert until someone turns it on.
    // Unlike the regulator, turning THIS on does not close any loop — the gate
    // can only ever write exactly 0.0 — so NFR-10 acceptance 10 is unaffected
    // either way. What it DOES change is how a crab or spin entry feels, which
    // is why it is a deliberate switch and not a default. See alignment_gate.hpp.
    alignment_config_.enabled =
      auto_declare<bool>("alignment_gate_enabled", alignment_config_.enabled);
    alignment_config_.entry_jump_rad =
      auto_declare<double>("alignment_entry_jump_rad", alignment_config_.entry_jump_rad);
    alignment_config_.entry_error_rad =
      auto_declare<double>("alignment_entry_error_rad", alignment_config_.entry_error_rad);
    alignment_config_.exit_tolerance_rad =
      auto_declare<double>("alignment_exit_tolerance_rad", alignment_config_.exit_tolerance_rad);
    alignment_config_.timeout_sec =
      auto_declare<double>("alignment_timeout_sec", alignment_config_.timeout_sec);
    alignment_log_period_sec_ =
      auto_declare<double>("alignment_log_period_sec", alignment_log_period_sec_);

    std::string alignment_error;
    if (!AlignmentGate::validate(alignment_config_, alignment_error)) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Steering alignment gate: %s", alignment_error.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    alignment_gate_.configure(alignment_config_);

    // --- PER-WHEEL VELOCITY REGULATOR ---------------------------------------
    // DEFAULT FALSE, IN THE STRUCT AND IN ros2_controllers.yaml. While it is
    // false the regulator is never called and the wheel command is the
    // bit-identical feedforward value, so FR-11 item 2 and NFR-10 acceptance 10
    // hold unchanged. Turning it on is a REQUIREMENTS decision, not a tuning
    // step — see the header of wheel_regulator.hpp.
    // EVERY GAIN IS TO-VERIFY. Nothing here is tuned; the per-value reasoning is
    // in ros2_controllers.yaml and at the declarations in WheelRegulatorConfig.
    regulator_config_.enabled =
      auto_declare<bool>("wheel_regulator_enabled", regulator_config_.enabled);
    regulator_config_.kp = auto_declare<double>("wheel_regulator_kp", regulator_config_.kp);
    regulator_config_.ki = auto_declare<double>("wheel_regulator_ki", regulator_config_.ki);
    regulator_config_.max_correction_fraction = auto_declare<double>(
      "wheel_regulator_max_correction_fraction", regulator_config_.max_correction_fraction);
    regulator_config_.max_sample_age_sec = auto_declare<double>(
      "wheel_regulator_max_sample_age_sec", regulator_config_.max_sample_age_sec);
    regulator_log_period_sec_ =
      auto_declare<double>("wheel_regulator_log_period_sec", regulator_log_period_sec_);
    // NOT a parameter of its own, on purpose: the correction must not be able to
    // push the command past the ceiling the control law already applied to the
    // setpoint, and a second copy of that ceiling could disagree with the first.
    regulator_config_.output_limit_rad_s = max_wheel_angular_speed_;
    // ONE `assume_live_provenance` PARAMETER, TWO CONSUMERS. Both the stall
    // detector and the regulator gate on the same latched topic, so they share
    // the same statement about whether that topic exists on this platform
    // (false real, true in swerve_controller.sim.yaml). Two parameters would be
    // two chances for the two consumers of one topic to disagree about it.
    regulator_config_.assume_live_provenance = stall_config_.assume_live_provenance;

    std::string regulator_error;
    if (!WheelRegulator::validate(regulator_config_, regulator_error)) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Wheel velocity regulator: %s", regulator_error.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    wheel_regulator_.configure(regulator_config_);

    if (steering_joint_names_.size() != kNumWheels || wheel_joint_names_.size() != kNumWheels) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "steering_joint_names and wheel_joint_names must each have exactly %zu entries", kNumWheels);
      return controller_interface::CallbackReturn::ERROR;
    }
    if (steer_alignment_reference_rad_ <= 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "steer_alignment_reference_rad must be positive");
      return controller_interface::CallbackReturn::ERROR;
    }

    // The model works in Lee-2015 order (FL, BL, BR, FR); the parameter is in
    // joint order. Converted explicitly here, never implicitly.
    model_ = std::make_unique<SwerveKinematics>(
      a_, b_, wheel_radius_, to_model_order(wheel_lateral_offset_));
    joint_steering_limits_ = SteeringLimits::from_outward_inward(
      outward_deg * M_PI / 180.0, inward_deg * M_PI / 180.0, outward_sign);
    model_steering_limits_ = joint_steering_limits_.in_model_order();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SwerveController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : steering_joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto & joint : wheel_joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

controller_interface::InterfaceConfiguration SwerveController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // Steering POSITION: the feedback the limit resolution and the slew braking
  // work from. On the real robot this carries /hw/steer_states, adopted by
  // GripperXInterface (FR-10) — before that fix it was a constant 0.0 on
  // hardware while being truthful in sim, which is precisely the fork FR-10
  // closed and the reason FR-10 gated this controller's feedback path.
  for (const auto & joint : steering_joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  // Wheel VELOCITY: read and published next to the command (FR-11 item 1).
  // NOT fed back into the control law (FR-11 item 2).
  for (const auto & joint : wheel_joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  // Wheel POSITION: the ACCUMULATED encoder count, and the quantity HWR-30a's
  // detection keys off. Claimed only while something needs it — stall detection
  // or the velocity regulator; see needs_wheel_position_state().
  //
  // WHY POSITION AND NOT VELOCITY: HWR-30a binds the detection to the
  // encoder-valid condition and to the counts themselves. The velocity state is
  // a first difference computed in the firmware — it is exactly the quantity the
  // requirement forbids keying off, because a dead encoder reporting a plausible
  // 0.0 and a healthy stationary wheel are bit-identical in it (FR-11's
  // superseded provenance criterion).
  //
  // Both platforms declare it: gripperx_v1.ros2_control.xacro's
  // ros2_control_velocity_joint macro carries <state_interface name="position"/>
  // on all four wheel joints, GripperXInterface::export_state_interfaces()
  // exports position for every joint, and GazeboSimSystem reports true joint
  // positions. The parameter exists so that a component which does NOT export it
  // can still run this controller instead of failing to activate.
  if (needs_wheel_position_state()) {
    for (const auto & joint : wheel_joint_names_) {
      config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
  }
  // The steering VELOCITY state is deliberately NOT claimed: it is a constant
  // 0.0 on the real robot and truthful in Gazebo (OP-26 / D16, accepted
  // deviation). Claiming it would give this controller a value that means
  // different things on the two platforms, for no consumer.
  return config;
}

controller_interface::CallbackReturn SwerveController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const StampedTwist empty_twist;
  const StampedDirectSteer empty_direct;
  cmd_vel_buffer_.writeFromNonRT(empty_twist);
  direct_steer_buffer_.writeFromNonRT(empty_direct);
  cmd_vel_rx_count_.store(0);
  mode_is_autonomous_.store(false);
  steer_diff_omega_filtered_ = 0.0;
  last_wheel_command_.fill(0.0);
  last_wheel_correction_.fill(0.0);
  last_wheel_regulating_.fill(false);
  last_wheel_regulator_status_.fill(kRegulatorDisabled);
  regulator_at_limit_.fill(false);
  regulator_last_limit_log_sec_.fill(0.0);
  regulator_ever_limit_logged_.fill(false);
  wheel_regulator_.configure(regulator_config_);
  regulator_enabled_rt_.store(regulator_config_.enabled);
  regulator_active_ = regulator_config_.enabled;
  // The regulator needs the wheel POSITION state (its new-sample novelty signal)
  // and the latched provenance topic. Both are claimed here or not at all, and
  // stall detection claims both as well — so on the real robot, where
  // stall_detection_enabled is true, the runtime switch always has what it needs.
  regulator_resources_claimed_ = stall_config_.enabled || regulator_config_.enabled;
  stall_detector_.configure(stall_config_);
  // configure() resets. A gate must never come back from a lifecycle transition
  // still withholding the drive, and must never come back believing the modules
  // are where they were before the robot was switched off.
  alignment_gate_.configure(alignment_config_);
  last_alignment_status_ = alignment_gate_.status();
  last_alignment_error_rad_ = 0.0;
  last_manoeuvre_ = ReportMsg::MANOEUVRE_IDLE;
  alignment_last_log_sec_ = 0.0;
  alignment_ever_logged_ = false;
  stall_last_log_sec_.fill(0.0);
  stall_ever_logged_.fill(false);
  stall_multi_wheel_condition_ = false;
  // Everything is UNKNOWN until the latched topic says otherwise. UNKNOWN is
  // negative on purpose (FR-11 item 5 / D14): "the message did not say" must sort
  // below the weakest thing the firmware can claim, and it can never arm the
  // detector.
  const std::array<int, kNumWheels> unknown_provenance{
    kStallProvenanceUnknown, kStallProvenanceUnknown, kStallProvenanceUnknown,
    kStallProvenanceUnknown};
  wheel_provenance_buffer_.writeFromNonRT(unknown_provenance);

  // Explicit QoS on every chain input. SystemDefaultsQoS() resolved to
  // BEST_EFFORT on this stack once and cost the watchdog 14 minutes of blindness
  // (gripperx_interface.cpp L388-389, SR-11 (4)) — a defaulted QoS on a command
  // topic repeats that bug exactly.
  rclcpp::QoS command_qos(rclcpp::KeepLast(10));
  command_qos.reliable();

  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, command_qos,
    std::bind(&SwerveController::cmd_vel_callback, this, std::placeholders::_1));
  direct_steer_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    direct_steer_topic_, command_qos,
    std::bind(&SwerveController::direct_steer_callback, this, std::placeholders::_1));
  active_mode_sub_ = get_node()->create_subscription<std_msgs::msg::String>(
    active_mode_topic_, command_qos,
    std::bind(&SwerveController::active_mode_callback, this, std::placeholders::_1));

  // ------------------------------------------------------------- HWR-30a gate
  // THE QoS HERE IS LOAD-BEARING AND A PLAIN VOLATILE SUBSCRIPTION SILENTLY
  // FAILS. GripperXInterface publishes /hw/wheel_feedback_valid LATCHED —
  // KeepLast(1) + RELIABLE + TRANSIENT_LOCAL — and ON CHANGE ONLY. Provenance
  // typically settles during activation and then never changes again, so a
  // volatile subscriber that comes up afterwards receives NOTHING, ever, reads
  // UNKNOWN forever, and the stall detector stays permanently disarmed while
  // looking perfectly healthy. Match all three settings or the feature is dead.
  //
  // ONE SUBSCRIPTION, TWO CONSUMERS (FR-11 item 6 gets its second one here): the
  // stall detector and the velocity regulator both gate per wheel on this same
  // latched provenance. A second subscription to the same topic would be a
  // second decoder of a safety-relevant flag, able to disagree with the first.
  if (stall_config_.enabled || regulator_config_.enabled) {
    rclcpp::QoS provenance_qos(rclcpp::KeepLast(1));
    provenance_qos.reliable();
    provenance_qos.transient_local();
    wheel_feedback_valid_sub_ = get_node()->create_subscription<std_msgs::msg::Int32MultiArray>(
      wheel_feedback_valid_topic_, provenance_qos,
      std::bind(&SwerveController::wheel_feedback_valid_callback, this, std::placeholders::_1));
  }

  // THE ECHO'S PUBLISHING PATH IS THE REQUIREMENT, not a detail (OP-18a item 1,
  // SR-11). It is filled from update(), i.e. from the controller_manager UPDATE
  // LOOP, and handed to a realtime_tools::RealtimePublisher, which owns its own
  // thread. Neither half touches the controller_manager EXECUTOR. If it did, the
  // echo would go silent for the same reason the command does and the watchdog's
  // divergence check would collapse into its silence check — the regression
  // option W1 was rejected for. QoS as mandated: KeepLast(10) + RELIABLE.
  rclcpp::QoS echo_qos(rclcpp::KeepLast(10));
  echo_qos.reliable();
  intent_echo_pub_ = std::make_unique<
    realtime_tools::RealtimePublisher<gripperx_control_msgs::msg::SwerveIntentEcho>>(
    get_node()->create_publisher<gripperx_control_msgs::msg::SwerveIntentEcho>(
      intent_echo_topic_, echo_qos));

  rclcpp::QoS report_qos(rclcpp::KeepLast(10));
  report_qos.reliable();
  wheel_report_pub_ = std::make_unique<
    realtime_tools::RealtimePublisher<gripperx_control_msgs::msg::WheelVelocityReport>>(
    get_node()->create_publisher<gripperx_control_msgs::msg::WheelVelocityReport>(
      wheel_report_topic_, report_qos));

  // SR-13 (rev 4) item 1: a tier-1 cut-off disables ONE motor while three keep
  // driving, so the robot keeps moving and the fault is invisible from the
  // driver's seat unless it is published. Latched, so an operator or a health
  // node that attaches later still learns that a wheel is off.
  rclcpp::QoS stall_qos(rclcpp::KeepLast(1));
  stall_qos.reliable();
  stall_qos.transient_local();
  stall_state_pub_ = get_node()->create_publisher<gripperx_control_msgs::msg::WheelStallState>(
    stall_state_topic_, stall_qos);

  // RUNTIME SWITCH (user decision 2026-08-20). Validation refuses what cannot
  // work; the post-set callback only publishes the accepted value into an atomic.
  on_set_parameters_handle_ = get_node()->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return this->validate_parameter_update(parameters);
    });
  post_set_parameters_handle_ = get_node()->add_post_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      this->apply_parameter_update(parameters);
    });

  RCLCPP_INFO(
    get_node()->get_logger(),
    "swerve_controller configured. cmd_vel=%s direct_steer=%s (%.2fs) active_mode=%s "
    "geometry(a=%.6f b=%.6f r=%.6f) cmd_vel_timeout=%.2fs steer_differential=%s",
    cmd_vel_topic_.c_str(), direct_steer_topic_.c_str(), direct_timeout_sec_,
    active_mode_topic_.c_str(), a_, b_, wheel_radius_, cmd_vel_timeout_sec_,
    enable_steer_feedback_differential_ ? "on" : "off");
  // Its own line, and unconditional. `ros2 param get` is not usable on this
  // machine during bringup (DDS traffic there provokes controller_manager
  // overruns), so journalctl has to be enough to tell a corrected machine from
  // one still running the king-pin model on the all-zero default.
  {
    const bool corrected = std::any_of(
      wheel_lateral_offset_.begin(), wheel_lateral_offset_.end(),
      [](double h) { return h != 0.0; });
    RCLCPP_INFO(
      get_node()->get_logger(),
      "King-pin -> tyre-contact lateral offsets (joint order FL, FR, BL, BR, m): "
      "[%+.6f, %+.6f, %+.6f, %+.6f] -- CONTACT-POINT correction %s. Wheel speeds carry "
      "-omega*h per wheel, applied after the +-180deg module fold; steering angles are "
      "unaffected by it.",
      wheel_lateral_offset_[0], wheel_lateral_offset_[1], wheel_lateral_offset_[2],
      wheel_lateral_offset_[3],
      corrected ? "ACTIVE" : "INACTIVE (all zero -- this machine spins the king-pin model, "
                             "i.e. it under-rotates ~21 % on every commanded rotation)");
  }
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Steering windows (joint order FL, FR, BL, BR, deg): %s -- must mirror steer_servo.yaml. "
    "Requests outside them reduce |omega| (wider turn) or, if the direction of travel itself is "
    "unreachable, are rejected.",
    joint_steering_limits_.describe().c_str());
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Wheel command multipliers (joint order): [%.1f, %.1f, %.1f, %.1f]. Open-loop control law: "
    "wheel velocity feedback is read and published, never fed back (FR-11).",
    wheel_command_multipliers_[0], wheel_command_multipliers_[1], wheel_command_multipliers_[2],
    wheel_command_multipliers_[3]);
  if (regulator_config_.enabled) {
    // WARN, not ERROR — downgraded 2026-08-21, OP-36(b). It was ERROR because an
    // enabled regulator contradicted FR-11 item 2 and NFR-10 acceptance 10 "as
    // written". The user has since DECIDED to enable it permanently (2026-08-20),
    // so it no longer contradicts anything; it is the intended configuration.
    // Leaving it at ERROR cost more than it bought: SR-13 item 2 requires ERROR to
    // mean "commands no longer reach the hardware", and an ERROR on every single
    // boot of a healthy machine devalues the level for the case it is reserved for.
    // It stays a WARN and not an INFO because it IS a standing condition worth
    // seeing: the machine runs closed-loop from boot, and that must not be
    // discoverable only by reading a config file.
    RCLCPP_WARN(
      get_node()->get_logger(),
      "WHEEL VELOCITY REGULATOR IS ENABLED (user decision 2026-08-20; this is the intended "
      "configuration). The control law is NOT open-loop: NFR-10 acceptance 10 no longer holds. "
      "PI trim on the feedforward: "
      "kp=%.4f ki=%.4f (both TO-VERIFY, neither tuned), correction bounded at %.0f%% of the "
      "setpoint magnitude, integrator clamped to the same bound. A wheel regulates only while its "
      "provenance on %s is LIVE (assume_live_provenance=%s), its feedback is newer than %.2f s, "
      "and it is not latched off by HWR-30a. SLOW-END FLOOR %.2f rad/s (FR-14 item 12): at or below "
      "that commanded magnitude the correction is exactly 0.0, the integrator is held reset and the "
      "wheel reports REGULATOR_OFF_BELOW_FLOOR. The floor IS stall_min_command_rad_s -- the "
      "regulator only adds authority where HWR-30a is watching -- so moving that parameter moves "
      "this floor. Corrections are published per wheel on %s.",
      regulator_config_.kp, regulator_config_.ki,
      regulator_config_.max_correction_fraction * 100.0, wheel_feedback_valid_topic_.c_str(),
      regulator_config_.assume_live_provenance ? "true" : "false",
      regulator_config_.max_sample_age_sec, stall_config_.min_command_rad_s,
      wheel_report_topic_.c_str());
  } else {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Wheel velocity regulator is DISABLED (default). The control law is open-loop feedforward "
      "and the regulator is never called: the value written to each wheel command interface is the "
      "kinematic setpoint itself. FR-11 item 2 / NFR-10 acceptance 10 hold. It can be switched on "
      "and off at runtime with `ros2 param set /swerve_controller wheel_regulator_enabled "
      "true|false`, effective on the next update() cycle%s.",
      regulator_resources_claimed_
        ? ""
        : " -- BUT NOT IN THIS CONFIGURATION: stall detection is off too, so the wheel POSITION "
          "state interfaces and the provenance subscription were never claimed and the switch will "
          "be REFUSED");
  }

  if (stall_config_.enabled) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Stall detection (HWR-30a tier 1) ON: trip when |command| > %.2f rad/s AND encoder "
      "provenance LIVE AND wheel POSITION moves < %.4f rad for %.2f s. Response: that wheel's "
      "velocity command -> 0, the other three and the steering untouched. Latch clears only when "
      "|command| falls to <= %.2f rad/s and rises again (OP-25 proposal). At most %zu wheel(s) "
      "latched at once -- more is tier 2 (HWR-30b, blocked). Provenance from %s (latched); "
      "assume_live_provenance=%s. ALL THRESHOLDS ARE TO-VERIFY (no GB37-50 stall-current "
      "measurement exists).",
      stall_config_.min_command_rad_s, stall_config_.min_position_delta_rad,
      stall_config_.window_sec, stall_config_.release_command_rad_s,
      stall_config_.max_latched_wheels, wheel_feedback_valid_topic_.c_str(),
      stall_config_.assume_live_provenance ? "true" : "false");
  } else {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Stall detection (HWR-30a) is DISABLED by parameter: a blocked drive motor will keep being "
      "commanded. The wheel POSITION state interfaces are not claimed either.");
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!resolve_interface_indices()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  steer_diff_omega_filtered_ = 0.0;
  last_wheel_command_.fill(0.0);
  last_wheel_correction_.fill(0.0);
  last_wheel_regulating_.fill(false);
  last_wheel_regulator_status_.fill(kRegulatorDisabled);
  regulator_at_limit_.fill(false);
  regulator_last_limit_log_sec_.fill(0.0);
  regulator_ever_limit_logged_.fill(false);
  // An integrator must NEVER survive a lifecycle transition either: it would
  // apply, on the first cycle back, an authority earned before the controller
  // stopped — against a robot that may have been moved, re-surfaced or unloaded
  // in between. It also forgets the last feedback sample, so the first cycle
  // after activation cannot integrate across the inactive period.
  wheel_regulator_.reset();
  // A stall latch must NEVER survive a lifecycle transition: coming back from
  // inactive with a wheel still silently disabled is the "motion, but degraded,
  // and no message" state SR-13 forbids, and nothing in the activation path
  // would report it.
  stall_detector_.reset();
  stall_last_log_sec_.fill(0.0);
  stall_ever_logged_.fill(false);
  stall_multi_wheel_condition_ = false;
  publish_stall_state(get_node()->get_clock()->now());
  // Drop anything that arrived while inactive, so the first update() cannot act
  // on a twist from before this activation.
  const StampedTwist empty_twist;
  const StampedDirectSteer empty_direct;
  cmd_vel_buffer_.writeFromNonRT(empty_twist);
  direct_steer_buffer_.writeFromNonRT(empty_direct);

  // SR-14, ITEM 1 — DELIBERATELY NOTHING IS WRITTEN HERE.
  // The requirement is that on activation the steering command interfaces hold
  // the servos' MEASURED position and the wheels are zero. That is already true
  // when this runs: GripperXInterface::on_activate() waits for one fresh, finite
  // /hw/steer_states sample and seeds it into both the position state AND the
  // position command interfaces, with the wheel commands zeroed (0ed3f56,
  // 17cb6d8). Measured on the bench for JointGroupPositionController: a
  // controller that merely activates does not overwrite the command interface.
  // A redundant hold here would be untested code on a safety path, and it would
  // move the guarantee into the layer that varies between real and sim — the
  // hardware component's half cannot fork, because GazeboSimSystem never loads
  // it. What IS in scope for this controller is its FIRST COMMANDED VALUE, and
  // that is handled in update(): a zero or stale /cmd_vel writes no steering at
  // all, so the seeded measurement survives it.
  RCLCPP_INFO(
    get_node()->get_logger(),
    "swerve_controller activated. No command written on activation (SR-14 item 1): the steering "
    "command interfaces keep the value the hardware component seeded from the measurement.");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Wheels to zero on the way out; steering is NOT written, so it holds
  // (OP-24 / S1: the steering is never moved except by a command). Writing zero
  // into the four steering POSITION command interfaces here would be a CENTRE
  // command at full servo torque, which is the exact defect §3.1.8 row 10
  // records against publish_zero_commands().
  const std::array<double, kNumWheels> zeros{0.0, 0.0, 0.0, 0.0};
  write_wheel_commands(get_node()->get_clock()->now(), zeros);
  wheel_regulator_.reset();
  stall_detector_.reset();
  alignment_gate_.reset();
  last_alignment_status_ = alignment_gate_.status();
  last_alignment_error_rad_ = 0.0;
  last_manoeuvre_ = ReportMsg::MANOEUVRE_IDLE;
  stall_multi_wheel_condition_ = false;
  publish_stall_state(get_node()->get_clock()->now());
  interfaces_resolved_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

void SwerveController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  StampedTwist stamped;
  stamped.twist = *msg;
  stamped.stamp = get_node()->get_clock()->now();
  stamped.sequence = cmd_vel_rx_count_.fetch_add(1) + 1;
  stamped.valid = true;
  cmd_vel_buffer_.writeFromNonRT(stamped);
}

void SwerveController::direct_steer_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() < kNumWheels) {
    return;  // same length guard as steer_servo_node._on_direct_steer
  }
  StampedDirectSteer stamped;
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    stamped.angles[i] = msg->data[i];
  }
  stamped.stamp = get_node()->get_clock()->now();
  stamped.valid = true;
  direct_steer_buffer_.writeFromNonRT(stamped);
}

void SwerveController::active_mode_callback(const std_msgs::msg::String::SharedPtr msg)
{
  mode_is_autonomous_.store(msg->data == autonomous_mode_name_);
}

void SwerveController::wheel_feedback_valid_callback(
  const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  // A message too short to carry all four wheels says NOTHING about any of them,
  // so it decodes to UNKNOWN for all four rather than to a partial verdict. Same
  // rule as GripperXInterface applies to a short /hw/joint_states (FR-11 item 5,
  // decode note 2): "the message did not say" must never collapse into a claim.
  std::array<int, kNumWheels> provenance{
    kStallProvenanceUnknown, kStallProvenanceUnknown, kStallProvenanceUnknown,
    kStallProvenanceUnknown};
  if (msg->data.size() >= kNumWheels) {
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      provenance[i] = msg->data[i];
    }
  } else {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 10000,
      "%s carries only %zu values (< %zu): wheel-feedback provenance is UNKNOWN for all four "
      "wheels, so stall detection (HWR-30a) stays DISARMED.",
      wheel_feedback_valid_topic_.c_str(), msg->data.size(), kNumWheels);
  }
  // Index i is wheel i in joint order FL, FR, BL, BR. That is the order
  // GripperXInterface publishes (kWheelJointOrder) and the order
  // wheel_joint_names carries -- asserted here in words because nothing in the
  // Int32MultiArray names its wheels, and a silent reordering would arm the
  // detector against the wrong encoder.
  wheel_provenance_buffer_.writeFromNonRT(provenance);
}

bool SwerveController::resolve_interface_indices()
{
  auto find_index = [](const auto & interfaces, const std::string & joint,
                       const std::string & interface_name, std::size_t & out) {
    for (std::size_t i = 0; i < interfaces.size(); ++i) {
      if (
        interfaces[i].get_prefix_name() == joint &&
        interfaces[i].get_interface_name() == interface_name) {
        out = i;
        return true;
      }
    }
    return false;
  };

  for (std::size_t i = 0; i < kNumWheels; ++i) {
    if (!find_index(
          command_interfaces_, steering_joint_names_[i], hardware_interface::HW_IF_POSITION,
          steer_cmd_index_[i]) ||
        !find_index(
          command_interfaces_, wheel_joint_names_[i], hardware_interface::HW_IF_VELOCITY,
          wheel_cmd_index_[i]) ||
        !find_index(
          state_interfaces_, steering_joint_names_[i], hardware_interface::HW_IF_POSITION,
          steer_state_index_[i]) ||
        !find_index(
          state_interfaces_, wheel_joint_names_[i], hardware_interface::HW_IF_VELOCITY,
          wheel_state_index_[i]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Could not resolve all command/state interfaces for wheel %zu (%s / %s).", i,
        steering_joint_names_[i].c_str(), wheel_joint_names_[i].c_str());
      interfaces_resolved_ = false;
      return false;
    }
    if (
      needs_wheel_position_state() &&
      !find_index(
        state_interfaces_, wheel_joint_names_[i], hardware_interface::HW_IF_POSITION,
        wheel_position_state_index_[i]))
    {
      // Claimed in state_interface_configuration(), so the resource manager has
      // already refused activation if it were genuinely absent -- reaching here
      // means the claim and the lookup disagree, which is a bug, not a platform
      // difference. FAIL rather than run a detector with an unresolved index:
      // silently disarming would leave "stall detection ON" in the configure log
      // while nothing was watching.
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Stall detection (HWR-30a, enabled=%s) or the wheel velocity regulator (enabled=%s) needs "
        "the wheel POSITION state interface for %s and it could not be resolved. Set "
        "stall_detection_enabled: false and wheel_regulator_enabled: false to run without it.",
        stall_config_.enabled ? "true" : "false", regulator_config_.enabled ? "true" : "false",
        wheel_joint_names_[i].c_str());
      interfaces_resolved_ = false;
      return false;
    }
  }
  interfaces_resolved_ = true;
  return true;
}

bool SwerveController::read_steering_feedback(std::array<double, kNumWheels> & joint_order_angles)
{
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    const auto value = state_interfaces_[steer_state_index_[i]].get_optional();
    if (!value.has_value()) {
      return false;
    }
    joint_order_angles[i] = value.value();
  }
  return all_finite(joint_order_angles);
}

const char * SwerveController::provenance_label(int provenance)
{
  switch (provenance) {
    case kStallProvenanceNoEncoder:
      return "NoEncoder";
    case kStallProvenanceInitFailed:
      return "InitFailed";
    case kStallProvenanceLiveUnconfirmed:
      return "LiveUnconfirmed";
    case kStallProvenanceLive:
      return "Live";
    default:
      return "UNKNOWN";
  }
}

rcl_interfaces::msg::SetParametersResult SwerveController::validate_parameter_update(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    const std::string & name = parameter.get_name();

    if (name == "wheel_regulator_enabled") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = "wheel_regulator_enabled must be a bool";
        return result;
      }
      // A switch that cannot work must be REFUSED, not accepted and quietly
      // ignored. The wheel POSITION state interfaces and the provenance
      // subscription are claimed at configure/activate time and cannot be
      // acquired afterwards, so a regulator that was disabled in the config
      // while stall detection was ALSO disabled has nothing to run on.
      if (parameter.as_bool() && !regulator_resources_claimed_) {
        result.successful = false;
        result.reason =
          "wheel_regulator_enabled cannot be turned on at runtime in this configuration: neither "
          "stall_detection_enabled nor wheel_regulator_enabled was true when the controller was "
          "configured, so the wheel POSITION state interfaces and the /hw/wheel_feedback_valid "
          "subscription were never claimed. Set wheel_regulator_enabled in the config and reload "
          "the controller.";
        return result;
      }
      continue;
    }

    // THE GAINS ARE NOT RUNTIME-SETTABLE, AND SILENTLY IGNORING A SET WOULD BE
    // WORSE THAN REFUSING IT. They are read once in on_init and copied into the
    // regulator's config; accepting a new value at runtime without applying it
    // would leave `ros2 param get` disagreeing with the running control law,
    // which is exactly the class of silent divergence SR-13 is about.
    if (
      name == "wheel_regulator_kp" || name == "wheel_regulator_ki" ||
      name == "wheel_regulator_max_correction_fraction" ||
      name == "wheel_regulator_max_sample_age_sec")
    {
      result.successful = false;
      result.reason =
        name +
        " takes effect only at (re)configure. Only wheel_regulator_enabled is runtime-settable "
        "(user decision 2026-08-20: the OFF switch must work on a driving robot). Change the "
        "value in ros2_controllers.yaml and reload the controller.";
      return result;
    }
  }

  return result;
}

void SwerveController::apply_parameter_update(
  const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "wheel_regulator_enabled") {
      // The ONLY thing the executor thread does. The reset, the reconfigure and
      // the log all happen in the control loop, on the cycle that observes the
      // change — so there is no window in which the executor and the control
      // loop hold different ideas of the regulator's state.
      regulator_enabled_rt_.store(parameter.as_bool());
    }
  }
}

void SwerveController::write_wheel_commands(
  const rclcpp::Time & time, const std::array<double, kNumWheels> & joint_order_rad_s,
  const std::array<double, kNumWheels> * steer_target,
  const std::array<double, kNumWheels> * steer_measured)
{
  // ===========================================================================
  // THE SINGLE CHOKE POINT. Every branch of update() and on_deactivate() reaches
  // the wheel command interfaces through here, so there is no path that skips
  // what follows. `joint_order_rad_s` is the REQUESTED command: the kinematic
  // feedforward, post wheel_command_multipliers.
  //
  // THREE STAGES ACT ON IT, IN THIS ORDER, AND THE ORDER IS THE SAFETY PROPERTY:
  //
  //   1. THE VELOCITY REGULATOR may add a BOUNDED CORRECTION to each wheel
  //      (FR-11; wheel_regulator.hpp). DISABLED BY DEFAULT — while
  //      `wheel_regulator_enabled` is false this stage is not entered at all,
  //      `effective` remains a bit-identical copy of the requested command, and
  //      the control law is open-loop exactly as FR-11 item 2 and NFR-10
  //      acceptance 10 require. Nothing here rounds, scales or re-adds anything
  //      to it on the way past.
  //
  //   2. THE STEERING ALIGNMENT GATE may replace ALL FOUR commands with exactly
  //      0.0 while the modules are still travelling into a new pose
  //      (alignment_gate.hpp). DISABLED BY DEFAULT. It sits AFTER the regulator
  //      for the same reason stage 3 does — a zero that a correction can still be
  //      added to is not a zero — and BEFORE stage 3 so that HWR-30a keeps the
  //      last word, which is the property its own contract is written on.
  //      Unlike stage 1 it reads no measured VELOCITY and can only ever write
  //      exactly 0.0, so it closes no loop and leaves NFR-10 acceptance 10 alone.
  //
  //   3. HWR-30a TIER 1 comes LAST and it is the last thing that touches the
  //      command. The only thing it may do is replace a latched wheel's command
  //      with EXACTLY 0.0 — the same authority limit A3 carries in §3.1.3.
  //      Because it runs after the regulator, no correction can ever stand
  //      between a latched wheel and its zero, and the exactness of that zero is
  //      unaffected by whether the regulator ran.
  //
  // THE TWO STAGES SHARE ONE THRESHOLD, AND IT IS A SAFETY COUPLING RATHER THAN
  // A SHORTCUT: stage 1's slow-end floor (FR-14 item 12) IS stall_min_command_rad_s,
  // so the regulator only ever adds authority inside the band where stage 2 is
  // armed and watching. Below it stage 1 holds its correction at exactly 0.0 and
  // reports kRegulatorOffBelowFloor. The floor is read from stall_config_ per
  // cycle, never copied, so the two can never disagree about the one number.
  //
  // The detector JUDGES the REQUESTED command, not the regulated one, and that
  // is deliberate: OP-25's release edge and the arming threshold
  // stall_min_command_rad_s are defined on what was ASKED FOR. The floor is
  // compared against that same requested command for the same reason, and with
  // the same strict `>` the detector arms with, so the boundary case cannot fall
  // between the two. Feeding it a
  // command that a regulator has already moved by up to 30 % would shift a
  // safety threshold by the same 30 %. Its verdict is then applied to whatever
  // the regulator produced. With the regulator off `effective` IS the requested
  // command, so this is bit-identical to assigning the detector's own
  // `result.commands` — which is what this function did before the regulator
  // existed.
  //
  // WHAT NEITHER STAGE TOUCHES, deliberately:
  //  * the other three wheels -- tier 1 is a SINGLE-motor response and is
  //    explicitly NOT a hard stop of the machine (HWR-30);
  //  * the steering, in any branch (OP-24 / S1);
  //  * the intent echo, which keeps reporting the CONSUMED TWIST. The W2
  //    watchdog (OP-18a, SR-11) compares /cmd_vel against that echo -- twist
  //    against twist -- and counts the echo's sequence. Gating or trimming a
  //    wheel command is invisible to it, which is correct: the echo answers "is
  //    the controller alive", not "is that wheel turning";
  //  * the SR-14 activation gate -- nothing new is written on activation, the
  //    detector and the regulator are only reset there.
  // ===========================================================================
  const std::array<bool, kNumWheels> was_regulating = last_wheel_regulating_;
  std::array<double, kNumWheels> effective = joint_order_rad_s;
  last_wheel_correction_.fill(0.0);
  last_wheel_regulating_.fill(false);
  last_wheel_regulator_status_.fill(kRegulatorDisabled);

  // ------------------------------------------- RUNTIME ENABLE/DISABLE EDGE
  // USER DECISION 2026-08-20: the switch takes effect on the NEXT update()
  // cycle, on a live driving robot, in both directions.
  //
  // BOTH EDGES RESET. configure() clears every integrator and forgets the last
  // feedback sample, so:
  //  * turning it OFF cannot leave a correction standing, and
  //  * turning it ON later cannot apply authority earned before it was switched
  //    off. The disabled path does not integrate at all — update() is not called
  //    while regulator_active_ is false, so there is no hidden state to carry.
  //
  // THE OFF TRANSITION IS A STEP, NOT A RAMP, AND THAT IS DELIBERATE. The step
  // is exactly the correction that was standing, which the authority bound caps
  // at max_correction_fraction (30 %) of the setpoint — the same size of step
  // the machine already takes whenever a wheel loses provenance or its feedback
  // goes stale, and small against the slew brake's own 0.45x scale steps. A ramp
  // would need a rate constant nobody has measured, and it would mean the OFF
  // switch does not fully take effect on the cycle it is pressed, which is
  // precisely what this decision was made to avoid: an off-switch that is still
  // partly on is not an off-switch.
  const bool regulator_requested = regulator_enabled_rt_.load();
  const bool regulator_turned_on = regulator_requested && !regulator_active_;
  const bool regulator_turned_off = !regulator_requested && regulator_active_;
  if (regulator_turned_on || regulator_turned_off) {
    WheelRegulatorConfig live_config = regulator_config_;
    live_config.enabled = regulator_requested;
    wheel_regulator_.configure(live_config);  // configure() resets
    regulator_active_ = regulator_requested;
    regulator_at_limit_.fill(false);
  }

  // interfaces_resolved_ is part of every condition below because the wheel
  // POSITION index map is only meaningful after on_activate resolved it;
  // on_deactivate can otherwise reach this with an unresolved map.
  const bool have_position_state = needs_wheel_position_state() && interfaces_resolved_;

  // ONE READ OF THE SHARED INPUTS, used by both stages. The latched provenance
  // and the accumulated wheel position are read here rather than twice, so the
  // two stages can never act on two different views of the same cycle.
  std::array<int, kNumWheels> provenance{
    kStallProvenanceUnknown, kStallProvenanceUnknown, kStallProvenanceUnknown,
    kStallProvenanceUnknown};
  std::array<double, kNumWheels> wheel_position{};
  std::array<bool, kNumWheels> wheel_position_valid{};
  if (have_position_state) {
    provenance = *wheel_provenance_buffer_.readFromRT();
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      const auto value = state_interfaces_[wheel_position_state_index_[i]].get_optional();
      wheel_position_valid[i] = value.has_value();
      wheel_position[i] = value.has_value() ? value.value() : 0.0;
    }
  }

  // ------------------------------------------- STAGE 1: velocity regulator
  std::array<double, kNumWheels> regulator_measured{};
  if (regulator_active_ && interfaces_resolved_) {
    WheelRegulatorInput input;
    input.now_sec = time.seconds();
    input.setpoint = joint_order_rad_s;
    input.position = wheel_position;
    input.position_valid = wheel_position_valid;
    input.provenance = provenance;
    // THE SLOW-END FLOOR (FR-14 item 12, A17) IS HWR-30a's ARMING THRESHOLD, and
    // this line is the whole coupling: the value is read out of the DETECTOR's
    // own config, every cycle, so the floor cannot drift away from the threshold
    // the detector arms on. The regulator may only add authority where HWR-30a is
    // watching — the rationale is at the floor gate in wheel_regulator.cpp.
    // NOTE THE SCOPE OF THE COUPLING: it is to the THRESHOLD VALUE, not to
    // stall_detection_enabled. With the detector switched off the threshold is
    // still configured and still floors the regulator, but nothing is watching
    // above it either; that combination is not a configuration anything ships.
    input.stall_min_command_rad_s = stall_config_.min_command_rad_s;
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      const auto measured = state_interfaces_[wheel_state_index_[i]].get_optional();
      input.measured_valid[i] = measured.has_value();
      input.measured[i] = measured.has_value() ? measured.value() : 0.0;
      regulator_measured[i] = input.measured[i];
      // The latch state as it stands BEFORE stage 2 runs, i.e. the verdict of
      // the PREVIOUS cycle. The one-cycle lag is in the safe direction and is
      // load-bearing in only one place: on the cycle a wheel trips, the
      // regulator has already produced a correction for it — and stage 2 then
      // overwrites the whole command with 0.0 anyway, so the lag cannot reach
      // the actuator. From the next cycle until the latch clears, the wheel gets
      // no correction and its integrator is held reset, which is what stops a
      // wound-up correction being handed to the wheel the moment it un-latches.
      input.stall_latched[i] = stall_detector_.latched(i);
    }

    const auto result = wheel_regulator_.update(input);
    effective = result.commands;
    last_wheel_correction_ = result.correction;
    last_wheel_regulating_ = result.regulating;
    last_wheel_regulator_status_ = result.status;
  }

  // ------------------------------------- STAGE 2: STEERING ALIGNMENT GATE
  // Both pointers null means no steering command is being written on this cycle
  // (OP-24/S1 hold, the no-feedback branch, on_deactivate). Those branches have
  // already zeroed the wheels for their own reasons, so there is nothing for the
  // gate to withhold and no new target to align to.
  {
    const std::array<double, kNumWheels> no_angles{0.0, 0.0, 0.0, 0.0};
    const bool target_written = (steer_target != nullptr) && (steer_measured != nullptr);
    const auto result = alignment_gate_.update(
      time.seconds(), effective, target_written ? *steer_target : no_angles,
      target_written ? *steer_measured : no_angles, target_written);
    effective = result.commands;
    last_alignment_status_ = result.status;
    last_alignment_error_rad_ = result.max_error_rad;

    // EDGE-TRIGGERED, never per cycle, and rate-limited across the hysteresis
    // band. The line names the error because "the drive is being withheld" is
    // only actionable next to "and the modules are still 47 deg out".
    if (result.state_changed) {
      const double now = time.seconds();
      const bool rate_ok = !alignment_ever_logged_ ||
                           (now - alignment_last_log_sec_) >= alignment_log_period_sec_ ||
                           now < alignment_last_log_sec_;
      if (rate_ok) {
        alignment_ever_logged_ = true;
        alignment_last_log_sec_ = now;
        if (result.status == kAlignSlewing) {
          RCLCPP_INFO(
            get_node()->get_logger(),
            "ALIGNMENT GATE ENGAGED: the commanded steering pose moved away from where the "
            "modules stand (worst wheel %+.1f deg out) -- drive held at zero on all four wheels "
            "until they arrive, or for %.2f s.",
            result.max_error_rad * kRadToDeg, alignment_config_.timeout_sec);
        } else if (result.status == kAlignTimedOut) {
          RCLCPP_WARN(
            get_node()->get_logger(),
            "ALIGNMENT GATE RELEASED ON THE TIMEOUT after %.2f s: the modules did not report "
            "reaching the pose (worst wheel still %+.1f deg out). Drive is flowing again, but "
            "the pose is NOT confirmed. On this machine /hw/steer_states reports the SERVO, not "
            "the wheel -- OP-29 measured up to 17 deg of play between them in a crab.",
            alignment_config_.timeout_sec, result.max_error_rad * kRadToDeg);
        } else if (result.status == kAlignPassing) {
          RCLCPP_INFO(
            get_node()->get_logger(),
            "ALIGNMENT GATE RELEASED: modules in pose (worst wheel %+.1f deg), drive restored.",
            result.max_error_rad * kRadToDeg);
        }
      }
    }
  }

  // ------------------------------------------- STAGE 3: HWR-30a, TIER 1, LAST
  if (stall_config_.enabled && interfaces_resolved_) {
    stall_detector_.set_provenance(provenance);
    const auto result = stall_detector_.update(
      time.seconds(), joint_order_rad_s, wheel_position, wheel_position_valid);
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      if (stall_detector_.latched(i)) {
        effective[i] = 0.0;
        // Reported as zero effort, because zero effort is what was WRITTEN. The
        // report must describe the command, not the intention behind it.
        last_wheel_correction_[i] = 0.0;
        last_wheel_regulating_[i] = false;
        // The reason must survive the gate, or a latched wheel would report
        // "disabled" while the regulator is running (FR-14 A7 + item 7).
        if (regulator_active_) {
          last_wheel_regulator_status_[i] = kRegulatorOffStallLatched;
        }
      }
    }
    log_stall_events(result, joint_order_rad_s, time);
    if (result.state_changed) {
      publish_stall_state(time);
    }
  }

  // ------------------------------------------- THE AUTHORITY-LIMIT LOG
  // FR-14 item 8: "reaching [the limit] MUST be reported, not silently clipped".
  // The published per-wheel status is the primary report; this line exists for
  // the human watching a terminal during the first enable, and it carries the
  // three numbers needed to distinguish SATURATION from a SIGN ERROR, which look
  // identical in magnitude: with the sign correct, `correction` has the SAME sign
  // as (setpoint - measured). EDGE-TRIGGERED, never per cycle.
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    const bool at_limit =
      regulator_active_ && last_wheel_regulator_status_[i] == kRegulatorAtAuthorityLimit;
    if (at_limit && !regulator_at_limit_[i]) {
      const double now = time.seconds();
      const bool rate_ok = !regulator_ever_limit_logged_[i] ||
                           (now - regulator_last_limit_log_sec_[i]) >= regulator_log_period_sec_ ||
                           now < regulator_last_limit_log_sec_[i];
      if (rate_ok) {
        regulator_ever_limit_logged_[i] = true;
        regulator_last_limit_log_sec_[i] = now;
        RCLCPP_WARN(
          get_node()->get_logger(),
          "REGULATOR AT AUTHORITY LIMIT: %s setpoint %+.3f rad/s, measured %+.3f rad/s, correction "
          "%+.3f rad/s (the full %.0f%% bound), commanded %+.3f rad/s. The feedforward is wrong for "
          "this surface by more than the regulator may trim (OP-32) -- this is EXPECTED, not a "
          "fault, and the residual error will NOT go to zero. SIGN CHECK: the correction must have "
          "the SAME sign as (setpoint - measured) = %+.3f; if it does not, this is a sign error and "
          "the regulator is making it worse.",
          wheel_joint_names_[i].c_str(), joint_order_rad_s[i], regulator_measured[i],
          last_wheel_correction_[i], regulator_config_.max_correction_fraction * 100.0,
          effective[i], joint_order_rad_s[i] - regulator_measured[i]);
      }
    } else if (!at_limit && regulator_at_limit_[i]) {
      RCLCPP_INFO(
        get_node()->get_logger(),
        "Regulator left the authority limit on %s: correction %+.3f rad/s is inside the bound "
        "again.",
        wheel_joint_names_[i].c_str(), last_wheel_correction_[i]);
    }
    regulator_at_limit_[i] = at_limit;
  }

  // ------------------------------------------- THE TRANSITION LOG
  // The first enable on hardware happens with a human watching a terminal, so it
  // must be obvious FROM THE LOG ALONE whether the regulator is actually acting.
  // Both the regulating flags and the raw provenance code are printed per wheel:
  // "enabled" and "acting" are different statements, and the usual reason for the
  // gap is provenance below Live on a wheel whose encoder never counted.
  if (regulator_turned_on || regulator_turned_off) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "WHEEL VELOCITY REGULATOR %s at runtime (parameter wheel_regulator_enabled). %s Per wheel "
      "now: %s regulating=%s provenance=%s | %s regulating=%s provenance=%s | %s regulating=%s "
      "provenance=%s | %s regulating=%s provenance=%s. Corrections are published on %s.",
      regulator_turned_on ? "ENABLED" : "DISABLED",
      regulator_turned_on
        ? "Every integrator starts at zero; corrections are bounded to the authority fraction of "
          "the setpoint. The control law is NO LONGER OPEN-LOOP (cf. FR-11 item 2)."
        : "Every correction went to zero on this cycle -- a step, bounded by the authority "
          "fraction -- and every integrator is reset, so a later re-enable starts clean. The "
          "control law is open-loop again.",
      wheel_joint_names_[0].c_str(),
      (regulator_turned_on ? last_wheel_regulating_[0] : was_regulating[0]) ? "yes" : "no",
      provenance_label(provenance[0]), wheel_joint_names_[1].c_str(),
      (regulator_turned_on ? last_wheel_regulating_[1] : was_regulating[1]) ? "yes" : "no",
      provenance_label(provenance[1]), wheel_joint_names_[2].c_str(),
      (regulator_turned_on ? last_wheel_regulating_[2] : was_regulating[2]) ? "yes" : "no",
      provenance_label(provenance[2]), wheel_joint_names_[3].c_str(),
      (regulator_turned_on ? last_wheel_regulating_[3] : was_regulating[3]) ? "yes" : "no",
      provenance_label(provenance[3]), wheel_report_topic_.c_str());
  }

  for (std::size_t i = 0; i < kNumWheels; ++i) {
    if (!command_interfaces_[wheel_cmd_index_[i]].set_value(effective[i])) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Could not write wheel velocity command for %s.", wheel_joint_names_[i].c_str());
    }
    // The REPORTED commanded value is what was WRITTEN, not what was requested
    // (FR-11 item 1: "exactly the value written to the wheel VELOCITY command
    // interface"). A latched wheel therefore reports commanded 0.0, and
    // stall_latched says why; a regulated wheel reports the corrected value, and
    // `correction` says how much of it the regulator added.
    last_wheel_command_[i] = effective[i];
  }
}

void SwerveController::log_stall_events(
  const StallDetectorResult & result, const std::array<double, kNumWheels> & requested,
  const rclcpp::Time & time)
{
  const double now = time.seconds();
  bool multi_wheel = false;

  for (std::size_t i = 0; i < kNumWheels; ++i) {
    const auto & event = result.events[i];
    if (event.multi_wheel_refused) {
      multi_wheel = true;
    }

    // EDGE-TRIGGERED, never per cycle: these flags are true only on the cycle of
    // the transition, and the latch itself makes a trip a once-per-blockage
    // event. The extra per-wheel rate limit bounds a pathological trip/release
    // oscillation. It is NOT RCLCPP_*_THROTTLE on purpose -- that macro keeps
    // its state per CALL SITE, so one wheel tripping would suppress the ERROR
    // line of another wheel tripping in the same second, and "which wheel" is
    // the entire point of a per-wheel detector.
    const bool rate_ok = !stall_ever_logged_[i] ||
                         (now - stall_last_log_sec_[i]) >= stall_log_period_sec_ ||
                         now < stall_last_log_sec_[i];

    if (event.tripped && rate_ok) {
      stall_ever_logged_[i] = true;
      stall_last_log_sec_[i] = now;
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "STALL (HWR-30a tier 1): %s commanded %+.2f rad/s but its encoder position moved less "
        "than %.4f rad for %.2f s. That motor is now LATCHED OFF (velocity command forced to 0); "
        "the other three keep running and the steering is untouched. It stays off until its "
        "command drops to <= %.2f rad/s and rises again (OP-25). Trip #%u for this wheel since "
        "activation. Thresholds are TO-VERIFY.",
        wheel_joint_names_[i].c_str(), requested[i], stall_config_.min_position_delta_rad,
        stall_config_.window_sec, stall_config_.release_command_rad_s,
        stall_detector_.trip_count(i));
    }
    if (event.released) {
      RCLCPP_INFO(
        get_node()->get_logger(),
        "STALL cleared (HWR-30a): %s received a fresh command after its stall latch and is being "
        "driven again.",
        wheel_joint_names_[i].c_str());
    }
  }

  if (multi_wheel) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "MULTI-WHEEL STALL CONDITION: more wheels satisfy the stall criterion than tier 1 may latch "
      "(cap %zu). NO further wheel has been switched off. 'More than one motor' is HWR-30 TIER 2, "
      "which is BLOCKED on the unmeasured GB37-50 stall current (WIRING_PLAN 8.5) and is NOT "
      "implemented. The same signature is produced by a LOST /hw/joint_states -- every wheel "
      "position freezes at once -- which the hardware component's own state_timeout_sec handles.",
      stall_config_.max_latched_wheels);
  }

  if (multi_wheel != stall_multi_wheel_condition_) {
    stall_multi_wheel_condition_ = multi_wheel;
    publish_stall_state(time);
  }
}

void SwerveController::publish_stall_state(const rclcpp::Time & time)
{
  if (!stall_state_pub_) {
    return;
  }
  gripperx_control_msgs::msg::WheelStallState msg;
  msg.header.stamp = time;
  msg.joint_names = wheel_joint_names_;
  msg.latched.resize(kNumWheels);
  msg.armed.resize(kNumWheels);
  msg.trip_count.resize(kNumWheels);
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    msg.latched[i] = stall_detector_.latched(i);
    msg.armed[i] = stall_detector_.armed(i);
    msg.trip_count[i] = stall_detector_.trip_count(i);
  }
  msg.multi_wheel_condition = stall_multi_wheel_condition_;
  stall_state_pub_->publish(msg);
}

void SwerveController::write_steering_commands(
  const std::array<double, kNumWheels> & joint_order_rad)
{
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    if (!command_interfaces_[steer_cmd_index_[i]].set_value(joint_order_rad[i])) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Could not write steering position command for %s.", steering_joint_names_[i].c_str());
    }
  }
}

double SwerveController::steer_alignment_scale(double target_angle, double current_angle) const
{
  const double error = std::fabs(normalize_angle(target_angle - current_angle));
  if (error <= steer_alignment_deadband_rad_) {
    return 1.0;
  }
  return std::max(steer_alignment_min_scale_, 1.0 - (error / steer_alignment_reference_rad_));
}

std::array<double, kNumWheels> SwerveController::apply_steer_feedback_differential(
  const BodyTwist & desired_body_twist,
  const std::array<double, kNumWheels> & current_steering_angles_model,
  const std::array<double, kNumWheels> & wheel_angular_speeds_model, double dt)
{
  // Gate: only active below steer_diff_omega_gate, so that the autonomy path
  // (Nav2 -> cmd_vel with omega != 0), where the IK already computes the
  // differential from the desired twist, is not corrected a second time.
  if (std::fabs(desired_body_twist.omega) > steer_diff_omega_gate_) {
    steer_diff_omega_filtered_ = 0.0;
    return wheel_angular_speeds_model;
  }

  double sum = 0.0;
  for (const double speed : wheel_angular_speeds_model) {
    sum += speed;
  }
  const double nominal_linear = wheel_radius_ * (sum / static_cast<double>(kNumWheels));
  // COUPLED TO THE BRAKING REFERENCE, and it is the only place the two 2026-08-19
  // decisions touch. The speeds arriving here are POST-scale, so this gate sees
  // scale * commanded speed. With the old IK reference, keyboard cornering was
  // pinned at scale 0.45 and this gate opened above 0.0667 m/s commanded; with
  // the commanded reference the settled scale is 1.0 and it opens above
  // 0.030 m/s. In the band between, the differential now engages where it
  // previously did not. Unreachable from the keyboard (linear_vel_m_s is fixed
  // at 0.5 m/s), reachable from Nav2 — measure it there, do not pre-tune it.
  if (std::fabs(nominal_linear) < steer_diff_min_speed_mps_) {
    steer_diff_omega_filtered_ = 0.0;
    return wheel_angular_speeds_model;
  }

  const std::array<double, kNumWheels> uniform_speeds{
    nominal_linear, nominal_linear, nominal_linear, nominal_linear};
  const BodyTwist synthetic =
    model_->forward_kinematics_body(current_steering_angles_model, uniform_speeds);

  const double alpha = (dt > 0.0 && steer_diff_time_constant_sec_ > 0.0)
                         ? dt / (steer_diff_time_constant_sec_ + dt)
                         : 1.0;
  steer_diff_omega_filtered_ += alpha * (synthetic.omega - steer_diff_omega_filtered_);
  const double omega_estimate = std::max(
    -steer_diff_max_omega_, std::min(steer_diff_max_omega_, steer_diff_omega_filtered_));

  const auto ideal_commands =
    model_->inverse_kinematics(BodyTwist{nominal_linear, 0.0, omega_estimate});
  const auto targets =
    resolve_wheel_targets(ideal_commands, current_steering_angles_model, model_steering_limits_);
  if (!targets.has_value()) {
    // The reconstructed pose is not steerable within the per-wheel windows, so
    // its speed split would not describe the pose the wheels are in. Leave the
    // uniform speeds alone rather than differentiate on a fiction.
    return wheel_angular_speeds_model;
  }

  std::array<double, kNumWheels> new_speeds{};
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    double angular_speed = (*targets)[i].speed / wheel_radius_;
    const double nominal_speed = wheel_angular_speeds_model[i];
    // Conservative limiting: no wheel is braked/accelerated more strongly than
    // [min_ratio, max_ratio] * the originally uniform speed — a safety net
    // against noisy/faulty servo feedback.
    double lo = nominal_speed * steer_diff_min_ratio_;
    double hi = nominal_speed * steer_diff_max_ratio_;
    if (nominal_speed < 0.0) {
      lo = nominal_speed * steer_diff_max_ratio_;
      hi = nominal_speed * steer_diff_min_ratio_;
    }
    new_speeds[i] = std::max(lo, std::min(hi, angular_speed));
  }
  return new_speeds;
}

void SwerveController::report_limit_status(const LimitedTwist & limited)
{
  if (limited.status == LimitStatus::kOk) {
    return;
  }

  std::string detail;
  for (const auto & violation : limited.violations) {
    if (!detail.empty()) {
      detail += "; ";
    }
    detail += violation.describe();
  }
  if (detail.empty()) {
    detail = "no wheel solution";
  }

  if (limited.status == LimitStatus::kOmegaReduced) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Steering limit: omega %.3f -> %.3f rad/s at vx=%.2f vy=%.2f m/s (%s). Same manoeuvre, "
      "wider radius.",
      limited.requested_omega, limited.twist.omega, limited.twist.vx, limited.twist.vy,
      detail.c_str());
    return;
  }

  // Silent clamping is the failure mode the whole limit machinery exists to
  // remove (§3.1.4 (a) item 2), so a rejected request must never pass
  // unremarked.
  RCLCPP_ERROR_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 2000,
    "Steering limit: twist REJECTED, holding steering and commanding zero drive -- the requested "
    "direction of travel is unreachable even without rotation (%s). PURE sideways travel (crab, "
    "vy only) is NOT in this category: it resolves via the +-180 module flip and is what the "
    "teleop arrow keys use (FR-7). What lands here are steep diagonals (FR-8) and directions no "
    "module solution fits.",
    detail.c_str());
}

void SwerveController::publish_intent_echo(
  const rclcpp::Time & time, const StampedTwist & consumed)
{
  if (!intent_echo_pub_ || !intent_echo_pub_->trylock()) {
    return;
  }
  auto & msg = intent_echo_pub_->msg_;
  msg.header.stamp = time;
  msg.header.frame_id.clear();
  msg.twist = consumed.twist;
  msg.sequence = consumed.sequence;
  intent_echo_pub_->unlockAndPublish();
}

void SwerveController::publish_wheel_report(const rclcpp::Time & time)
{
  if (!wheel_report_pub_ || !wheel_report_pub_->trylock()) {
    return;
  }
  auto & msg = wheel_report_pub_->msg_;
  msg.header.stamp = time;
  msg.header.frame_id.clear();
  msg.joint_names = wheel_joint_names_;
  msg.commanded.resize(kNumWheels);
  msg.measured.resize(kNumWheels);
  msg.stall_latched.resize(kNumWheels);
  msg.stall_trip_count.resize(kNumWheels);
  msg.correction.resize(kNumWheels);
  msg.regulating.resize(kNumWheels);
  msg.regulator_status.resize(kNumWheels);
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    msg.commanded[i] = last_wheel_command_[i];
    // FR-11, THE FL-27 % LESSON: the effort a regulator applies MUST be a
    // first-class published signal, not something a reader infers from the gap
    // between commanded and measured. A regulator masks exactly the structural
    // faults the diagnostics rely on -- a loose connection on one wheel simply
    // gets driven harder until the numbers agree -- so the correction is the
    // signal that keeps the fault visible while it is being compensated.
    // Exactly 0.0 / false on every wheel while the regulator is disabled.
    msg.correction[i] = last_wheel_correction_[i];
    msg.regulating[i] = last_wheel_regulating_[i];
    msg.regulator_status[i] = static_cast<uint8_t>(last_wheel_regulator_status_[i]);
    const auto measured = state_interfaces_[wheel_state_index_[i]].get_optional();
    msg.measured[i] = measured.has_value() ? measured.value()
                                           : std::numeric_limits<double>::quiet_NaN();
    // HWR-30a rides along with commanded/measured because "commanded 7 rad/s,
    // measured 0 rad/s" and "this wheel is latched off" are two halves of one
    // reading. The LATCHED copy is /swerve_controller/stall_state.
    msg.stall_latched[i] = stall_detector_.latched(i);
    msg.stall_trip_count[i] = stall_detector_.trip_count(i);
  }
  msg.alignment_status = static_cast<uint8_t>(last_alignment_status_);
  msg.alignment_max_error_rad = last_alignment_error_rad_;
  msg.manoeuvre = static_cast<uint8_t>(last_manoeuvre_);
  wheel_report_pub_->unlockAndPublish();
}

controller_interface::return_type SwerveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!interfaces_resolved_) {
    return controller_interface::return_type::ERROR;
  }

  const StampedTwist consumed = *cmd_vel_buffer_.readFromRT();
  const StampedDirectSteer direct = *direct_steer_buffer_.readFromRT();

  // Default for every branch that returns early. All of them command zero drive,
  // so IDLE is what they are: reporting the last named manoeuvre through a hold
  // would say the robot is still crabbing while its wheels are commanded to zero.
  last_manoeuvre_ = ReportMsg::MANOEUVRE_IDLE;

  const std::array<double, kNumWheels> zeros{0.0, 0.0, 0.0, 0.0};

  // ------------------------------------------------------------------ stage 2
  // OP-24 / S1 stage 2 — THREE cases, and this is deliberately NOT a copy of
  // swerve_cmd_node. That node acts on staleness only, and the stale branch
  // NEVER EXECUTES in normal operation: teleop_mux publishes a default-
  // constructed Twist at 20 Hz whenever no source passes freshness
  // (teleop_mux_node.py L134), so /cmd_vel never goes stale at idle. The
  // centring came from the CONTENT — atan2(0, 0) = 0 rad on every wheel — and
  // was MEASURED driving real wheels from up to 3.43 deg off centre to within
  // 0.09-0.79 deg of it, seconds after startup, on 2026-08-19.
  //   (i)   fresh non-zero twist -> normal IK
  //   (ii)  fresh ZERO twist     -> wheels zero, steering HOLDS
  //   (iii) genuinely stale      -> wheels zero, steering HOLDS
  // "Holds" means NOT WRITING the four steering command interfaces on that
  // cycle. No latch, no stored setpoint, no re-issued value: they keep what was
  // last written to them, and on the very first cycle that is the measured
  // position the hardware component seeded at activation (SR-14).
  //
  // The zero test is EXACT — vx == vy == omega == 0.0 on the RECEIVED Twist
  // (user decision 2026-08-19). No tolerance, no epsilon, no new parameter.
  // Recorded re-open trigger: Nav2 / DWB emits near-zero-but-not-zero twists,
  // so the epsilon gets MEASURED when the autonomous path is first driven. Do
  // not introduce one before then.
  const double cmd_age =
    consumed.valid ? (time - consumed.stamp).seconds() : std::numeric_limits<double>::infinity();
  const bool cmd_stale = !consumed.valid || cmd_age > cmd_vel_timeout_sec_;
  const bool cmd_is_zero = consumed.valid && consumed.twist.linear.x == 0.0 &&
                           consumed.twist.linear.y == 0.0 && consumed.twist.angular.z == 0.0;

  // ------------------------------------------------------------------- A2
  // OP-23 / A2-b: the /teleop/direct_steer override wins while fresh AND while
  // the active mode is not `autonomous`, then falls back. Same three-way rule
  // as steer_servo_node had, now in ONE place for real and sim. Strict `<`
  // and the mode gate are carried over verbatim from steer_servo_node L749/751.
  const bool direct_fresh =
    direct.valid && !mode_is_autonomous_.load() &&
    ((time - direct.stamp).seconds() < direct_timeout_sec_);

  std::array<double, kNumWheels> steering_joint_order{};
  bool write_steering = false;

  if (direct_fresh) {
    // The override is a COMMAND, so it moves the steering in every case — that
    // is what makes the spacebar E-stop's centring half and FR-13's dedicated
    // centring command work while the robot is idle and /cmd_vel is a stream of
    // zeros. It is clamped against the same window the IK plans in; the
    // calibrated clamp in steer_servo_node stays untouched behind it as SR-6's
    // last line of defence, and an intervention here is logged, never silent.
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      const double requested = direct.angles[i];
      steering_joint_order[i] = joint_steering_limits_.clamp(i, requested);
      if (std::fabs(steering_joint_order[i] - requested) > kAngleToleranceRad) {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "direct_steer override: %s requested %+.1fdeg, clamped to %+.1fdeg (window "
          "[%+.1f, %+.1f]deg).",
          joint_steering_limits_.label(i), requested * kRadToDeg,
          steering_joint_order[i] * kRadToDeg, joint_steering_limits_.lower(i) * kRadToDeg,
          joint_steering_limits_.upper(i) * kRadToDeg);
      }
    }
    write_steering = true;
  }

  std::array<double, kNumWheels> current_steering_joint{};
  if (!read_steering_feedback(current_steering_joint)) {
    // No usable steering feedback: the limit resolution and the slew braking
    // both depend on it, so nothing about the drive can be planned. Wheels to
    // zero, steering held. swerve_cmd_node simply returned here, which in a
    // controller would leave the LAST wheel command standing on the interface
    // and keep the robot driving — so this branch is deliberately stronger than
    // the node it replaces.
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "No valid steering position feedback on the state interfaces -- wheels commanded to zero, "
      "steering held.");
    write_wheel_commands(time, zeros);
    if (write_steering) {
      write_steering_commands(steering_joint_order);
    }
    publish_intent_echo(time, consumed);
    publish_wheel_report(time);
    return controller_interface::return_type::OK;
  }

  const auto current_steering_model = to_model_order(current_steering_joint);

  if (cmd_stale || cmd_is_zero) {
    // Cases (ii) and (iii). Behaviourally identical at the actuators; listed
    // apart in OP-24 because an implementer reading only the old text would
    // build (iii) and ship the defect.
    write_wheel_commands(time, zeros);
    if (write_steering) {
      write_steering_commands(steering_joint_order);
    }
    publish_intent_echo(time, consumed);
    publish_wheel_report(time);
    return controller_interface::return_type::OK;
  }

  // ------------------------------------------------------- case (i): normal IK
  BodyTwist desired{};
  if (enforce_front_forward_ && !allow_reverse_) {
    desired.vx = std::max(0.0, consumed.twist.linear.x);
    desired.vy = 0.0;
    desired.omega = consumed.twist.angular.z;
  } else {
    desired.vx = consumed.twist.linear.x;
    desired.vy = enforce_front_forward_ ? 0.0 : consumed.twist.linear.y;
    desired.omega = consumed.twist.angular.z;
  }

  // enable_point_turn is FALSE in the active configuration and the tank-turn
  // path (_compute_point_turn) is therefore NOT ported — stated here rather
  // than silently omitted (NFR-11). Pure rotation uses the SWERVE spin, whose
  // pose (+-58.570 deg outward on every wheel, = atan2(a, b)) sits inside the
  // 100 deg outward window and is reached through the ordinary IK below. The
  // +-50.7 this line carried until 2026-08-21 is the RETIRED b = 0.16556
  // geometry and overstates the remaining steering margin by 8 deg. The Python
  // tracking-controller path (use_direct_ik: false) is likewise not ported.

  const auto limited =
    limit_twist_to_steering_range(*model_, desired, current_steering_model, model_steering_limits_);
  report_limit_status(limited);

  if (limited.status == LimitStatus::kRejected) {
    // No reachable pose for this direction of travel. Zero drive, and the
    // steering HOLDS — moving in some other direction than the one requested
    // would be worse than not moving. swerve_cmd_node re-published the MEASURED
    // angles here; under OP-24 / S1 "hold" is the absence of a write, so this
    // holds the last COMMANDED angle instead. The difference is deliberate:
    // re-issuing the measurement every cycle would make the controller chase
    // servo noise and would be a command nobody asked for.
    write_wheel_commands(time, zeros);
    if (write_steering) {
      write_steering_commands(steering_joint_order);
    }
    publish_intent_echo(time, consumed);
    publish_wheel_report(time);
    return controller_interface::return_type::OK;
  }

  // The LIMITED twist, not the requested one: limit_twist_to_steering_range may
  // have reduced |omega| to make the pose reachable, and a manoeuvre label must
  // describe what the machine is doing rather than what was asked for.
  last_manoeuvre_ = classify_manoeuvre(limited.twist);

  const auto & targets = *limited.targets;
  std::array<double, kNumWheels> steering_model{};
  std::array<double, kNumWheels> wheel_speeds_model{};
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    // The steering angles arrive reachable (limit_twist_to_steering_range has
    // guaranteed it), so nothing is clamped here — this only applies the slew
    // braking and the wheel speed saturation.
    //
    // THE BRAKING REFERENCE IS THE POST-ARBITRATION TARGET, i.e. the angle this
    // cycle actually commands, not the IK-derived one (user decision
    // 2026-08-19). This is a DELIBERATE DEPARTURE from strict functional
    // equivalence with swerve_cmd_node and therefore from NFR-10 acceptance 1;
    // it is recorded as such rather than left to look like a porting slip.
    //
    // Why: swerve_cmd_node cannot see arbitration point A2 at all — the
    // /teleop/direct_steer override lived in steer_servo_node, a different
    // process. Braking against the IK target was not a decision there, it was
    // the only thing reachable. In keyboard mode teleop_mux zeroes angular.z
    // and vy, so the IK target is straight ahead while the wheels are really at
    // the A/D angle: the error never shrinks, and cornering runs permanently at
    // the floor scale (0.45 at 35 deg of steering) as an artefact of the split
    // chain. With A2 inside this controller the commanded target is knowable,
    // so the brake now measures what it was always meant to measure: how far
    // the modules still are from where they are being told to go.
    //
    // Constants are UNCHANGED (min_scale 0.45, deadband 0.12 rad, reference
    // 1.0472 rad). This is a change of reference, not of tuning.
    //
    // No held target can reach this line. `write_steering == false` here means
    // no override is fresh, so the reference is the IK target being written on
    // this same cycle; the OP-24/S1 hold branches (stale twist, exact-zero
    // twist) and the kRejected branch all return above with the wheels already
    // zeroed, so a steering angle that is merely being HELD never scales a
    // drive command.
    const double scale_reference_angle =
      direct_fresh ? steering_joint_order[kModelToJointIndex[i]] : targets[i].angle;
    const double scale = steer_alignment_scale(scale_reference_angle, current_steering_model[i]);
    double angular_speed = (targets[i].speed / wheel_radius_) * scale;
    angular_speed =
      std::max(-max_wheel_angular_speed_, std::min(max_wheel_angular_speed_, angular_speed));
    steering_model[i] = targets[i].angle;
    wheel_speeds_model[i] = angular_speed;
  }

  if (enable_steer_feedback_differential_) {
    wheel_speeds_model = apply_steer_feedback_differential(
      limited.twist, current_steering_model, wheel_speeds_model, period.seconds());
  }

  if (!direct_fresh) {
    steering_joint_order = to_joint_order(steering_model);
    write_steering = true;
  }

  auto wheel_joint_order = to_joint_order(wheel_speeds_model);
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    wheel_joint_order[i] *= wheel_command_multipliers_[i];
  }

  write_steering_commands(steering_joint_order);
  // The POST-ARBITRATION target — the A2 direct_steer override when one is
  // fresh, the IK target otherwise — matching what steer_alignment_scale brakes
  // against. A direct_steer command that jumps the pose 90 deg while the robot
  // is driving is exactly as much of a transition as an IK one, and the gate
  // must see both or it guards the wrong half of the arbitration.
  write_wheel_commands(time, wheel_joint_order, &steering_joint_order, &current_steering_joint);
  publish_intent_echo(time, consumed);
  publish_wheel_report(time);

  return controller_interface::return_type::OK;
}

}  // namespace gripperx_swerve_controller

PLUGINLIB_EXPORT_CLASS(
  gripperx_swerve_controller::SwerveController, controller_interface::ControllerInterface)
