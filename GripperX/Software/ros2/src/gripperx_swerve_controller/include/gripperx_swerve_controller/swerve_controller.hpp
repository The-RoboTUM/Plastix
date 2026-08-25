// swerve_controller — the 4WIS4WID kinematics as a ros2_control controller.
//
// NFR-10 / §3.1 variant B. Subscribes /cmd_vel directly (analogous to
// diff_drive_controller) and writes the 8 steering/wheel COMMAND INTERFACES,
// replacing swerve_cmd_node + joint_command_bridge in the active path.
//
// NAMING, because there is a deliberate collision to resolve.
// gripperx_control/src/gripperx_control/swerve_controller.py already exists: it
// is the old PYTHON kinematic TRACKING-controller library (the
// _compute_tracking_control path), which is not in the active configuration and
// is not ported here. To keep the two unmistakable:
//   * package  : gripperx_swerve_controller   (its own package — a ros2_control
//                controller plugin cannot live in the ament_python
//                gripperx_control package anyway)
//   * class    : gripperx_swerve_controller::SwerveController
//   * plugin   : gripperx_swerve_controller/SwerveController
//   * instance : "swerve_controller" in ros2_controllers.yaml — the name §3.1.2
//                and NFR-10 use, and the one that must appear in the graph.
// Nothing imports the Python file from here; the two never meet in one process.
// When the deletion round comes, the Python file should be renamed to
// swerve_tracking_controller.py rather than merely deleted, so the tracking
// path stays findable in git history under a name that says what it is.
//
// WHAT THIS CONTROLLER IS NOT, BY DEFAULT: it is OPEN-LOOP on wheel velocity.
// It reads the four wheel velocity STATE interfaces and publishes them next to
// the commanded value (FR-11 item 1), and no error term, integrator or gain acts
// on them (FR-11 item 2, NFR-10 acceptance 10). A rebuild that quietly gained a
// regulator has FAILED NFR-10, because acceptance 1 (functional equivalence to
// today's chain) becomes unfalsifiable the moment a regulator can mask the
// rebuild's own faults.
//
// A PER-WHEEL VELOCITY REGULATOR NOW EXISTS AS STRUCTURE (WheelRegulator,
// wheel_regulator.hpp) AND IS DISABLED BY DEFAULT — `wheel_regulator_enabled`
// is false in the struct default AND in ros2_controllers.yaml. While it is
// false, update() never calls it: the command written to the interface is the
// bit-identical feedforward value, so the sentence above stays literally true
// and NFR-10 acceptance 10 stays satisfied. Enabling it VIOLATES FR-11 item 2 as
// that item is written today, so it is a requirements decision and not a tuning
// step. Full rationale, including why the structure may exist now (FR-11 item 3),
// at the top of wheel_regulator.hpp.
//
// HWR-30a LIVES HERE TOO, AND IT IS STILL NOT A REGULATOR. The encoder-based
// stall detection and its tier-1 response (StallDetector, stall_detector.hpp)
// sit LAST in the chain, between everything else and the command interfaces. The
// only thing they can do to a wheel command is replace it with exactly 0.0; they
// never scale it, never bias it and never read the measured velocity. The
// ordering — regulator first, stall gate last — is enforced in one place and is
// documented at write_wheel_commands(). Placement rationale, in
// full, at the top of stall_detector.hpp: the hardware interface is replaced by
// gz_ros2_control/GazeboSimSystem in the twin, so a detector placed there would
// be structurally untestable in simulation.

#ifndef GRIPPERX_SWERVE_CONTROLLER__SWERVE_CONTROLLER_HPP_
#define GRIPPERX_SWERVE_CONTROLLER__SWERVE_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "gripperx_control_msgs/msg/swerve_intent_echo.hpp"
#include "gripperx_control_msgs/msg/wheel_stall_state.hpp"
#include "gripperx_control_msgs/msg/wheel_velocity_report.hpp"
#include "gripperx_swerve_controller/alignment_gate.hpp"
#include "gripperx_swerve_controller/stall_detector.hpp"
#include "gripperx_swerve_controller/steering_limits.hpp"
#include "gripperx_swerve_controller/swerve_kinematics.hpp"
#include "gripperx_swerve_controller/wheel_regulator.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace gripperx_swerve_controller
{

class SwerveController : public controller_interface::ControllerInterface
{
public:
  SwerveController() = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// /cmd_vel as received, with the time it arrived and its arrival index.
  struct StampedTwist
  {
    geometry_msgs::msg::Twist twist;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    uint64_t sequence{0};
    bool valid{false};
  };

  /// /teleop/direct_steer as received (joint order FL, FR, BL, BR).
  struct StampedDirectSteer
  {
    std::array<double, kNumWheels> angles{};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void direct_steer_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void active_mode_callback(const std_msgs::msg::String::SharedPtr msg);
  void wheel_feedback_valid_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

  /// RUNTIME SWITCH FOR THE VELOCITY REGULATOR — user decision 2026-08-20.
  /// `ros2 param set /swerve_controller wheel_regulator_enabled false` must take
  /// effect on the NEXT update() cycle, on a live driving robot, with no reload:
  /// "reload the controller" is far too slow to be an off-switch. Validation
  /// happens in the on-set callback (which may still refuse), the accepted value
  /// lands in an atomic in the post-set callback, and update() picks it up.
  /// Everything that actually changes state — reset, log — happens in the update
  /// loop, so the executor thread never touches the regulator.
  rcl_interfaces::msg::SetParametersResult validate_parameter_update(
    const std::vector<rclcpp::Parameter> & parameters);
  void apply_parameter_update(const std::vector<rclcpp::Parameter> & parameters);
  static const char * provenance_label(int provenance);

  bool resolve_interface_indices();
  /// The wheel POSITION state interfaces are claimed only when something needs
  /// them, so a hardware component that does not export them can still run this
  /// controller. TWO consumers now: HWR-30a keys its detection off the
  /// accumulated position, and the regulator uses it as its new-sample novelty
  /// signal (a quantised velocity legitimately repeats at a steady speed; an
  /// accumulator does not).
  bool needs_wheel_position_state() const
  {
    return stall_config_.enabled || regulator_config_.enabled;
  }
  bool read_steering_feedback(std::array<double, kNumWheels> & joint_order_angles);

  /// The ONLY place a wheel velocity command reaches a command interface — every
  /// branch of update() and on_deactivate() funnel through it, which is what
  /// makes the tier-1 gate unbypassable. `joint_order_rad_s` is the REQUESTED
  /// (feedforward) command. TWO stages act on it, in this order and only this
  /// order: the velocity regulator may add a bounded correction (disabled by
  /// default, so normally a no-op), and then HWR-30a's stall gate forces every
  /// latched wheel to exactly zero. The gate is LAST, so no correction can ever
  /// stand between a latched wheel and its zero.
  /// `steer_target` / `steer_measured` are the POST-ARBITRATION commanded
  /// steering angles and the measured ones, joint order, for the alignment gate.
  /// BOTH NULL means "no steering command is being written on this cycle", which
  /// under OP-24/S1 is what a hold IS — the gate then has no new target to align
  /// to and keeps its state. Passed explicitly rather than stashed in a member
  /// so that every one of the five call sites has to say which case it is in.
  void write_wheel_commands(
    const rclcpp::Time & time, const std::array<double, kNumWheels> & joint_order_rad_s,
    const std::array<double, kNumWheels> * steer_target = nullptr,
    const std::array<double, kNumWheels> * steer_measured = nullptr);
  void log_stall_events(
    const StallDetectorResult & result, const std::array<double, kNumWheels> & requested,
    const rclcpp::Time & time);
  void publish_stall_state(const rclcpp::Time & time);
  void write_steering_commands(const std::array<double, kNumWheels> & joint_order_rad);
  void publish_intent_echo(const rclcpp::Time & time, const StampedTwist & consumed);
  void publish_wheel_report(const rclcpp::Time & time);
  void report_limit_status(const LimitedTwist & limited);

  /// Port of swerve_cmd_node._steer_alignment_scale: reduce wheel drive while
  /// the steering modules are still rotating. `reference_rad` is only the
  /// NORMALISATION of "how big is a big steering error" — it is NOT a limit.
  ///
  /// The formula and its three constants are unchanged. What changed (user
  /// decision 2026-08-19) is what `target_angle` is: the caller passes the
  /// POST-ARBITRATION commanded angle — the A2 direct_steer override when one
  /// is fresh, the IK target otherwise — instead of always the IK target. See
  /// the block comment at the call site in update().
  double steer_alignment_scale(double target_angle, double current_angle) const;

  /// Port of swerve_cmd_node._apply_steer_feedback_differential (task #21).
  std::array<double, kNumWheels> apply_steer_feedback_differential(
    const BodyTwist & desired_body_twist,
    const std::array<double, kNumWheels> & current_steering_angles_model,
    const std::array<double, kNumWheels> & wheel_angular_speeds_model, double dt);

  // --- parameters (mirrored from swerve_cmd.yaml unless stated) --------------
  double a_{0.1809};
  double b_{0.1087};
  double wheel_radius_{0.070};
  // King-pin -> tyre-contact lateral offset per wheel, JOINT order FL, FR, BL,
  // BR, signed in base_link y. ZERO BY DEFAULT ON PURPOSE: a config that does
  // not carry the key must behave exactly as the controller did before the
  // contact-point correction existed.
  std::array<double, kNumWheels> wheel_lateral_offset_{0.0, 0.0, 0.0, 0.0};
  std::vector<std::string> steering_joint_names_;
  std::vector<std::string> wheel_joint_names_;
  std::array<double, kNumWheels> wheel_command_multipliers_{1.0, 1.0, 1.0, 1.0};
  double steer_alignment_min_scale_{0.45};
  double steer_alignment_deadband_rad_{0.12};
  double steer_alignment_reference_rad_{1.0472};
  double max_wheel_angular_speed_{12.0};
  double cmd_vel_timeout_sec_{0.5};
  bool enforce_front_forward_{false};
  bool allow_reverse_{true};
  bool enable_steer_feedback_differential_{false};
  double steer_diff_omega_gate_{0.05};
  double steer_diff_min_speed_mps_{0.03};
  double steer_diff_time_constant_sec_{0.3};
  double steer_diff_max_omega_{1.5};
  double steer_diff_min_ratio_{0.5};
  double steer_diff_max_ratio_{1.5};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string direct_steer_topic_{"/teleop/direct_steer"};
  double direct_timeout_sec_{0.5};
  std::string active_mode_topic_{"/teleop/active_mode"};
  std::string autonomous_mode_name_{"autonomous"};
  std::string intent_echo_topic_{"/swerve_controller/intent_echo"};
  std::string wheel_report_topic_{"/swerve_controller/wheel_velocities"};

  // --- PER-WHEEL VELOCITY REGULATOR (structure only, DISABLED BY DEFAULT) ---
  // Defaults live in WheelRegulatorConfig; every gain there is TO-VERIFY and
  // says so at its declaration and in ros2_controllers.yaml. `enabled` is FALSE
  // in both places: merging and deploying this changes nothing about how the
  // robot drives until someone deliberately turns it on.
  //
  // `enabled` in this struct is the CONFIGURED value and is never written at
  // runtime: it decides which RESOURCES are claimed (the wheel POSITION state
  // interfaces and the provenance subscription), and resources cannot be claimed
  // after activation. The runtime switch is regulator_enabled_rt_ below.
  WheelRegulatorConfig regulator_config_{};
  /// The live enable state, written by the parameter callback (executor thread),
  /// read by update() (control loop). Initialised from the configured value.
  std::atomic<bool> regulator_enabled_rt_{false};
  /// The control loop's own mirror of the above, used to detect the transition.
  /// Only the control loop touches it.
  bool regulator_active_{false};
  /// Whether the resources the regulator needs were claimed at configure time
  /// (i.e. stall detection or the regulator was enabled in the config). When
  /// false, the runtime switch REFUSES to turn the regulator on and says why —
  /// silently accepting a switch that cannot work is the worse answer.
  bool regulator_resources_claimed_{false};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_parameters_handle_;

  // --- HWR-30a: encoder-based stall detection + tier-1 response -------------
  // Defaults live in StallDetectorConfig; every threshold among them is
  // TO-VERIFY and says so at its declaration there and in ros2_controllers.yaml.
  StallDetectorConfig stall_config_{};

  // --- STEERING ALIGNMENT GATE (structure + wiring, DISABLED BY DEFAULT) -----
  // Withholds drive while the modules slew into a new pose — the controller-side
  // replacement for gripperx_teleop/manoeuvre.py's TransitionGuard, covering
  // every twist source instead of only the keyboard. Rationale, and why it ships
  // off, at the top of alignment_gate.hpp.
  AlignmentGateConfig alignment_config_{};
  AlignmentGate alignment_gate_;
  /// Last published gate state, and the classification of the twist that
  /// produced it. Both are DERIVED and neither is read back by the control path.
  int last_alignment_status_{kAlignDisabled};
  double last_alignment_error_rad_{0.0};
  int last_manoeuvre_{0};
  /// Rate limit on the gate's log line. A LOGGING cadence, not a control
  /// threshold: the transitions are edge events and this only bounds an
  /// oscillation across the hysteresis band.
  double alignment_log_period_sec_{5.0};
  double alignment_last_log_sec_{0.0};
  bool alignment_ever_logged_{false};
  /// Per-wheel rate limit on the authority-limit log line. A LOGGING cadence,
  /// not a control threshold and not TO-VERIFY: the transitions are edge events
  /// and this only bounds an oscillation across the bound. Per wheel for the
  /// same reason stall_log_period_sec_ is — RCLCPP_*_THROTTLE keeps its state per
  /// CALL SITE, so one wheel would suppress another's line.
  double regulator_log_period_sec_{5.0};

  std::string wheel_feedback_valid_topic_{"/hw/wheel_feedback_valid"};
  std::string stall_state_topic_{"/swerve_controller/stall_state"};
  /// Per-wheel rate limit on the trip ERROR line. A LOGGING cadence, not a
  /// safety threshold, and not TO-VERIFY: the trips themselves are edge events,
  /// this only bounds a pathological trip/release oscillation. Per wheel on
  /// purpose — RCLCPP_*_THROTTLE keeps its state per CALL SITE, so it would let
  /// one wheel's trip suppress another wheel's.
  double stall_log_period_sec_{5.0};

  // --- derived --------------------------------------------------------------
  std::unique_ptr<SwerveKinematics> model_;
  SteeringLimits joint_steering_limits_;
  SteeringLimits model_steering_limits_;

  // --- subscriptions / realtime plumbing ------------------------------------
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr direct_steer_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_mode_sub_;
  realtime_tools::RealtimeBuffer<StampedTwist> cmd_vel_buffer_;
  realtime_tools::RealtimeBuffer<StampedDirectSteer> direct_steer_buffer_;
  std::atomic<bool> mode_is_autonomous_{false};
  std::atomic<uint64_t> cmd_vel_rx_count_{0};

  std::unique_ptr<realtime_tools::RealtimePublisher<gripperx_control_msgs::msg::SwerveIntentEcho>>
    intent_echo_pub_;
  std::unique_ptr<
    realtime_tools::RealtimePublisher<gripperx_control_msgs::msg::WheelVelocityReport>>
    wheel_report_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr wheel_feedback_valid_sub_;
  realtime_tools::RealtimeBuffer<std::array<int, kNumWheels>> wheel_provenance_buffer_;
  /// NOT a RealtimePublisher: it publishes edge-triggered (on activation and on
  /// every latch change), TRANSIENT_LOCAL, at most a handful of messages per
  /// drive. A dropped trylock on a latched topic would lose the edge and leave
  /// a late subscriber reading a stale latch, which is the one thing a latched
  /// topic must not do.
  rclcpp::Publisher<gripperx_control_msgs::msg::WheelStallState>::SharedPtr stall_state_pub_;

  // --- interface index maps (joint order FL, FR, BL, BR) --------------------
  std::array<std::size_t, kNumWheels> steer_cmd_index_{};
  std::array<std::size_t, kNumWheels> wheel_cmd_index_{};
  std::array<std::size_t, kNumWheels> steer_state_index_{};
  std::array<std::size_t, kNumWheels> wheel_state_index_{};
  /// Wheel POSITION state — claimed only while stall detection is enabled
  /// (HWR-30a), so a hardware component that does not export it can still run
  /// this controller by setting stall_detection_enabled: false.
  std::array<std::size_t, kNumWheels> wheel_position_state_index_{};
  bool interfaces_resolved_{false};

  // --- update-loop state ----------------------------------------------------
  double steer_diff_omega_filtered_{0.0};
  std::array<double, kNumWheels> last_wheel_command_{};
  /// The correction actually contained in last_wheel_command_, and whether the
  /// wheel was regulated when it was produced. Both are published (FR-11: the
  /// per-wheel control EFFORT must be observable, not only its result) and both
  /// are all-zero / all-false whenever the regulator is disabled.
  std::array<double, kNumWheels> last_wheel_correction_{};
  std::array<bool, kNumWheels> last_wheel_regulating_{};
  /// Per-wheel WHY (WheelRegulatorStatus). FR-14 item 7: a regulator that
  /// quietly stops regulating one wheel looks exactly like one that is working.
  std::array<int, kNumWheels> last_wheel_regulator_status_{
    kRegulatorDisabled, kRegulatorDisabled, kRegulatorDisabled, kRegulatorDisabled};
  /// Edge state for the authority-limit log line.
  std::array<bool, kNumWheels> regulator_at_limit_{};
  std::array<double, kNumWheels> regulator_last_limit_log_sec_{};
  std::array<bool, kNumWheels> regulator_ever_limit_logged_{};
  WheelRegulator wheel_regulator_;
  StallDetector stall_detector_;
  std::array<double, kNumWheels> stall_last_log_sec_{};
  std::array<bool, kNumWheels> stall_ever_logged_{};
  bool stall_multi_wheel_condition_{false};
};

}  // namespace gripperx_swerve_controller

#endif  // GRIPPERX_SWERVE_CONTROLLER__SWERVE_CONTROLLER_HPP_
