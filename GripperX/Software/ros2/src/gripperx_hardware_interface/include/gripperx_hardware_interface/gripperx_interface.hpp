#ifndef BOT_HARDWARE_INTERFACE__BOT_INTERFACE_HPP_
#define BOT_HARDWARE_INTERFACE__BOT_INTERFACE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "gripperx_control_msgs/msg/swerve_intent_echo.hpp"
#include "gripperx_hardware_interface/command_watchdog.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

namespace gripperx_hardware_interface
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Provenance of the per-wheel velocity/position feedback (FR-11 items 5/6,
// deviation D14). Mirrors EncoderStatus in the ESP32 firmware's
// motor_controller.hpp for codes 0..3 — change both or neither.
//
// MONOTONE IN CONFIDENCE: `>= kLiveUnconfirmed` is the measurement test;
// below it the value is an echo of the command or unknown. Do not renumber.
//
// kUnknown is NEGATIVE on purpose: a too-short message decodes to it (the
// firmware never sends it), and it must sort below the weakest firmware code
// so it can never satisfy the measurement test.
enum WheelFeedbackProvenance : int
{
  kProvenanceUnknown = -1,
  kProvenanceNoEncoder = 0,
  kProvenanceInitFailed = 1,
  kProvenanceLiveUnconfirmed = 2,
  kProvenanceLive = 3
};

class GripperXInterface : public hardware_interface::SystemInterface
{
public:
  GripperXInterface();
  ~GripperXInterface() override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr size_t kNumSteerJoints = 4;
  static constexpr size_t kNumWheelJoints = 4;
  static constexpr size_t kNumJoints = kNumSteerJoints + kNumWheelJoints;

  // Default activation window for the SR-14 gate. 120.0 s is a USER-SET value,
  // PENDING COLD-BOOT VERIFICATION: it replaces a warm-bringup-derived default
  // because cold starts take longer, but the time a cold boot needs has not
  // been measured.
  //
  // Ceiling: --controller-manager-timeout in real_robot.launch.py's spawners
  // MUST stay above this value, or they die while the gate is still
  // legitimately waiting and never get to report anything. Keep the two in
  // step.
  static constexpr double kDefaultSteerStatesActivationTimeoutSec = 120.0;

  void joint_states_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void steer_states_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  bool wait_for_steer_states(std::vector<double> & measured);
  void update_steering_position_states();
  void set_steer_states_valid(bool valid);
  void set_wheel_feedback_provenance(const std::array<int, kNumWheelJoints> & provenance);
  void publish_zero_commands();
  void publish_stop_commands();
  void wheel_command_input_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void cmd_vel_input_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void intent_echo_callback(const gripperx_control_msgs::msg::SwerveIntentEcho::SharedPtr msg);
  void watchdog_check();
  void watchdog_check_wheel_commands(const std::chrono::steady_clock::time_point & now);
  void watchdog_check_twist_echo(const std::chrono::steady_clock::time_point & now);
  void start_watchdog();
  void stop_watchdog();
  bool parse_joint_layout();

  std::string joint_commands_topic_{"/hw/joint_commands"};
  std::string joint_states_topic_{"/hw/joint_states"};
  double state_timeout_sec_{1.0};

  // FR-10: the ESP32 has no steering sensor and never writes state indices
  // 0-3, so that state would be truthful in Gazebo but a constant 0.0 (false)
  // on the real robot. Measured instead by steer_servo_node, which reads the
  // Feetech servos on the Pi's own USB bus and publishes them here.
  std::string steer_states_topic_{"/hw/steer_states"};
  double steer_states_timeout_sec_{0.5};
  std::string steer_states_valid_topic_{"/hw/steer_states_valid"};

  // SR-14 item 4: how long on_activate() waits for the FIRST valid measurement before it
  // refuses to activate. Distinct from steer_states_timeout_sec_, which is the runtime
  // freshness window (FR-10) — one is a startup budget, the other a staleness rule.
  double steer_states_activation_timeout_sec_{kDefaultSteerStatesActivationTimeoutSec};

  // FR-11 items 5/6: whether the per-wheel velocity in /hw/joint_states[4-7]
  // is a measurement or the command echoed back. A latched per-wheel code,
  // not a state interface, because GazeboSimSystem exports no such interface
  // — a controller claiming it would fail to activate in sim and fork real
  // from sim (§3.1.6 / SR-14 item 4).
  std::string wheel_feedback_valid_topic_{"/hw/wheel_feedback_valid"};

  // Command watchdog behind the ros2_control layer. Detects a wedged
  // controller_manager that republishes a stale command while fresh inputs
  // still arrive, or a dead upstream command source. Runs in its own node +
  // executor + thread, independent of both the read/update/write cycle and
  // the controller_manager executor, which is the failure candidate.
  bool command_watchdog_enabled_{true};
  double command_timeout_sec_{0.5};
  double command_watchdog_rate_hz_{50.0};
  double command_divergence_eps_{1e-3};
  std::string wheel_command_topic_{"/wheel_velocity_controller/commands"};

  // Which reference input the watchdog polices against (OP-18a). Defaults to
  // the OLD one: /wheel_velocity_controller/commands only disappears when the
  // NFR-10 rebuild takes over the active path. Switching to W2 is then a
  // one-word change in gripperx_v1.ros2_control.xacro, not a code change.
  WatchdogReference watchdog_reference_{WatchdogReference::kWheelCommands};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string intent_echo_topic_{"/swerve_controller/intent_echo"};

  // TWIST DIVERGENCE TOLERANCE — a distinct quantity, `TO-VERIFY` (OP-18a item 5).
  // NOT command_divergence_eps (0.001, rad/s of WHEEL angular velocity):
  // these are m/s / rad/s of the BODY twist, and reusing it would compare
  // chassis yaw rate against wheel spin rate as the same number.
  //
  // Must stay well below the finest real /cmd_vel step any source here can
  // emit (~0.005 m/s / 0.012 rad/s, keyboard_teleop_node's manoeuvre slew),
  // and SMALL rather than half that step: too large silently disables
  // divergence detection, too small only sets an inert "changed" flag.
  //
  // NOT derived from the autonomous path: Nav2/DWB emits continuous twists
  // with no quantisation floor and has never been driven (same gap as
  // OP-24/S1 stage 2's exact-zero test).
  TwistTolerance twist_tolerance_{};

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_steering_commands_;
  std::vector<double> hw_wheel_commands_;

  std::unordered_map<std::string, size_t> steer_index_by_joint_;
  std::unordered_map<std::string, size_t> wheel_index_by_joint_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_commands_pub_;

  std::mutex joint_states_mutex_;
  std_msgs::msg::Float64MultiArray latest_joint_states_;
  std::chrono::steady_clock::time_point last_joint_states_time_;
  bool joint_states_received_{false};
  std::atomic<bool> active_{false};

  // FR-10 steering feedback. Serviced by the controller_manager executor, like
  // joint_states_sub_; read() only copies the cached vector under the mutex.
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr steer_states_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr steer_states_valid_pub_;

  std::mutex steer_states_mutex_;
  std::vector<double> latest_steer_states_;
  std::chrono::steady_clock::time_point last_steer_states_time_;
  bool steer_states_received_{false};

  // Health edge tracking; touched only from the update thread (read()).
  bool steer_states_valid_{false};
  bool steer_states_valid_published_{false};

  // FR-11 wheel-feedback provenance. Same ownership rule as above: evaluated and
  // published from read() only, so no lock is needed.
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr wheel_feedback_valid_pub_;
  std::array<int, kNumWheelJoints> wheel_feedback_provenance_{
    {kProvenanceUnknown, kProvenanceUnknown, kProvenanceUnknown, kProvenanceUnknown}};
  bool wheel_feedback_valid_published_{false};

  // Watchdog infrastructure (independent of the controller_manager executor).
  rclcpp::Node::SharedPtr watchdog_node_;
  rclcpp::executors::SingleThreadedExecutor::UniquePtr watchdog_executor_;
  std::thread watchdog_thread_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_command_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr watchdog_commands_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::mutex wheel_command_mutex_;
  std::vector<double> latest_wheel_command_;
  std::chrono::steady_clock::time_point last_wheel_command_time_;
  bool wheel_command_received_{false};

  // Timestamp tied to the current divergence episode. Plain time_point, not
  // optional: there is no sentinel for "unset" here, and this value must not
  // be read unless divergence_active_ is true — that flag, not this member,
  // is what tracks whether a divergence is currently open.
  bool divergence_active_{false};
  std::chrono::steady_clock::time_point divergence_since_;

  // W2 reference input (OP-18a). Both subscriptions live on watchdog_node_,
  // serviced by the watchdog's OWN executor and thread — the property the
  // whole design turns on. Both carry KeepLast(10) + RELIABLE explicitly.
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<gripperx_control_msgs::msg::SwerveIntentEcho>::SharedPtr intent_echo_sub_;
  // The two callbacks and the timer run on that one executor thread, so this
  // mutex guards against nothing today. Kept because the state is safety
  // state, and a future multi-threaded executor must not turn a correctness
  // question into an archaeology question.
  std::mutex twist_watchdog_mutex_;
  TwistEchoWatchdog twist_watchdog_;

  std::atomic<bool> commands_stale_{false};
};

}  // namespace gripperx_hardware_interface

#endif  // BOT_HARDWARE_INTERFACE__BOT_INTERFACE_HPP_
