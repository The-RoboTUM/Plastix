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

// Provenance of the per-wheel velocity/position feedback (FR-11 items 5/6, deviation
// D14). Mirrors EncoderStatus in the ESP32 firmware's motor_controller.hpp for codes
// 0..3 — change both or neither.
//
// MONOTONE IN CONFIDENCE: `>= kLiveUnconfirmed` is the measurement test, everything
// below it means the value is an echo of the command or simply unknown. Do not
// renumber.
//
// kUnknown is a Pi-side value the firmware never sends: it is what a message too short
// to carry the provenance block decodes to. It is NEGATIVE on purpose, so that a
// message which says nothing sorts below the weakest thing the firmware can claim and
// can never satisfy the measurement test.
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

  // Default activation window for the SR-14 gate.
  //
  // 120.0 s is a USER-SET value (2026-08-18), PENDING COLD-BOOT VERIFICATION. The user set
  // it on the grounds that cold starts have taken considerably longer than the warm figure
  // measured below; the measurement that a cold boot actually needs has not been taken yet.
  //
  // The measured derivation it replaces is kept, so the reasoning is not lost: on a clean
  // WARM bringup (2026-08-18) the first /hw/steer_states arrived 5.885 s after 'activate'
  // began, and a single steer_servo_node respawn cycle costs a further 11.6 s
  // (respawn_delay 5.0 + node start 2.2 + servo-bus scan 4.4) -> 17.5 s worst case, which
  // is where the previous default of 20.0 s came from. That figure is a lower bound on what
  // is needed, not an upper bound: it says nothing about a cold boot.
  //
  // Ceiling: the spawners' --controller-manager-timeout in real_robot.launch.py MUST stay
  // above this value, otherwise the spawners die while the gate is still legitimately
  // waiting and the gate never gets to report anything. It was raised 30 -> 150 s with this
  // change. Keep the two in step.
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

  // FR-10: the ESP32 has no steering sensor and never writes state indices 0-3, so the
  // steering position state interface would read a constant 0.0 on the real robot while
  // being truthful in Gazebo. The measurement comes from steer_servo_node, which reads the
  // Feetech servos on the Pi's own USB bus and publishes them here.
  std::string steer_states_topic_{"/hw/steer_states"};
  double steer_states_timeout_sec_{0.5};
  std::string steer_states_valid_topic_{"/hw/steer_states_valid"};

  // SR-14 item 4: how long on_activate() waits for the FIRST valid measurement before it
  // refuses to activate. Distinct from steer_states_timeout_sec_, which is the runtime
  // freshness window (FR-10) — one is a startup budget, the other a staleness rule.
  double steer_states_activation_timeout_sec_{kDefaultSteerStatesActivationTimeoutSec};

  // FR-11 items 5/6: whether the per-wheel velocity in /hw/joint_states[4-7] is a
  // measurement or the command echoed back. Published as a latched per-wheel code
  // rather than exported as a state interface ON PURPOSE: GazeboSimSystem does not
  // export such an interface, so a controller claiming it would fail to activate in
  // sim and fork real from sim (§3.1.6 / SR-14 item 4).
  std::string wheel_feedback_valid_topic_{"/hw/wheel_feedback_valid"};

  // Command watchdog behind the ros2_control layer. Detects a wedged
  // controller_manager that keeps republishing a stale (frozen) command while
  // fresh controller inputs are still arriving, and a dead upstream command
  // source. Runs in its own node + executor + thread, independent of both the
  // read/update/write cycle AND the controller_manager executor (which is the
  // failure candidate, see incident 2026-07-06 #5 / Task #14).
  bool command_watchdog_enabled_{true};
  double command_timeout_sec_{0.5};
  double command_watchdog_rate_hz_{50.0};
  double command_divergence_eps_{1e-3};
  std::string wheel_command_topic_{"/wheel_velocity_controller/commands"};

  // Which reference input the watchdog polices against (OP-18a). DEFAULT IS THE
  // OLD ONE ON PURPOSE: /wheel_velocity_controller/commands only disappears when
  // the NFR-10 rebuild takes over the active path, and until then the running
  // robot must keep the watchdog it has. Switching to W2 is then a one-word
  // change in gripperx_v1.ros2_control.xacro, not a code change, and both
  // reference inputs can exist side by side while the switch-over is staged.
  WatchdogReference watchdog_reference_{WatchdogReference::kWheelCommands};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string intent_echo_topic_{"/swerve_controller/intent_echo"};

  // TWIST DIVERGENCE TOLERANCE — A NEW QUANTITY, AND `TO-VERIFY` (OP-18a item 5).
  //
  // It is NOT command_divergence_eps (0.001). That value is rad/s of WHEEL
  // angular velocity; these are m/s and rad/s of the BODY twist. Reusing it
  // would compare chassis yaw rate against wheel spin rate as if they were the
  // same number.
  //
  // Derived from what the tree can actually command. The finest non-zero
  // /cmd_vel component any source in this repository emits is a manoeuvre-slew
  // twist from keyboard_teleop_node: crab_speed_m_s 0.25 x manoeuvre_pose_scale
  // 0.02 = 0.005 m/s, and spin_speed_rad_s 0.60 x 0.02 = 0.012 rad/s. Nothing
  // between there and zero exists, and teleop_mux copies the values through
  // unmodified, so a genuine command change is never smaller than that.
  //
  // 1.0e-4 sits 50x / 120x below the finest real step, with room for a future
  // finer source, and ~1e12 above the float64 representation floor at these
  // magnitudes — so no rounding can fabricate a change.
  //
  // Chosen SMALL rather than "half the finest step" because the two errors are
  // not symmetric: too large silently deletes divergence detection (the check
  // the 2026-07-06 incident exists for), while too small can only set a
  // "changed" flag that is inert unless the echo sequence is ALREADY frozen —
  // i.e. unless the fault is genuinely present.
  //
  // NOT derived from the autonomous path: Nav2/DWB emits continuous twists with
  // no quantisation floor and has never been driven, so no measurement exists.
  // That is the same gap OP-24/S1 stage 2 records for its exact-zero test, and
  // it gets measured at the same time.
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

  // Timestamp when output/input divergence started; nullopt/unset while aligned.
  bool divergence_active_{false};
  std::chrono::steady_clock::time_point divergence_since_;

  // W2 reference input (OP-18a). Both subscriptions live on watchdog_node_ and
  // are therefore serviced by the watchdog's OWN executor and thread — the
  // property the whole design turns on, since the controller_manager executor
  // is the failure candidate. Both carry KeepLast(10) + RELIABLE explicitly.
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<gripperx_control_msgs::msg::SwerveIntentEcho>::SharedPtr intent_echo_sub_;
  // The two callbacks and the timer all run on that single executor thread, so
  // this mutex guards against nothing today. It is kept because the state is
  // safety state and a future multi-threaded executor here must not turn a
  // correctness question into an archaeology question.
  std::mutex twist_watchdog_mutex_;
  TwistEchoWatchdog twist_watchdog_;

  std::atomic<bool> commands_stale_{false};
};

}  // namespace gripperx_hardware_interface

#endif  // BOT_HARDWARE_INTERFACE__BOT_INTERFACE_HPP_
