#ifndef BOT_HARDWARE_INTERFACE__BOT_INTERFACE_HPP_
#define BOT_HARDWARE_INTERFACE__BOT_INTERFACE_HPP_

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace gripperx_hardware_interface
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

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

  void joint_states_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void publish_zero_commands();
  void publish_stop_commands();
  void wheel_command_input_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void watchdog_check();
  void start_watchdog();
  void stop_watchdog();
  bool parse_joint_layout();

  std::string joint_commands_topic_{"/hw/joint_commands"};
  std::string joint_states_topic_{"/hw/joint_states"};
  double state_timeout_sec_{1.0};

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

  std::atomic<bool> commands_stale_{false};
};

}  // namespace gripperx_hardware_interface

#endif  // BOT_HARDWARE_INTERFACE__BOT_INTERFACE_HPP_
