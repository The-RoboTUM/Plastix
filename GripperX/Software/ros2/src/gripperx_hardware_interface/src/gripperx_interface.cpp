#include "gripperx_hardware_interface/gripperx_interface.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/clock.hpp"

namespace gripperx_hardware_interface
{
namespace
{
constexpr size_t kNumSteerJoints = 4;
constexpr size_t kNumWheelJoints = 4;
constexpr size_t kNumJoints = kNumSteerJoints + kNumWheelJoints;

constexpr std::array<const char *, kNumSteerJoints> kSteerJointOrder = {
  "f_left_steer", "f_right_steer", "b_leftsteer", "b_rightsteer"};
constexpr std::array<const char *, kNumWheelJoints> kWheelJointOrder = {
  "f_leftwheel", "f_rightwheel", "b_leftwheel", "b_rightwheel"};

bool parse_bool_param(const std::string & value)
{
  return !(value == "false" || value == "False" || value == "0");
}
}  // namespace

GripperXInterface::GripperXInterface() = default;

GripperXInterface::~GripperXInterface()
{
  active_.store(false);
  stop_watchdog();
}

CallbackReturn GripperXInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const auto commands_topic_it = info_.hardware_parameters.find("joint_commands_topic");
  if (commands_topic_it != info_.hardware_parameters.end()) {
    joint_commands_topic_ = commands_topic_it->second;
  }

  const auto states_topic_it = info_.hardware_parameters.find("joint_states_topic");
  if (states_topic_it != info_.hardware_parameters.end()) {
    joint_states_topic_ = states_topic_it->second;
  }

  const auto timeout_it = info_.hardware_parameters.find("state_timeout_sec");
  if (timeout_it != info_.hardware_parameters.end()) {
    state_timeout_sec_ = std::stod(timeout_it->second);
  }

  const auto wd_enabled_it = info_.hardware_parameters.find("command_watchdog_enabled");
  if (wd_enabled_it != info_.hardware_parameters.end()) {
    command_watchdog_enabled_ = parse_bool_param(wd_enabled_it->second);
  }

  const auto wd_timeout_it = info_.hardware_parameters.find("command_timeout_sec");
  if (wd_timeout_it != info_.hardware_parameters.end()) {
    command_timeout_sec_ = std::stod(wd_timeout_it->second);
  }

  const auto wd_rate_it = info_.hardware_parameters.find("command_watchdog_rate_hz");
  if (wd_rate_it != info_.hardware_parameters.end()) {
    command_watchdog_rate_hz_ = std::stod(wd_rate_it->second);
  }

  const auto wd_eps_it = info_.hardware_parameters.find("command_divergence_eps");
  if (wd_eps_it != info_.hardware_parameters.end()) {
    command_divergence_eps_ = std::stod(wd_eps_it->second);
  }

  const auto wheel_cmd_topic_it = info_.hardware_parameters.find("wheel_command_topic");
  if (wheel_cmd_topic_it != info_.hardware_parameters.end()) {
    wheel_command_topic_ = wheel_cmd_topic_it->second;
  }

  if (command_watchdog_rate_hz_ <= 0.0) {
    RCLCPP_WARN(
      rclcpp::get_logger("GripperXInterface"),
      "command_watchdog_rate_hz must be > 0; falling back to 50 Hz.");
    command_watchdog_rate_hz_ = 50.0;
  }

  if (!parse_joint_layout()) {
    return CallbackReturn::ERROR;
  }

  hw_positions_.assign(info_.joints.size(), 0.0);
  hw_velocities_.assign(info_.joints.size(), 0.0);
  hw_steering_commands_.assign(kNumSteerJoints, 0.0);
  hw_wheel_commands_.assign(kNumWheelJoints, 0.0);

  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperXInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("GripperXInterface"), "Loaned node unavailable during activation.");
    return CallbackReturn::ERROR;
  }

  joint_states_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    joint_states_topic_, rclcpp::SensorDataQoS(),
    std::bind(&GripperXInterface::joint_states_callback, this, std::placeholders::_1));

  joint_commands_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    joint_commands_topic_, rclcpp::SystemDefaultsQoS());

  active_.store(true);
  commands_stale_.store(false);
  publish_zero_commands();

  start_watchdog();

  const auto activation_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!joint_states_received_ && std::chrono::steady_clock::now() < activation_deadline) {
    rclcpp::sleep_for(std::chrono::milliseconds(50));
  }

  if (!joint_states_received_) {
    RCLCPP_WARN(
      rclcpp::get_logger("GripperXInterface"),
      "No %s received within 5 s; continuing activation anyway.",
      joint_states_topic_.c_str());
  }

  RCLCPP_INFO(
    rclcpp::get_logger("GripperXInterface"),
    "Activated. commands=%s states=%s",
    joint_commands_topic_.c_str(), joint_states_topic_.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperXInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  active_.store(false);

  stop_watchdog();

  publish_zero_commands();

  joint_states_sub_.reset();
  joint_commands_pub_.reset();
  joint_states_received_ = false;

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GripperXInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() * 2);

  for (size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index) {
    const auto & joint = info_.joints[joint_index];
    state_interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_POSITION, &hw_positions_[joint_index]);
    state_interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[joint_index]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GripperXInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(kNumJoints);

  for (size_t index = 0; index < kNumSteerJoints; ++index) {
    command_interfaces.emplace_back(
      kSteerJointOrder[index], hardware_interface::HW_IF_POSITION,
      &hw_steering_commands_[index]);
  }

  for (size_t index = 0; index < kNumWheelJoints; ++index) {
    command_interfaces.emplace_back(
      kWheelJointOrder[index], hardware_interface::HW_IF_VELOCITY, &hw_wheel_commands_[index]);
  }

  return command_interfaces;
}

hardware_interface::return_type GripperXInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std_msgs::msg::Float64MultiArray latest_states;
  std::chrono::steady_clock::time_point latest_time;
  bool received = false;

  {
    std::lock_guard<std::mutex> lock(joint_states_mutex_);
    if (joint_states_received_) {
      latest_states = latest_joint_states_;
      latest_time = last_joint_states_time_;
      received = true;
    }
  }

  if (!received) {
    return hardware_interface::return_type::OK;
  }

  if (state_timeout_sec_ > 0.0) {
    const auto age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - latest_time).count();
    if (age > state_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("GripperXInterface"), *get_node()->get_clock(), 2000,
        "%s stale (%.3f s old).", joint_states_topic_.c_str(), age);
      return hardware_interface::return_type::ERROR;
    }
  }

  if (latest_states.data.size() < kNumJoints) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("GripperXInterface"), *get_node()->get_clock(), 2000,
      "%s expected %zu values, got %zu.",
      joint_states_topic_.c_str(), kNumJoints, latest_states.data.size());
    return hardware_interface::return_type::ERROR;
  }

  for (size_t index = 0; index < kNumSteerJoints; ++index) {
    const auto joint_it = steer_index_by_joint_.find(kSteerJointOrder[index]);
    if (joint_it == steer_index_by_joint_.end()) {
      continue;
    }
    const size_t joint_index = joint_it->second;
    hw_positions_[joint_index] = latest_states.data[index];
    hw_velocities_[joint_index] = 0.0;
  }

  for (size_t index = 0; index < kNumWheelJoints; ++index) {
    const auto joint_it = wheel_index_by_joint_.find(kWheelJointOrder[index]);
    if (joint_it == wheel_index_by_joint_.end()) {
      continue;
    }
    const size_t joint_index = joint_it->second;
    hw_velocities_[joint_index] = latest_states.data[kNumSteerJoints + index];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GripperXInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!active_.load() || !joint_commands_pub_) {
    return hardware_interface::return_type::OK;
  }

  // Fail-safe: if the watchdog flagged the command chain as stale, never forward
  // the (frozen) controller command; publish a stop (wheels zero, steering held).
  if (command_watchdog_enabled_ && commands_stale_.load()) {
    publish_stop_commands();
    return hardware_interface::return_type::OK;
  }

  std_msgs::msg::Float64MultiArray command_msg;
  command_msg.data.reserve(kNumJoints);

  for (size_t index = 0; index < kNumSteerJoints; ++index) {
    command_msg.data.push_back(hw_steering_commands_[index]);
  }
  for (size_t index = 0; index < kNumWheelJoints; ++index) {
    command_msg.data.push_back(hw_wheel_commands_[index]);
  }

  joint_commands_pub_->publish(command_msg);
  return hardware_interface::return_type::OK;
}

void GripperXInterface::joint_states_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> lock(joint_states_mutex_);
  latest_joint_states_ = *msg;
  last_joint_states_time_ = std::chrono::steady_clock::now();
  joint_states_received_ = true;
}

void GripperXInterface::publish_zero_commands()
{
  if (!joint_commands_pub_) {
    return;
  }

  std_msgs::msg::Float64MultiArray command_msg;
  command_msg.data.assign(kNumJoints, 0.0);
  joint_commands_pub_->publish(command_msg);
}

void GripperXInterface::publish_stop_commands()
{
  // Stop = drive wheels to zero, hold the last commanded steering angle so the
  // servos do not perform a surprise re-centering move while the robot halts.
  std_msgs::msg::Float64MultiArray command_msg;
  command_msg.data.reserve(kNumJoints);
  for (size_t index = 0; index < kNumSteerJoints; ++index) {
    command_msg.data.push_back(hw_steering_commands_[index]);
  }
  for (size_t index = 0; index < kNumWheelJoints; ++index) {
    command_msg.data.push_back(0.0);
  }

  // Publish on both paths so the stop goes out even if either the
  // controller_manager node or the RT write() cycle is degraded.
  if (joint_commands_pub_) {
    joint_commands_pub_->publish(command_msg);
  }
  if (watchdog_commands_pub_) {
    watchdog_commands_pub_->publish(command_msg);
  }
}

void GripperXInterface::start_watchdog()
{
  if (!command_watchdog_enabled_) {
    RCLCPP_WARN(
      rclcpp::get_logger("GripperXInterface"),
      "Command watchdog DISABLED via parameter; no stale-command protection active.");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(wheel_command_mutex_);
    wheel_command_received_ = false;
    latest_wheel_command_.clear();
  }
  divergence_active_ = false;
  commands_stale_.store(false);

  // Dedicated node/executor/thread, independent of the controller_manager
  // executor (the failure candidate). A subscription serviced by our own
  // executor keeps receiving fresh controller inputs even if the CM executor
  // is wedged; our own publisher can emit a stop even if the CM node is stuck.
  watchdog_node_ = std::make_shared<rclcpp::Node>("gripperx_interface_watchdog");

  // joint_command_bridge publishes the wheel commands RELIABLE + KeepLast(10).
  // SystemDefaultsQoS() leaves reliability as SYSTEM_DEFAULT, which resolves to the
  // DDS DataReader default (BEST_EFFORT) under rmw_fastrtps. On this stack a
  // BEST_EFFORT reader effectively did NOT receive the RELIABLE stream (verified:
  // the watchdog saw a single sample over 14 min while a RELIABLE reader saw 30 Hz),
  // so the deadman stayed latched in silence-STOP and zeroed /hw/joint_commands,
  // blocking all driving. Match the publisher explicitly so the safety deadman
  // actually observes the live command stream (RELIABLE also avoids best-effort
  // drops looking like silence / falsely tripping the divergence check).
  rclcpp::QoS wheel_command_qos(rclcpp::KeepLast(10));
  wheel_command_qos.reliable();
  wheel_command_sub_ = watchdog_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    wheel_command_topic_, wheel_command_qos,
    std::bind(&GripperXInterface::wheel_command_input_callback, this, std::placeholders::_1));

  watchdog_commands_pub_ = watchdog_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    joint_commands_topic_, rclcpp::SystemDefaultsQoS());

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / command_watchdog_rate_hz_));
  watchdog_timer_ = watchdog_node_->create_wall_timer(
    period, std::bind(&GripperXInterface::watchdog_check, this));

  watchdog_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  watchdog_executor_->add_node(watchdog_node_);
  watchdog_thread_ = std::thread([this]() { watchdog_executor_->spin(); });

  RCLCPP_INFO(
    rclcpp::get_logger("GripperXInterface"),
    "Command watchdog active. input=%s timeout=%.3fs rate=%.1fHz eps=%.4f",
    wheel_command_topic_.c_str(), command_timeout_sec_, command_watchdog_rate_hz_,
    command_divergence_eps_);
}

void GripperXInterface::stop_watchdog()
{
  if (watchdog_executor_) {
    watchdog_executor_->cancel();
  }
  if (watchdog_thread_.joinable()) {
    watchdog_thread_.join();
  }

  watchdog_timer_.reset();
  wheel_command_sub_.reset();
  watchdog_commands_pub_.reset();
  if (watchdog_executor_ && watchdog_node_) {
    watchdog_executor_->remove_node(watchdog_node_);
  }
  watchdog_executor_.reset();
  watchdog_node_.reset();
  commands_stale_.store(false);
}

void GripperXInterface::wheel_command_input_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(wheel_command_mutex_);
  latest_wheel_command_ = msg->data;
  last_wheel_command_time_ = std::chrono::steady_clock::now();
  wheel_command_received_ = true;
}

void GripperXInterface::watchdog_check()
{
  if (!command_watchdog_enabled_ || !active_.load() || command_timeout_sec_ <= 0.0) {
    commands_stale_.store(false);
    return;
  }

  const auto now = std::chrono::steady_clock::now();

  std::vector<double> latest_input;
  bool received = false;
  double input_age = 0.0;
  {
    std::lock_guard<std::mutex> lock(wheel_command_mutex_);
    received = wheel_command_received_;
    if (received) {
      latest_input = latest_wheel_command_;
      input_age = std::chrono::duration<double>(now - last_wheel_command_time_).count();
    }
  }

  if (!received) {
    // Nothing received yet (startup grace); do not enforce.
    commands_stale_.store(false);
    return;
  }

  // 1) Silence: the upstream command source (bridge/swerve/mux) stopped
  //    publishing entirely.
  const bool silence_stale = input_age > command_timeout_sec_;

  // 2) Divergence: the controller_manager is still driving write() but no
  //    longer applies fresh inputs (executor wedged), so the published command
  //    is frozen while the input has changed. Compare the controller INPUT with
  //    the command interface the controller is supposed to have written.
  bool diverging = false;
  if (latest_input.size() >= kNumWheelJoints) {
    for (size_t index = 0; index < kNumWheelJoints; ++index) {
      if (std::fabs(latest_input[index] - hw_wheel_commands_[index]) > command_divergence_eps_) {
        diverging = true;
        break;
      }
    }
  }

  if (diverging) {
    if (!divergence_active_) {
      divergence_active_ = true;
      divergence_since_ = now;
    }
  } else {
    divergence_active_ = false;
  }

  const bool divergence_stale =
    divergence_active_ &&
    std::chrono::duration<double>(now - divergence_since_).count() > command_timeout_sec_;

  const bool stale = silence_stale || divergence_stale;
  commands_stale_.store(stale);

  if (stale) {
    publish_stop_commands();
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("GripperXInterface"), *watchdog_node_->get_clock(), 1000,
      "Command watchdog STOP: %s (input_age=%.3fs). Publishing wheel-zero / steering-hold.",
      silence_stale ? "no fresh controller command" : "controller output frozen vs. fresh input",
      input_age);
  }
}

bool GripperXInterface::parse_joint_layout()
{
  steer_index_by_joint_.clear();
  wheel_index_by_joint_.clear();

  for (size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index) {
    const auto & joint_name = info_.joints[joint_index].name;

    for (size_t steer_index = 0; steer_index < kNumSteerJoints; ++steer_index) {
      if (joint_name == kSteerJointOrder[steer_index]) {
        steer_index_by_joint_[joint_name] = joint_index;
      }
    }

    for (size_t wheel_index = 0; wheel_index < kNumWheelJoints; ++wheel_index) {
      if (joint_name == kWheelJointOrder[wheel_index]) {
        wheel_index_by_joint_[joint_name] = joint_index;
      }
    }
  }

  if (steer_index_by_joint_.size() != kNumSteerJoints ||
    wheel_index_by_joint_.size() != kNumWheelJoints)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("GripperXInterface"),
      "Expected %zu steering and %zu wheel joints in URDF, found %zu and %zu.",
      kNumSteerJoints, kNumWheelJoints, steer_index_by_joint_.size(),
      wheel_index_by_joint_.size());
    return false;
  }

  return true;
}

}  // namespace gripperx_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  gripperx_hardware_interface::GripperXInterface, hardware_interface::SystemInterface)
