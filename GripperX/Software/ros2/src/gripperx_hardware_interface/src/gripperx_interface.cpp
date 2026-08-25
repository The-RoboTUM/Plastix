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

// hw/joint_states layout. Indices 0..7 (kNumJoints) are the original contract and stay
// the MINIMUM a publisher has to provide — hw_firmware_mock and any older firmware are
// still accepted. Firmware with encoder feedback (HWR-10) appends 4 measured wheel
// positions in rad at 8..11; those are only read when the message is long enough.
constexpr size_t kWheelPositionOffset = kNumJoints;
constexpr size_t kNumStateValuesWithWheelPositions = kWheelPositionOffset + kNumWheelJoints;

// Firmware with the provenance block (FR-11 items 5/6) appends 4 more values at 12..15,
// one EncoderStatus code per wheel in the same FL, FR, BL, BR order. Read only when the
// message is long enough — same shape as the wheel-position guard above, and for the same
// reason: a short message must degrade to "unknown", never to "valid".
constexpr size_t kWheelProvenanceOffset = kNumStateValuesWithWheelPositions;
constexpr size_t kNumStateValuesWithProvenance = kWheelProvenanceOffset + kNumWheelJoints;

constexpr std::array<int, 4> kAllProvenanceUnknown = {
  kProvenanceUnknown, kProvenanceUnknown, kProvenanceUnknown, kProvenanceUnknown};

const char * provenance_name(int provenance)
{
  switch (provenance) {
    case kProvenanceNoEncoder:
      return "NO_ENCODER";
    case kProvenanceInitFailed:
      return "INIT_FAILED";
    case kProvenanceLiveUnconfirmed:
      return "LIVE_UNCONFIRMED";
    case kProvenanceLive:
      return "LIVE";
    default:
      return "UNKNOWN";
  }
}

// A measurement, or an echo of the command? The single place that decides.
bool provenance_is_measurement(int provenance)
{
  return provenance >= kProvenanceLiveUnconfirmed;
}

// The wire carries doubles. Anything that is not one of the firmware's codes — a NaN, a
// value from a foreign publisher, a future code this build does not know — decodes to
// UNKNOWN rather than to the nearest known value.
int decode_provenance(double raw)
{
  if (!std::isfinite(raw)) {
    return kProvenanceUnknown;
  }
  const double rounded = std::round(raw);
  if (std::fabs(raw - rounded) > 1e-6) {
    return kProvenanceUnknown;
  }
  const int code = static_cast<int>(rounded);
  if (code < kProvenanceNoEncoder || code > kProvenanceLive) {
    return kProvenanceUnknown;
  }
  return code;
}

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

  const auto steer_states_topic_it = info_.hardware_parameters.find("steer_states_topic");
  if (steer_states_topic_it != info_.hardware_parameters.end()) {
    steer_states_topic_ = steer_states_topic_it->second;
  }

  const auto steer_states_timeout_it =
    info_.hardware_parameters.find("steer_states_timeout_sec");
  if (steer_states_timeout_it != info_.hardware_parameters.end()) {
    steer_states_timeout_sec_ = std::stod(steer_states_timeout_it->second);
  }

  const auto steer_states_activation_timeout_it =
    info_.hardware_parameters.find("steer_states_activation_timeout_sec");
  if (steer_states_activation_timeout_it != info_.hardware_parameters.end()) {
    steer_states_activation_timeout_sec_ = std::stod(steer_states_activation_timeout_it->second);
  }

  if (steer_states_activation_timeout_sec_ < 0.0) {
    RCLCPP_WARN(
      rclcpp::get_logger("GripperXInterface"),
      "steer_states_activation_timeout_sec must be >= 0; falling back to %.1f s.",
      kDefaultSteerStatesActivationTimeoutSec);
    steer_states_activation_timeout_sec_ = kDefaultSteerStatesActivationTimeoutSec;
  }

  const auto steer_states_valid_topic_it =
    info_.hardware_parameters.find("steer_states_valid_topic");
  if (steer_states_valid_topic_it != info_.hardware_parameters.end()) {
    steer_states_valid_topic_ = steer_states_valid_topic_it->second;
  }

  const auto wheel_feedback_valid_topic_it =
    info_.hardware_parameters.find("wheel_feedback_valid_topic");
  if (wheel_feedback_valid_topic_it != info_.hardware_parameters.end()) {
    wheel_feedback_valid_topic_ = wheel_feedback_valid_topic_it->second;
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

  // OP-18a: which reference input the watchdog polices against. Selectable so
  // that switching the running robot from the superseded per-wheel reference to
  // the W2 twist/echo pair is a configuration change in
  // gripperx_v1.ros2_control.xacro and not a code change, and so that both can
  // exist while the old chain is still the active one. An unknown value is a
  // hard ERROR, not a silent fallback: guessing here would silently choose
  // which safety check runs.
  const auto wd_reference_it = info_.hardware_parameters.find("watchdog_reference");
  if (wd_reference_it != info_.hardware_parameters.end()) {
    const std::string & reference = wd_reference_it->second;
    if (reference == "wheel_commands") {
      watchdog_reference_ = WatchdogReference::kWheelCommands;
    } else if (reference == "twist_echo") {
      watchdog_reference_ = WatchdogReference::kTwistEcho;
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("GripperXInterface"),
        "watchdog_reference must be 'wheel_commands' or 'twist_echo', got '%s'.",
        reference.c_str());
      return CallbackReturn::ERROR;
    }
  }

  const auto cmd_vel_topic_it = info_.hardware_parameters.find("cmd_vel_topic");
  if (cmd_vel_topic_it != info_.hardware_parameters.end()) {
    cmd_vel_topic_ = cmd_vel_topic_it->second;
  }

  const auto echo_topic_it = info_.hardware_parameters.find("intent_echo_topic");
  if (echo_topic_it != info_.hardware_parameters.end()) {
    intent_echo_topic_ = echo_topic_it->second;
  }

  const auto twist_linear_eps_it = info_.hardware_parameters.find("command_twist_linear_eps");
  if (twist_linear_eps_it != info_.hardware_parameters.end()) {
    twist_tolerance_.linear = std::stod(twist_linear_eps_it->second);
  }

  const auto twist_angular_eps_it = info_.hardware_parameters.find("command_twist_angular_eps");
  if (twist_angular_eps_it != info_.hardware_parameters.end()) {
    twist_tolerance_.angular = std::stod(twist_angular_eps_it->second);
  }

  if (twist_tolerance_.linear < 0.0 || twist_tolerance_.angular < 0.0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GripperXInterface"),
      "command_twist_linear_eps / command_twist_angular_eps must be >= 0.");
    return CallbackReturn::ERROR;
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

  {
    std::lock_guard<std::mutex> lock(steer_states_mutex_);
    steer_states_received_ = false;
    latest_steer_states_.clear();
  }
  steer_states_valid_ = false;
  steer_states_valid_published_ = false;
  wheel_feedback_provenance_ = kAllProvenanceUnknown;
  wheel_feedback_valid_published_ = false;
  joint_states_received_ = false;

  // Subscriptions first, PUBLISHERS LATER. Everything that can put a message on
  // /hw/joint_commands — joint_commands_pub_ here and watchdog_commands_pub_ in
  // start_watchdog() — is created only AFTER the activation gate below has passed, so a
  // refused activation cannot leave a command behind on the wire (SR-14 acceptance:
  // "activation fails visibly and no command is published"). Do not hoist them back up.
  joint_states_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    joint_states_topic_, rclcpp::SensorDataQoS(),
    std::bind(&GripperXInterface::joint_states_callback, this, std::placeholders::_1));

  // QoS stated explicitly, NOT SystemDefaultsQoS(). steer_servo_node publishes with the
  // rclpy default profile (`create_publisher(..., 10)` = KEEP_LAST(10), RELIABLE,
  // VOLATILE). SystemDefaultsQoS() leaves reliability as SYSTEM_DEFAULT, which resolves
  // to the DDS DataReader default (BEST_EFFORT) under rmw_fastrtps — the recorded
  // incident on the watchdog subscription below, where a BEST_EFFORT reader saw one
  // sample in 14 minutes against a RELIABLE 30 Hz writer. A steering measurement that
  // silently never arrives is exactly the failure FR-10 exists to remove, so the reader
  // matches the writer deliberately.
  rclcpp::QoS steer_states_qos(rclcpp::KeepLast(10));
  steer_states_qos.reliable();
  steer_states_qos.durability_volatile();
  steer_states_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    steer_states_topic_, steer_states_qos,
    std::bind(&GripperXInterface::steer_states_callback, this, std::placeholders::_1));

  // ── SR-14 item 4: the activation gate ──────────────────────────────────────────────
  // An activation that cannot read a valid actual steering position MUST NOT activate on
  // the interface default. The failure this closes: power off -> servos limp -> the wheels
  // are displaced by hand, terrain or transport -> power on -> a position controller
  // activates before ANY measurement has arrived and writes the command interface's
  // initial 0.0, i.e. CENTRE. Nobody asked for that motion; a controller merely came up.
  //
  // Bounded, not instant (NFR-3): on a clean bringup the first /hw/steer_states arrives
  // ~5.9 s after this component starts activating, because steer_servo_node has to scan
  // the servo bus first. Failing instantly would turn a safety guard into a boot failure.
  // The window is steer_states_activation_timeout_sec_ — see the xacro for how the value
  // was derived from that measurement.
  //
  // NOT gated on /hw/joint_states: the wheels are commanded to zero, which is safe from
  // any starting condition, so their feedback is not a precondition for a safe activation.
  // Only the steering carries a position command.
  std::vector<double> measured_steering;
  if (!wait_for_steer_states(measured_steering)) {
    joint_states_sub_.reset();
    steer_states_sub_.reset();
    {
      std::lock_guard<std::mutex> lock(steer_states_mutex_);
      steer_states_received_ = false;
      latest_steer_states_.clear();
    }
    joint_states_received_ = false;
    return CallbackReturn::ERROR;
  }

  // The measurement is adopted BEFORE anything is published, into both:
  //  * the position STATE interfaces, so the first controller update() already sees the
  //    real angle rather than the 0.0 that read() would only overwrite one cycle later;
  //  * the position COMMAND interfaces, so that the command this component publishes at
  //    30 Hz from now until a controller writes for the first time is "hold where you
  //    are" and not "go to centre" (SR-14 item 1, hardware-component half).
  for (size_t index = 0; index < kNumSteerJoints; ++index) {
    hw_steering_commands_[index] = measured_steering[index];
    const auto joint_it = steer_index_by_joint_.find(kSteerJointOrder[index]);
    if (joint_it != steer_index_by_joint_.end()) {
      hw_positions_[joint_it->second] = measured_steering[index];
    }
  }
  std::fill(hw_wheel_commands_.begin(), hw_wheel_commands_.end(), 0.0);

  // Latched health flag (SR-13): a late subscriber gets the current value immediately
  // instead of having to wait for the next edge.
  rclcpp::QoS steer_states_valid_qos(rclcpp::KeepLast(1));
  steer_states_valid_qos.reliable();
  steer_states_valid_qos.transient_local();
  steer_states_valid_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    steer_states_valid_topic_, steer_states_valid_qos);

  // Same latched profile, same reason (FR-11 item 6 / SR-13): the provenance of the
  // wheel feedback is a state, not an event, and a subscriber that comes up late must
  // learn it immediately rather than after the next change — which, on a healthy robot,
  // may be never.
  rclcpp::QoS wheel_feedback_valid_qos(rclcpp::KeepLast(1));
  wheel_feedback_valid_qos.reliable();
  wheel_feedback_valid_qos.transient_local();
  wheel_feedback_valid_pub_ = node->create_publisher<std_msgs::msg::Int32MultiArray>(
    wheel_feedback_valid_topic_, wheel_feedback_valid_qos);

  joint_commands_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    joint_commands_topic_, rclcpp::SystemDefaultsQoS());

  active_.store(true);
  commands_stale_.store(false);

  // First command out of this component on this activation. NOT publish_zero_commands():
  // eight zeros put 0.0 on the four steering slots, and steer_servo_node reads those slots
  // as an angle command — so "zero" is a CENTRE command with the servos' full torque
  // behind it, which is precisely the unrequested motion SR-14 exists to prevent. Wheels
  // zero, steering at the measurement we just waited for.
  publish_stop_commands();

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
    "Activated. commands=%s states=%s steer_states=%s (timeout %.3fs). Steering held at the "
    "measured angle [%.4f, %.4f, %.4f, %.4f] rad (FL, FR, BL, BR), wheels zero.",
    joint_commands_topic_.c_str(), joint_states_topic_.c_str(),
    steer_states_topic_.c_str(), steer_states_timeout_sec_,
    measured_steering[0], measured_steering[1], measured_steering[2], measured_steering[3]);

  return CallbackReturn::SUCCESS;
}

// Blocks until /hw/steer_states delivers one measurement that is present, fresh and
// finite, or until the activation window expires. Returns false on expiry, having said
// exactly why (SR-13: the failure must be readable at 2 a.m. without guessing).
//
// The subscription being waited on is the REAL FR-10 subscription on the component's own
// node, deliberately — a private node with a private subscription could match a publisher
// that the real one does not, and then the gate would attest to a data path other than the
// one the robot actually uses. That this works, i.e. that callbacks are serviced while
// on_activate blocks, is not an assumption: the pre-existing /hw/joint_states wait below
// has been returning successfully on every hardware bringup.
bool GripperXInterface::wait_for_steer_states(std::vector<double> & measured)
{
  const auto start = std::chrono::steady_clock::now();
  const auto deadline = start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(steer_states_activation_timeout_sec_));

  const char * reason = "no message has ever been received";
  bool announced_wait = false;

  while (rclcpp::ok()) {
    std::vector<double> latest;
    bool received = false;
    double age = 0.0;
    {
      std::lock_guard<std::mutex> lock(steer_states_mutex_);
      received = steer_states_received_;
      if (received) {
        latest = latest_steer_states_;
        age = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_steer_states_time_).count();
      }
    }

    if (received) {
      const bool fresh = steer_states_timeout_sec_ <= 0.0 || age <= steer_states_timeout_sec_;
      const bool finite = std::all_of(
        latest.begin(), latest.end(), [](double value) { return std::isfinite(value); });

      if (fresh && finite) {
        measured = latest;
        RCLCPP_INFO(
          rclcpp::get_logger("GripperXInterface"),
          "Steering measurement acquired from %s after %.3f s; activation may proceed "
          "(SR-14).",
          steer_states_topic_.c_str(),
          std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
        return true;
      }

      // A sample that arrived and then went stale means the publisher started and
      // stopped — a different fault from "never there", and worth distinguishing.
      reason = fresh ? "the last message contains a non-finite value"
                     : "messages arrived and then stopped (the publisher died)";
    }

    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      break;
    }

    if (!announced_wait &&
      std::chrono::duration<double>(now - start).count() > 1.0)
    {
      announced_wait = true;
      RCLCPP_INFO(
        rclcpp::get_logger("GripperXInterface"),
        "Waiting for the first steering measurement on %s before activating (up to %.1f s). "
        "steer_servo_node scans the servo bus at startup, so this normally takes a few "
        "seconds (SR-14 / NFR-3).",
        steer_states_topic_.c_str(), steer_states_activation_timeout_sec_);
    }

    rclcpp::sleep_for(std::chrono::milliseconds(50));
  }

  size_t publisher_count = 0;
  if (auto node = get_node()) {
    publisher_count = node->count_publishers(steer_states_topic_);
  }

  RCLCPP_ERROR(
    rclcpp::get_logger("GripperXInterface"),
    "ACTIVATION REFUSED (SR-14): no valid steering measurement on %s within %.1f s — %s. "
    "Publishers currently seen on that topic: %zu. The hardware component is NOT active and "
    "NO command was published. Without a measured steering angle an activating controller "
    "would command the interface default 0.0, i.e. steer to CENTRE — movement nobody asked "
    "for, at full servo torque. Fix the measurement, do not bypass it: %s is published by "
    "steer_servo_node (30 Hz). Check `journalctl -u gripperx-bringup.service | grep "
    "steer_servo_node` for its servo-bus scan, and that /dev/steering_servo exists.",
    steer_states_topic_.c_str(), steer_states_activation_timeout_sec_, reason,
    publisher_count, steer_states_topic_.c_str());

  return false;
}

CallbackReturn GripperXInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  active_.store(false);

  stop_watchdog();

  // Last command out of this component on this deactivation. NOT publish_zero_commands():
  // eight zeros put 0.0 on the four steering slots, and steer_servo_node reads those slots
  // as an ANGLE command — so deactivating would drive four live servos to CENTRE. That is
  // the SR-13 incident path (ESP32 flash -> /hw/joint_states stale -> read() returns ERROR
  // -> the component deactivates), i.e. a fault would answer with unrequested motion.
  // OP-24 / S1 is the rule at every stage: wheels to zero, steering holds its last
  // commanded angle. Same call, same reason, as on_activate().
  //
  // Asymmetry worth knowing on THIS path only: stop_watchdog() above has already reset
  // watchdog_commands_pub_, so of publish_stop_commands()'s deliberately dual publish only
  // the joint_commands_pub_ half actually goes out here -- the other is a no-op via its
  // null check. Not a fault (joint_commands_pub_ is the half steer_servo_node reads), but
  // the redundancy the dual publish exists for is NOT available during deactivation.
  publish_stop_commands();

  joint_states_sub_.reset();
  joint_commands_pub_.reset();
  joint_states_received_ = false;

  steer_states_sub_.reset();
  steer_states_valid_pub_.reset();
  wheel_feedback_valid_pub_.reset();
  {
    std::lock_guard<std::mutex> lock(steer_states_mutex_);
    steer_states_received_ = false;
    latest_steer_states_.clear();
  }
  steer_states_valid_ = false;
  steer_states_valid_published_ = false;
  wheel_feedback_provenance_ = kAllProvenanceUnknown;
  wheel_feedback_valid_published_ = false;

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
  // FR-10: the steering position state has its own source and its own freshness rule.
  // Done before the /hw/joint_states handling so that a missing or stale ESP32 stream
  // (early return / ERROR below) does not also freeze the steering feedback path.
  update_steering_position_states();

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
    // Nothing has arrived at all, so nothing is known about the wheel feedback either.
    set_wheel_feedback_provenance(kAllProvenanceUnknown);
    return hardware_interface::return_type::OK;
  }

  if (state_timeout_sec_ > 0.0) {
    const auto age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - latest_time).count();
    if (age > state_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("GripperXInterface"), *get_node()->get_clock(), 2000,
        "%s stale (%.3f s old).", joint_states_topic_.c_str(), age);
      // A frozen array is not a current measurement, whatever its provenance codes said
      // when it was fresh.
      set_wheel_feedback_provenance(kAllProvenanceUnknown);
      return hardware_interface::return_type::ERROR;
    }
  }

  if (latest_states.data.size() < kNumJoints) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("GripperXInterface"), *get_node()->get_clock(), 2000,
      "%s expected %zu values, got %zu.",
      joint_states_topic_.c_str(), kNumJoints, latest_states.data.size());
    set_wheel_feedback_provenance(kAllProvenanceUnknown);
    return hardware_interface::return_type::ERROR;
  }

  // Steering: indices 0-3 of /hw/joint_states are deliberately IGNORED (FR-10). The ESP32
  // has no steering sensor and never writes them, so they are a constant 0.0 that used to
  // be presented as a measurement. The position state is filled by
  // update_steering_position_states() from /hw/steer_states instead. The steering VELOCITY
  // state stays 0.0: steer_servo_node reports angle only, and differentiating it here would
  // invent a signal rather than measure one.
  for (size_t index = 0; index < kNumSteerJoints; ++index) {
    const auto joint_it = steer_index_by_joint_.find(kSteerJointOrder[index]);
    if (joint_it == steer_index_by_joint_.end()) {
      continue;
    }
    hw_velocities_[joint_it->second] = 0.0;
  }

  // Only firmware with encoder feedback publishes the appended wheel-position block.
  // Without it the wheel position state stays at its previous value rather than being
  // forced to zero, so a mid-run firmware downgrade does not produce a position jump.
  const bool has_wheel_positions =
    latest_states.data.size() >= kNumStateValuesWithWheelPositions;

  // Length guard for the provenance block, deliberately the SAME SHAPE as the one
  // above: a message too short to carry provenance maps to UNKNOWN — "we do not know
  // whether this is a measurement" — and never to a valid code. Silence must not be
  // readable as an assurance.
  const bool has_provenance =
    latest_states.data.size() >= kNumStateValuesWithProvenance;

  std::array<int, kNumWheelJoints> provenance = kAllProvenanceUnknown;
  if (has_provenance) {
    for (size_t index = 0; index < kNumWheelJoints; ++index) {
      provenance[index] =
        decode_provenance(latest_states.data[kWheelProvenanceOffset + index]);
    }
  }

  for (size_t index = 0; index < kNumWheelJoints; ++index) {
    const auto joint_it = wheel_index_by_joint_.find(kWheelJointOrder[index]);
    if (joint_it == wheel_index_by_joint_.end()) {
      continue;
    }
    const size_t joint_index = joint_it->second;

    // DELIBERATE ASYMMETRY WITH THE STEERING PATH ABOVE — DO NOT "FIX" IT INTO
    // CONSISTENCY. update_steering_position_states() HOLDS the last valid steering angle
    // when its source degrades. The wheel VELOCITY must NOT be held, and the difference
    // is not an oversight:
    //   * a held steering ANGLE describes a wheel that is still pointing where it was
    //     pointing, which is true — the servos hold position;
    //   * a held non-zero VELOCITY asserts that a possibly stationary robot is still
    //     moving, and it feeds odometry, which would integrate that fiction into a
    //     position error that grows without bound.
    // So the velocity is passed through exactly as the firmware sent it, whatever its
    // provenance, and the provenance topic below carries the "this is not a measurement"
    // verdict instead. Disabling any later closed loop on that verdict is FR-11 item 6 —
    // it is the consumer's job, not this line's.
    hw_velocities_[joint_index] = latest_states.data[kNumSteerJoints + index];

    if (has_wheel_positions) {
      hw_positions_[joint_index] = latest_states.data[kWheelPositionOffset + index];
    }
  }

  if (!has_wheel_positions) {
    // Option D. This used to be a WARN about odometry staying flat, which named a
    // symptom and buried the cause. A publisher that sends only the 8-value minimum has
    // NO ENCODERS AT ALL: indices 4-7 are then MotorController::getRPM()'s fallback
    // branch, i.e. the commanded velocity handed straight back. Everything downstream
    // sees a wheel that tracks its setpoint perfectly, because it IS its setpoint.
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("GripperXInterface"), *get_node()->get_clock(), 10000,
      "%s carries only %zu values (< %zu): this publisher has NO encoder feedback. "
      "Indices 4-7 are the COMMAND ECHOED BACK, not a measurement, and wheel odometry "
      "stays flat. All four wheels reported as provenance UNKNOWN (FR-11 items 5/6).",
      joint_states_topic_.c_str(), latest_states.data.size(),
      kNumStateValuesWithWheelPositions);
  } else if (!has_provenance) {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("GripperXInterface"), *get_node()->get_clock(), 10000,
      "%s carries %zu values (< %zu): wheel positions are present but the per-wheel "
      "provenance block is absent, so it is UNKNOWN whether indices 4-7 are measured or "
      "the command echoed back. Flash firmware with the provenance block (FR-11 item 5).",
      joint_states_topic_.c_str(), latest_states.data.size(),
      kNumStateValuesWithProvenance);
  }

  set_wheel_feedback_provenance(provenance);

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

void GripperXInterface::steer_states_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  // A short message is not a measurement: reject it instead of letting it refresh the
  // timestamp, otherwise a malformed publisher would keep the feedback looking healthy.
  if (msg->data.size() < kNumSteerJoints) {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("GripperXInterface"), *get_node()->get_clock(), 2000,
      "%s expected %zu values, got %zu; message discarded.",
      steer_states_topic_.c_str(), kNumSteerJoints, msg->data.size());
    return;
  }

  std::lock_guard<std::mutex> lock(steer_states_mutex_);
  latest_steer_states_.assign(msg->data.begin(), msg->data.begin() + kNumSteerJoints);
  last_steer_states_time_ = std::chrono::steady_clock::now();
  steer_states_received_ = true;
}

void GripperXInterface::update_steering_position_states()
{
  // FR-10 / §3.1.2 stage 8. Joint order on /hw/steer_states is FL, FR, BL, BR
  // (steer_servo_node STEER_JOINT_NAMES), which is kSteerJointOrder verbatim — no
  // remapping, and none may be introduced silently.
  std::vector<double> latest;
  bool received = false;
  double age = 0.0;

  {
    std::lock_guard<std::mutex> lock(steer_states_mutex_);
    received = steer_states_received_;
    if (received) {
      latest = latest_steer_states_;
      age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_steer_states_time_).count();
    }
  }

  if (!received) {
    // SR-13: never silent. Note that the state interfaces are NOT zeroed here — they keep
    // whatever they last held (0.0 only if nothing was ever received), and the health flag
    // says the value is not a measurement.
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("GripperXInterface"), *get_node()->get_clock(), 5000,
      "No %s received: the steering position state interfaces carry NO measurement. "
      "Is steer_servo_node running? (FR-10)",
      steer_states_topic_.c_str());
    set_steer_states_valid(false);
    return;
  }

  const bool fresh = steer_states_timeout_sec_ <= 0.0 || age <= steer_states_timeout_sec_;
  if (!fresh) {
    // Deliberately NOT a fallback to 0.0 (FR-10 item 3) and deliberately NOT
    // return_type::ERROR: a lost steering readback must not deactivate the hardware
    // component and take the whole drive path inactive with it (SR-13's recorded
    // incident). Hold the last valid measurement — consistent with OP-24/S1, hold on
    // loss — and make the degradation loud instead.
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("GripperXInterface"), *get_node()->get_clock(), 2000,
      "%s stale (%.3f s > %.3f s): HOLDING the last valid steering measurement. "
      "The steering position state is not current (FR-10 / SR-13).",
      steer_states_topic_.c_str(), age, steer_states_timeout_sec_);
    set_steer_states_valid(false);
    return;
  }

  for (size_t index = 0; index < kNumSteerJoints; ++index) {
    const auto joint_it = steer_index_by_joint_.find(kSteerJointOrder[index]);
    if (joint_it == steer_index_by_joint_.end()) {
      continue;
    }
    hw_positions_[joint_it->second] = latest[index];
  }

  set_steer_states_valid(true);
}

void GripperXInterface::set_steer_states_valid(bool valid)
{
  if (steer_states_valid_published_ && valid == steer_states_valid_) {
    return;
  }

  const bool was_valid = steer_states_valid_;
  const bool had_published = steer_states_valid_published_;
  steer_states_valid_ = valid;
  steer_states_valid_published_ = true;

  if (steer_states_valid_pub_) {
    std_msgs::msg::Bool msg;
    msg.data = valid;
    steer_states_valid_pub_->publish(msg);
  }

  if (valid && had_published && !was_valid) {
    RCLCPP_INFO(
      rclcpp::get_logger("GripperXInterface"),
      "%s recovered; steering position state is a live measurement again.",
      steer_states_topic_.c_str());
  }
}

void GripperXInterface::set_wheel_feedback_provenance(
  const std::array<int, kNumWheelJoints> & provenance)
{
  if (wheel_feedback_valid_published_ && provenance == wheel_feedback_provenance_) {
    return;
  }

  const auto previous = wheel_feedback_provenance_;
  const bool had_published = wheel_feedback_valid_published_;
  wheel_feedback_provenance_ = provenance;
  wheel_feedback_valid_published_ = true;

  if (wheel_feedback_valid_pub_) {
    std_msgs::msg::Int32MultiArray msg;
    msg.data.assign(provenance.begin(), provenance.end());
    wheel_feedback_valid_pub_->publish(msg);
  }

  // Log per wheel and only on the wheels that actually changed, so a single encoder
  // waking up does not reprint the other three. The wheel NAME is in the line: the
  // whole point of per-wheel provenance is being able to say which wheel.
  for (size_t index = 0; index < kNumWheelJoints; ++index) {
    if (had_published && previous[index] == provenance[index]) {
      continue;
    }
    const bool measurement = provenance_is_measurement(provenance[index]);
    if (measurement) {
      RCLCPP_INFO(
        rclcpp::get_logger("GripperXInterface"),
        "Wheel feedback provenance %s: %s -> %s (velocity IS a measurement).",
        kWheelJointOrder[index],
        had_published ? provenance_name(previous[index]) : "(none)",
        provenance_name(provenance[index]));
    } else {
      // SR-13: an echoed value must be a visible state, never a silent default.
      RCLCPP_ERROR(
        rclcpp::get_logger("GripperXInterface"),
        "Wheel feedback provenance %s: %s -> %s (velocity is NOT a measurement; it is "
        "the command echoed back or unknown — no closed loop may use it, FR-11 item 6).",
        kWheelJointOrder[index],
        had_published ? provenance_name(previous[index]) : "(none)",
        provenance_name(provenance[index]));
    }
  }
}

// UNUSED as of 2026-08-18 and deliberately kept unused: its last two call sites
// (on_activate, on_deactivate) were both moved to publish_stop_commands() because eight
// zeros are a CENTRE command for the four steering slots, not a stop (OP-24 / S1, SR-14).
// Do not reintroduce a call to this without re-deciding OP-24 — there is currently no
// situation in which this component may command the steering to zero on its own initiative.
// Retained only so the distinction stays legible; a later cleanup may delete it outright.
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
  {
    std::lock_guard<std::mutex> lock(twist_watchdog_mutex_);
    twist_watchdog_.configure(command_timeout_sec_, twist_tolerance_);
    twist_watchdog_.reset();
  }

  // Dedicated node/executor/thread, independent of the controller_manager
  // executor (the failure candidate). A subscription serviced by our own
  // executor keeps receiving fresh controller inputs even if the CM executor
  // is wedged; our own publisher can emit a stop even if the CM node is stuck.
  //
  // Under variant B this is MORE load-bearing, not less (OP-18b): kinematics,
  // controllers, hardware interface and main publisher then all live in
  // ros2_control_node, and this thread is the only in-process element left that
  // can emit a stop when that process's executor wedges. It is not weakened
  // here — both W2 subscriptions are added to this same node.
  watchdog_node_ = std::make_shared<rclcpp::Node>("gripperx_interface_watchdog");

  // EXPLICIT QoS ON EVERY WATCHDOG INPUT, in every reference mode.
  // SystemDefaultsQoS() leaves reliability as SYSTEM_DEFAULT, which resolves to the
  // DDS DataReader default (BEST_EFFORT) under rmw_fastrtps. On this stack a
  // BEST_EFFORT reader effectively did NOT receive the RELIABLE stream (verified:
  // the watchdog saw a single sample over 14 min while a RELIABLE reader saw 30 Hz),
  // so the deadman stayed latched in silence-STOP and zeroed /hw/joint_commands,
  // blocking all driving. Match the publisher explicitly so the safety deadman
  // actually observes the live command stream (RELIABLE also avoids best-effort
  // drops looking like silence / falsely tripping the divergence check).
  // SR-11 (4) / OP-18a item 2 make this binding for the NEW inputs too.
  rclcpp::QoS command_qos(rclcpp::KeepLast(10));
  command_qos.reliable();

  if (watchdog_reference_ == WatchdogReference::kWheelCommands) {
    // Superseded reference (SR-11 D2): the policed topic itself. Kept working
    // unchanged until the NFR-10 rebuild takes over the active path.
    wheel_command_sub_ = watchdog_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      wheel_command_topic_, command_qos,
      std::bind(&GripperXInterface::wheel_command_input_callback, this, std::placeholders::_1));
  } else {
    // OP-18a / W2. Two inputs, both upstream of and distinct from the policed
    // topic, both with a single designated writer, and neither requiring the
    // kinematics here — a twist is compared with a twist.
    cmd_vel_sub_ = watchdog_node_->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, command_qos,
      std::bind(&GripperXInterface::cmd_vel_input_callback, this, std::placeholders::_1));
    intent_echo_sub_ =
      watchdog_node_->create_subscription<gripperx_control_msgs::msg::SwerveIntentEcho>(
        intent_echo_topic_, command_qos,
        std::bind(&GripperXInterface::intent_echo_callback, this, std::placeholders::_1));
  }

  watchdog_commands_pub_ = watchdog_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    joint_commands_topic_, rclcpp::SystemDefaultsQoS());

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / command_watchdog_rate_hz_));
  watchdog_timer_ = watchdog_node_->create_wall_timer(
    period, std::bind(&GripperXInterface::watchdog_check, this));

  watchdog_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  watchdog_executor_->add_node(watchdog_node_);
  watchdog_thread_ = std::thread([this]() { watchdog_executor_->spin(); });

  if (watchdog_reference_ == WatchdogReference::kWheelCommands) {
    RCLCPP_INFO(
      rclcpp::get_logger("GripperXInterface"),
      "Command watchdog active. reference=wheel_commands input=%s timeout=%.3fs rate=%.1fHz "
      "eps=%.4f",
      wheel_command_topic_.c_str(), command_timeout_sec_, command_watchdog_rate_hz_,
      command_divergence_eps_);
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("GripperXInterface"),
      "Command watchdog active. reference=twist_echo inputs=%s + %s timeout=%.3fs rate=%.1fHz "
      "twist_eps=(%.2e m/s, %.2e rad/s, both TO-VERIFY)",
      cmd_vel_topic_.c_str(), intent_echo_topic_.c_str(), command_timeout_sec_,
      command_watchdog_rate_hz_, twist_tolerance_.linear, twist_tolerance_.angular);
  }
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
  cmd_vel_sub_.reset();
  intent_echo_sub_.reset();
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

void GripperXInterface::cmd_vel_input_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  const TwistSample twist{msg->linear.x, msg->linear.y, msg->angular.z};
  std::lock_guard<std::mutex> lock(twist_watchdog_mutex_);
  twist_watchdog_.on_cmd_vel(twist, std::chrono::steady_clock::now());
}

void GripperXInterface::intent_echo_callback(
  const gripperx_control_msgs::msg::SwerveIntentEcho::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  // Only `sequence` is consumed. The carried twist is deliberately NOT compared
  // against /cmd_vel: the two legitimately differ for up to one control period
  // in normal operation, and a value comparison would re-introduce exactly the
  // race the counter exists to avoid. The twist stays on the wire because it is
  // what makes the echo readable in a bag when something did go wrong.
  std::lock_guard<std::mutex> lock(twist_watchdog_mutex_);
  twist_watchdog_.on_echo(msg->sequence, std::chrono::steady_clock::now());
}

void GripperXInterface::watchdog_check()
{
  if (!command_watchdog_enabled_ || !active_.load() || command_timeout_sec_ <= 0.0) {
    commands_stale_.store(false);
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (watchdog_reference_ == WatchdogReference::kTwistEcho) {
    watchdog_check_twist_echo(now);
  } else {
    watchdog_check_wheel_commands(now);
  }
}

void GripperXInterface::watchdog_check_wheel_commands(
  const std::chrono::steady_clock::time_point & now)
{
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

// OP-18a / W2 — §3.1.5. The two failure modes the superseded check distinguished
// are reproduced here on inputs that survive the NFR-10 rebuild, and NEITHER
// needs the kinematics: /cmd_vel is a twist, the echo carries a twist, and the
// discrimination is done on the echo's monotonic counter.
void GripperXInterface::watchdog_check_twist_echo(const std::chrono::steady_clock::time_point & now)
{
  TwistEchoWatchdog::Verdict verdict;
  {
    std::lock_guard<std::mutex> lock(twist_watchdog_mutex_);
    verdict = twist_watchdog_.evaluate(now);
  }

  if (!verdict.enforcing) {
    commands_stale_.store(false);
    return;
  }

  if (verdict.echo_never_seen) {
    // Reported, never latched — see the reasoning in TwistEchoWatchdog::evaluate.
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("GripperXInterface"), *watchdog_node_->get_clock(), 5000,
      "Command watchdog: %s is flowing but no intent echo has ever arrived on %s. The divergence "
      "check is INACTIVE until the first echo. Normal for the startup window; persistent means "
      "swerve_controller is not running or the topic name does not match.",
      cmd_vel_topic_.c_str(), intent_echo_topic_.c_str());
  }

  // Kept in step with the superseded branch so both modes report the same shape
  // of state, and so a mode switch does not silently change the log.
  divergence_active_ = verdict.divergence;
  if (verdict.divergence) {
    divergence_since_ = now;
  }

  const bool stale = verdict.silence || verdict.divergence;
  commands_stale_.store(stale);

  if (stale) {
    publish_stop_commands();
    if (verdict.silence) {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("GripperXInterface"), *watchdog_node_->get_clock(), 1000,
        "Command watchdog STOP: no fresh command on %s (age=%.3fs). Publishing wheel-zero / "
        "steering-hold.",
        cmd_vel_topic_.c_str(), verdict.cmd_vel_age_sec);
    } else {
      RCLCPP_ERROR_THROTTLE(
        rclcpp::get_logger("GripperXInterface"), *watchdog_node_->get_clock(), 1000,
        "Command watchdog STOP: DIVERGENCE. %s is fresh (age=%.3fs) and has changed, but the "
        "swerve_controller intent echo counter has stood still for %.3fs -- the controller is no "
        "longer consuming commands (wedged executor: incident 2026-07-06, recurrence 2026-08-17). "
        "Publishing wheel-zero / steering-hold.",
        cmd_vel_topic_.c_str(), verdict.cmd_vel_age_sec, verdict.echo_frozen_sec);
    }
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
