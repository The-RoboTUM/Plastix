#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std::chrono_literals;

class ThrusterMixer : public rclcpp::Node
{
public:
  ThrusterMixer()
  : Node("thruster_mixer")
  {
    maximum_output_ =
      declare_parameter<double>("maximum_output", 0.35);

    command_timeout_ =
      declare_parameter<double>("command_timeout", 0.5);

    invert_left_ =
      declare_parameter<bool>("invert_left", false);

    invert_right_ =
      declare_parameter<bool>("invert_right", false);

    subscription_ =
      create_subscription<geometry_msgs::msg::Twist>(
      "/sharx/cmd_vel",
      10,
      std::bind(
        &ThrusterMixer::command_callback,
        this,
        std::placeholders::_1));

    publisher_ =
      create_publisher<std_msgs::msg::Float32MultiArray>(
      "/sharx/thruster_command",
      10);

    watchdog_ = create_wall_timer(
      100ms,
      std::bind(
        &ThrusterMixer::watchdog_callback,
        this));

    last_command_time_ = now();

    RCLCPP_INFO(get_logger(), "Thruster mixer started");
    RCLCPP_INFO(
      get_logger(),
      "Output limit: %.2f",
      maximum_output_);
  }

private:
  void command_callback(
    const geometry_msgs::msg::Twist::SharedPtr message)
  {
    last_command_time_ = now();
    command_active_ = true;

    const double forward =
      std::clamp(message->linear.x, -1.0, 1.0);

    const double turn =
      std::clamp(message->angular.z, -1.0, 1.0);

    // Differential-thrust mixing.
    double left = forward - turn;
    double right = forward + turn;

    left = std::clamp(
      left,
      -maximum_output_,
      maximum_output_);

    right = std::clamp(
      right,
      -maximum_output_,
      maximum_output_);

    if (invert_left_) {
      left = -left;
    }

    if (invert_right_) {
      right = -right;
    }

    publish_command(left, right);

    RCLCPP_INFO(
      get_logger(),
      "Left=%.2f Right=%.2f",
      left,
      right);
  }

  void watchdog_callback()
  {
    if (!command_active_) {
      return;
    }

    const double elapsed =
      (now() - last_command_time_).seconds();

    if (elapsed > command_timeout_) {
      publish_command(0.0, 0.0);
      command_active_ = false;

      RCLCPP_WARN(
        get_logger(),
        "Command timeout: thrusters stopped");
    }
  }

  void publish_command(
    double left,
    double right)
  {
    std_msgs::msg::Float32MultiArray message;

    message.data = {
      static_cast<float>(left),
      static_cast<float>(right)
    };

    publisher_->publish(message);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    subscription_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
    publisher_;

  rclcpp::TimerBase::SharedPtr watchdog_;

  rclcpp::Time last_command_time_;

  double maximum_output_{0.35};
  double command_timeout_{0.5};

  bool invert_left_{false};
  bool invert_right_{false};
  bool command_active_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<ThrusterMixer>());

  rclcpp::shutdown();

  return 0;
}
