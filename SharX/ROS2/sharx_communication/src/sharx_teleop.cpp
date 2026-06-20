#include <chrono>
#include <memory>
#include <termios.h>
#include <unistd.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class TerminalGuard
{
public:
  TerminalGuard()
  {
    tcgetattr(STDIN_FILENO, &original_);

    termios raw = original_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }

  ~TerminalGuard()
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &original_);
  }

private:
  termios original_{};
};

class SharXTeleop : public rclcpp::Node
{
public:
  SharXTeleop()
  : Node("sharx_teleop")
  {
    publisher_ =
      create_publisher<geometry_msgs::msg::Twist>(
      "/sharx/cmd_vel",
      10);

    linear_speed_ =
      declare_parameter<double>("linear_speed", 0.35);

    angular_speed_ =
      declare_parameter<double>("angular_speed", 0.35);

    timer_ = create_wall_timer(
      50ms,
      std::bind(&SharXTeleop::keyboard_callback, this));

    RCLCPP_INFO(get_logger(), "SharX teleop started");
    RCLCPP_INFO(get_logger(), "W: forward");
    RCLCPP_INFO(get_logger(), "S: reverse");
    RCLCPP_INFO(get_logger(), "A: turn left");
    RCLCPP_INFO(get_logger(), "D: turn right");
    RCLCPP_INFO(get_logger(), "Space: stop");
    RCLCPP_INFO(get_logger(), "Q: stop and quit");
  }

private:
  void keyboard_callback()
  {
    char key{};

    const ssize_t count =
      read(STDIN_FILENO, &key, sizeof(key));

    if (count <= 0) {
      return;
    }

    geometry_msgs::msg::Twist command;

    switch (key) {
      case 'w':
      case 'W':
        command.linear.x = linear_speed_;
        break;

      case 's':
      case 'S':
        command.linear.x = -linear_speed_;
        break;

      case 'a':
      case 'A':
        command.angular.z = angular_speed_;
        break;

      case 'd':
      case 'D':
        command.angular.z = -angular_speed_;
        break;

      case ' ':
        break;

      case 'q':
      case 'Q':
        publish_stop();
        rclcpp::shutdown();
        return;

      default:
        return;
    }

    publisher_->publish(command);

    RCLCPP_INFO(
      get_logger(),
      "linear.x=%.2f angular.z=%.2f",
      command.linear.x,
      command.angular.z);
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist command;
    publisher_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  double linear_speed_{0.35};
  double angular_speed_{0.35};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  TerminalGuard terminal_guard;

  rclcpp::spin(
    std::make_shared<SharXTeleop>());

  return 0;
}
