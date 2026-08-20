#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>


class ESP32SerialBridge : public rclcpp::Node
{
public:
  ESP32SerialBridge()
  : Node("esp32_serial_bridge"), serial_fd_(-1)
  {
    serial_port_ = declare_parameter<std::string>(
      "serial_port",
      "/dev/ttyACM0"
    );

    baud_rate_ = declare_parameter<int>(
      "baud_rate",
      115200
    );

    open_serial_port();

    subscription_ =
      create_subscription<std_msgs::msg::Float32MultiArray>(
        "/sharx/thruster_command",
        10,
        std::bind(
          &ESP32SerialBridge::thruster_callback,
          this,
          std::placeholders::_1
        )
      );

    // Check ESP32 messages continuously.
    serial_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(
        &ESP32SerialBridge::read_serial,
        this
      )
    );

    RCLCPP_INFO(
      get_logger(),
      "ESP32 serial bridge started"
    );

    RCLCPP_INFO(
      get_logger(),
      "Serial port: %s @ %d",
      serial_port_.c_str(),
      baud_rate_
    );

    RCLCPP_INFO(
      get_logger(),
      "Listening: /sharx/thruster_command"
    );
  }


  ~ESP32SerialBridge()
  {
    if (serial_fd_ >= 0)
    {
      close(serial_fd_);
    }
  }


private:

  void open_serial_port()
  {
    serial_fd_ = open(
      serial_port_.c_str(),
      O_RDWR | O_NOCTTY | O_NONBLOCK
    );

    if (serial_fd_ < 0)
    {
      throw std::runtime_error(
        "Could not open " +
        serial_port_ +
        ": " +
        std::strerror(errno)
      );
    }

    struct termios tty {};

    if (tcgetattr(serial_fd_, &tty) != 0)
    {
      close(serial_fd_);
      serial_fd_ = -1;

      throw std::runtime_error(
        "Failed to read serial port configuration"
      );
    }

    if (baud_rate_ != 115200)
    {
      RCLCPP_WARN(
        get_logger(),
        "Currently only 115200 baud is supported"
      );
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag =
      (tty.c_cflag & ~CSIZE) | CS8;

    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    tty.c_iflag &=
      ~(IXON | IXOFF | IXANY);

    tty.c_cflag |=
      (CLOCAL | CREAD);

    tty.c_cflag &=
      ~(PARENB | PARODD);

    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (
      tcsetattr(
        serial_fd_,
        TCSANOW,
        &tty
      ) != 0
    )
    {
      close(serial_fd_);
      serial_fd_ = -1;

      throw std::runtime_error(
        "Failed to configure serial port"
      );
    }

    tcflush(
      serial_fd_,
      TCIOFLUSH
    );
  }


  void thruster_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2)
    {
      RCLCPP_WARN(
        get_logger(),
        "Invalid thruster command: expected 2 values"
      );

      return;
    }

    const float left = msg->data[0];
    const float right = msg->data[1];

    std::ostringstream command;

    command
      << "THRUST,"
      << std::fixed
      << std::setprecision(3)
      << left
      << ","
      << right
      << "\n";

    const std::string command_string =
      command.str();

    const ssize_t bytes_written =
      write(
        serial_fd_,
        command_string.c_str(),
        command_string.size()
      );

    if (bytes_written < 0)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Serial write failed: %s",
        std::strerror(errno)
      );

      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "TX -> ESP32: THRUST,%.3f,%.3f",
      left,
      right
    );
  }


  void read_serial()
  {
    char buffer[256];

    while (true)
    {
      const ssize_t bytes_read =
        read(
          serial_fd_,
          buffer,
          sizeof(buffer)
        );

      if (bytes_read > 0)
      {
        receive_buffer_.append(
          buffer,
          bytes_read
        );

        process_received_lines();
      }
      else
      {
        if (
          bytes_read < 0 &&
          errno != EAGAIN &&
          errno != EWOULDBLOCK
        )
        {
          RCLCPP_ERROR(
            get_logger(),
            "Serial read failed: %s",
            std::strerror(errno)
          );
        }

        break;
      }
    }
  }


  void process_received_lines()
  {
    std::size_t newline_position;

    while (
      (
        newline_position =
          receive_buffer_.find('\n')
      ) != std::string::npos
    )
    {
      std::string response =
        receive_buffer_.substr(
          0,
          newline_position
        );

      receive_buffer_.erase(
        0,
        newline_position + 1
      );

      if (
        !response.empty() &&
        response.back() == '\r'
      )
      {
        response.pop_back();
      }

      if (response.empty())
      {
        continue;
      }

      if (
        response.rfind(
          "THRUST_OK,",
          0
        ) == 0
      )
      {
        RCLCPP_INFO(
          get_logger(),
          "RX <- ESP32: %s",
          response.c_str()
        );
      }

      else if (
        response.rfind(
          "WATCHDOG_STOP,",
          0
        ) == 0
      )
      {
        RCLCPP_WARN(
          get_logger(),
          "ESP32 WATCHDOG: %s",
          response.c_str()
        );
      }

      else if (
        response.rfind(
          "ERR,",
          0
        ) == 0
      )
      {
        RCLCPP_ERROR(
          get_logger(),
          "ESP32 error: %s",
          response.c_str()
        );
      }

      else
      {
        RCLCPP_INFO(
          get_logger(),
          "RX <- ESP32: %s",
          response.c_str()
        );
      }
    }
  }


  std::string serial_port_;
  int baud_rate_;
  int serial_fd_;

  std::string receive_buffer_;

  rclcpp::Subscription<
    std_msgs::msg::Float32MultiArray
  >::SharedPtr subscription_;

  rclcpp::TimerBase::SharedPtr serial_timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(
    argc,
    argv
  );

  try
  {
    rclcpp::spin(
      std::make_shared<
        ESP32SerialBridge
      >()
    );
  }
  catch (const std::exception & error)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger(
        "esp32_serial_bridge"
      ),
      "%s",
      error.what()
    );
  }

  rclcpp::shutdown();

  return 0;
}