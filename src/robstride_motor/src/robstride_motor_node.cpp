#include <chrono>
#include <cmath>
#include <memory>
#include <cstdint>
#include <cstring>
#include <string>
#include <array>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace
{

struct MotorLimits
{
  double position_min;
  double position_max;
  double velocity_min;
  double velocity_max;
  double torque_min;
  double torque_max;
  double kp_min;
  double kp_max;
  double kd_min;
  double kd_max;
};

const MotorLimits kRS06Limits = {
  -4.0 * M_PI, 4.0 * M_PI,
  -50.0, 50.0,
  -36.0, 36.0,
  0.0, 5000.0,
  0.0, 100.0
};

constexpr uint8_t kHostCanId = 0xFF;
constexpr uint8_t kCommTypeEnable = 3;
constexpr uint8_t kCommTypeStop = 4;
constexpr uint8_t kCommTypeParamRead = 0x11;
constexpr uint8_t kCommTypeParamWrite = 0x12;

constexpr uint16_t kParamRunMode = 0x7005;
constexpr uint16_t kParamLocRef = 0x7016;
constexpr uint16_t kParamSpeedLimit = 0x7017;
constexpr uint16_t kParamCurrentLimit = 0x7018;
constexpr uint16_t kParamMechPos = 0x7019;

constexpr uint8_t kCspRunMode = 5;

}  // namespace

class RobstrideMotorNode : public rclcpp::Node
{
public:
  RobstrideMotorNode()
  : rclcpp::Node("robstride_motor_node")
  {
    can_interface_ =
      this->declare_parameter<std::string>("can_interface", "can0");

    motor_model_ =
      this->declare_parameter<std::string>("motor_model", "RS06");

    motor_id_ =
      this->declare_parameter<int64_t>("motor_id");

    if (motor_model_ == "RS06") {
      motor_limits_ = kRS06Limits;
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Unsupported motor_model '%s'; only RS06 limits are implemented.",
        motor_model_.c_str());
    }

    if (open_can_socket()) {
      send_stop_frame();

      write_param_u8(kParamRunMode, kCspRunMode);
      write_param_float(kParamSpeedLimit, 1.0f);
      write_param_float(kParamCurrentLimit, 3.0f);

      float current_position = 0.0f;
      if (read_param_float(kParamMechPos, current_position)) {
        target_position_ = current_position;
        write_param_float(kParamLocRef, target_position_);
        send_enable_frame();

        RCLCPP_WARN(
          this->get_logger(),
          "Sending real CAN commands to motor %ld on %s (CSP mode). Starting position: %.3f rad.",
          static_cast<long>(motor_id_),
          can_interface_.c_str(),
          target_position_);
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "Could not read starting position from motor %ld; refusing to enable.",
          static_cast<long>(motor_id_));
      }
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Could not open CAN socket on '%s'. Commands will not reach the motor.",
        can_interface_.c_str());
    }

    actual_position_publisher_ =
      this->create_publisher<std_msgs::msg::Float64>(
        "motor/actual_position", 10);

    position_subscription_ =
      this->create_subscription<std_msgs::msg::Float64>(
        "motor/target_position",
        10,
        [this](std_msgs::msg::Float64::ConstSharedPtr message)
        {
          this->on_position_command(message);
        });

    feedback_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]()
      {
        this->poll_feedback();
      });
  }

  void stop()
  {
    feedback_timer_->cancel();

    if (can_socket_ >= 0) {
      send_stop_frame();
      close(can_socket_);
      can_socket_ = -1;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Sent stop command to the motor and closed the CAN socket.");
  }

private:
  std::string can_interface_;
  std::string motor_model_;
  int64_t motor_id_;
  MotorLimits motor_limits_{};
  int can_socket_ = -1;

  double target_position_ = 0.0;
  int feedback_poll_count_ = 0;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
    position_subscription_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    actual_position_publisher_;

  rclcpp::TimerBase::SharedPtr feedback_timer_;

  bool open_can_socket()
  {
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
      return false;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      close(can_socket_);
      can_socket_ = -1;
      return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      close(can_socket_);
      can_socket_ = -1;
      return false;
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 20000;
    setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    return true;
  }

  void send_frame(uint32_t can_id, const std::array<uint8_t, 8> & data)
  {
    if (can_socket_ < 0) {
      return;
    }

    struct can_frame frame;
    frame.can_id = can_id | CAN_EFF_FLAG;
    frame.can_dlc = 8;
    std::memcpy(frame.data, data.data(), 8);

    if (write(can_socket_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
      RCLCPP_WARN(this->get_logger(), "CAN write failed.");
    }
  }

  void send_enable_frame()
  {
    const uint32_t can_id =
      (static_cast<uint32_t>(kCommTypeEnable) << 24) |
      (static_cast<uint32_t>(kHostCanId) << 8) |
      static_cast<uint32_t>(motor_id_);

    send_frame(can_id, {0, 0, 0, 0, 0, 0, 0, 0});
  }

  void send_stop_frame()
  {
    const uint32_t can_id =
      (static_cast<uint32_t>(kCommTypeStop) << 24) |
      (static_cast<uint32_t>(kHostCanId) << 8) |
      static_cast<uint32_t>(motor_id_);

    send_frame(can_id, {0, 0, 0, 0, 0, 0, 0, 0});
  }

  void write_param_u8(uint16_t index, uint8_t value)
  {
    const uint32_t can_id =
      (static_cast<uint32_t>(kCommTypeParamWrite) << 24) |
      (static_cast<uint32_t>(kHostCanId) << 8) |
      static_cast<uint32_t>(motor_id_);

    std::array<uint8_t, 8> data = {0, 0, 0, 0, 0, 0, 0, 0};
    data[0] = static_cast<uint8_t>(index & 0xFF);
    data[1] = static_cast<uint8_t>((index >> 8) & 0xFF);
    data[4] = value;

    send_frame(can_id, data);
  }

  void write_param_float(uint16_t index, float value)
  {
    const uint32_t can_id =
      (static_cast<uint32_t>(kCommTypeParamWrite) << 24) |
      (static_cast<uint32_t>(kHostCanId) << 8) |
      static_cast<uint32_t>(motor_id_);

    std::array<uint8_t, 8> data = {0, 0, 0, 0, 0, 0, 0, 0};
    data[0] = static_cast<uint8_t>(index & 0xFF);
    data[1] = static_cast<uint8_t>((index >> 8) & 0xFF);

    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    data[4] = static_cast<uint8_t>(bits & 0xFF);
    data[5] = static_cast<uint8_t>((bits >> 8) & 0xFF);
    data[6] = static_cast<uint8_t>((bits >> 16) & 0xFF);
    data[7] = static_cast<uint8_t>((bits >> 24) & 0xFF);

    send_frame(can_id, data);
  }

  bool read_param_float(uint16_t index, float & out_value)
  {
    const uint32_t can_id =
      (static_cast<uint32_t>(kCommTypeParamRead) << 24) |
      (static_cast<uint32_t>(kHostCanId) << 8) |
      static_cast<uint32_t>(motor_id_);

    std::array<uint8_t, 8> request = {0, 0, 0, 0, 0, 0, 0, 0};
    request[0] = static_cast<uint8_t>(index & 0xFF);
    request[1] = static_cast<uint8_t>((index >> 8) & 0xFF);
    send_frame(can_id, request);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    struct can_frame reply;

    while (std::chrono::steady_clock::now() < deadline) {
      const ssize_t n = read(can_socket_, &reply, sizeof(reply));
      if (n != static_cast<ssize_t>(sizeof(reply))) {
        continue;
      }

      const uint32_t arb = reply.can_id & CAN_EFF_MASK;
      if (((arb >> 24) & 0x1F) != kCommTypeParamRead) {
        continue;
      }

      const uint16_t reply_index =
        static_cast<uint16_t>(reply.data[0]) |
        (static_cast<uint16_t>(reply.data[1]) << 8);
      if (reply_index != index) {
        continue;
      }

      const uint8_t responding_motor_id = (arb >> 8) & 0xFF;
      if (responding_motor_id != static_cast<uint8_t>(motor_id_)) {
        continue;
      }

      uint32_t bits =
        static_cast<uint32_t>(reply.data[4]) |
        (static_cast<uint32_t>(reply.data[5]) << 8) |
        (static_cast<uint32_t>(reply.data[6]) << 16) |
        (static_cast<uint32_t>(reply.data[7]) << 24);
      std::memcpy(&out_value, &bits, sizeof(out_value));
      return true;
    }

    return false;
  }

  void poll_feedback()
  {
    float actual = 0.0f;
    if (!read_param_float(kParamMechPos, actual)) {
      return;
    }

    std_msgs::msg::Float64 msg;
    msg.data = actual;
    actual_position_publisher_->publish(msg);

    if (++feedback_poll_count_ % 10 == 0) {
      RCLCPP_INFO(this->get_logger(), "Actual position: %.3f rad", actual);
    }
  }

  void on_position_command(
    std_msgs::msg::Float64::ConstSharedPtr message)
  {
    if (!std::isfinite(message->data)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Ignoring a non-finite target position.");
      return;
    }

    double target = message->data;
    if (target < motor_limits_.position_min) { target = motor_limits_.position_min; }
    if (target > motor_limits_.position_max) { target = motor_limits_.position_max; }

    target_position_ = target;
    write_param_float(kParamLocRef, static_cast<float>(target_position_));

    RCLCPP_INFO(
      this->get_logger(),
      "New target: %.3f rad",
      target_position_);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RobstrideMotorNode>();
  rclcpp::spin(node);

  node->stop();
  rclcpp::shutdown();

  return 0;
}