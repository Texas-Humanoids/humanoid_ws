#include <chrono>
#include <cmath>
#include <memory>
#include <cstdint>
#include <cstring>
#include <string>
#include <array>
#include <stdexcept>
#include <map>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "robstride_motor/can_protocol.hpp"

using namespace robstride_motor;

namespace
{

struct MotorLimits
{
  double position_min;
  double position_max;
};

// Position ranges from the RobStride manuals; every model in this family uses +/-4 pi.
const std::map<std::string, MotorLimits> kMotorLimitsByModel = {
  {"RS03", {-4.0 * M_PI, 4.0 * M_PI}},
  {"RS06", {-4.0 * M_PI, 4.0 * M_PI}},
};

}  // namespace

class RobstrideMotorNode : public rclcpp::Node
{
public:
  explicit RobstrideMotorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("robstride_motor_node", options)
  {
    can_interface_ =
      this->declare_parameter<std::string>("can_interface", "can0");

    motor_model_ =
      this->declare_parameter<std::string>("motor_model", "RS06");

    speed_limit_ = this->declare_parameter<double>("speed_limit", 1.0);
    current_limit_ = this->declare_parameter<double>("current_limit", 3.0);
    const double feedback_rate = this->declare_parameter<double>("feedback_rate", 50.0);
    if (!(speed_limit_ > 0.0) || !std::isfinite(speed_limit_) ||
      !(current_limit_ > 0.0) || !std::isfinite(current_limit_) ||
      !(feedback_rate > 0.0) || !std::isfinite(feedback_rate))
    {
      throw std::invalid_argument("speed_limit, current_limit and feedback_rate must be > 0");
    }

    motor_id_ =
      this->declare_parameter<int64_t>("motor_id");
    if (motor_id_ < 0 || motor_id_ > 127) {
      throw std::invalid_argument("motor_id must be in the range 0..127");
    }

    const auto it = kMotorLimitsByModel.find(motor_model_);
    if (it != kMotorLimitsByModel.end()) {
      motor_limits_ = it->second;
    } else {
      throw std::runtime_error(
      "Unsupported motor_model '" + motor_model_ + "'; supported models: RS03, RS06");
    }

    actual_position_publisher_ =
      this->create_publisher<std_msgs::msg::Float64>(
        "motor/actual_position", 10);
    actual_velocity_publisher_ =
      this->create_publisher<std_msgs::msg::Float64>(
        "motor/actual_velocity", 10);

    position_subscription_ =
      this->create_subscription<std_msgs::msg::Float64>(
        "motor/target_position",
        10,
      [this](std_msgs::msg::Float64::ConstSharedPtr message)
      {
        this->on_position_command(message);
        });

    feedback_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / feedback_rate)),
      [this]()
      {
        this->poll_feedback();
      });

    try {
      if (!open_can_socket()) {
        throw std::runtime_error("Could not open CAN socket on '" + can_interface_ + "'");
      }

      if (!send_stop_frame() ||
        !write_param_u8(kParamRunMode, kCspRunMode) ||
        !write_param_float(kParamSpeedLimit, static_cast<float>(speed_limit_)) ||
        !write_param_float(kParamCurrentLimit, static_cast<float>(current_limit_)))
      {
        throw std::runtime_error("CAN configuration write failed; refusing to enable");
      }

      float current_position = 0.0f;
      if (!read_param_float(kParamMechPos, current_position, kStartupReadTimeout) ||
        !std::isfinite(current_position))
      {
        throw std::runtime_error("Could not read a finite starting position; refusing to enable");
      }

      target_position_ = current_position;
      if (!write_param_float(kParamLocRef, current_position) || !send_enable_frame()) {
        throw std::runtime_error("CAN startup write failed; refusing to continue");
      }

      RCLCPP_WARN(
        this->get_logger(),
        "Sending real CAN commands to %s motor %ld on %s (CSP mode, %.2f rad/s, %.1f A). "
        "Starting position: %.3f rad.",
        motor_model_.c_str(), static_cast<long>(motor_id_), can_interface_.c_str(),
        speed_limit_, current_limit_, target_position_);
    } catch (...) {
      // A failed constructor does not invoke this class's destructor.
      close_can_socket();
      throw;
    }
  }

  ~RobstrideMotorNode() override
  {
    close_can_socket();
  }

  void stop()
  {
    feedback_timer_->cancel();
    close_can_socket();
  }

private:
  void close_can_socket() noexcept
  {
    if (can_socket_ >= 0) {
      // Keep cleanup independent of ROS logging and other throwing operations.
      struct can_frame frame{};
      frame.can_id =
        make_can_id(kCommTypeStop, kHostCanId, static_cast<uint8_t>(motor_id_)) | CAN_EFF_FLAG;
      frame.can_dlc = 8;
      (void)write(can_socket_, &frame, sizeof(frame));
      close(can_socket_);
      can_socket_ = -1;
    }
  }

  std::string can_interface_;
  std::string motor_model_;
  int64_t motor_id_;
  double speed_limit_ = 0.0;
  double current_limit_ = 0.0;
  MotorLimits motor_limits_{};
  int can_socket_ = -1;

  double target_position_ = 0.0;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
    position_subscription_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    actual_position_publisher_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    actual_velocity_publisher_;

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
    tv.tv_usec = 2000;  // read_param_float loops until its own deadline
    setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    return true;
  }

  bool send_frame(uint32_t can_id, const std::array<uint8_t, 8> & data)
  {
    if (can_socket_ < 0) {
      return false;
    }

    struct can_frame frame;
    frame.can_id = can_id | CAN_EFF_FLAG;
    frame.can_dlc = 8;
    std::memcpy(frame.data, data.data(), 8);

    if (write(can_socket_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
      RCLCPP_WARN(this->get_logger(), "CAN write failed.");
      return false;
    }
    return true;
  }

  bool send_enable_frame()
  {
    const uint32_t can_id =
      make_can_id(kCommTypeEnable, kHostCanId, static_cast<uint8_t>(motor_id_));
    return send_frame(can_id, {0, 0, 0, 0, 0, 0, 0, 0});
  }

  bool send_stop_frame()
  {
    const uint32_t can_id =
      make_can_id(kCommTypeStop, kHostCanId, static_cast<uint8_t>(motor_id_));
    return send_frame(can_id, {0, 0, 0, 0, 0, 0, 0, 0});
  }

  bool write_param_u8(uint16_t index, uint8_t value)
  {
    const uint32_t can_id =
      make_can_id(kCommTypeParamWrite, kHostCanId, static_cast<uint8_t>(motor_id_));
    return send_frame(can_id, encode_param_u8(index, value));
  }

  bool write_param_float(uint16_t index, float value)
  {
    const uint32_t can_id =
      make_can_id(kCommTypeParamWrite, kHostCanId, static_cast<uint8_t>(motor_id_));
    return send_frame(can_id, encode_param_float(index, value));
  }

  static constexpr std::chrono::milliseconds kStartupReadTimeout{200};
  // Short enough that a silent motor cannot stall the 50 Hz feedback loop.
  static constexpr std::chrono::milliseconds kPollReadTimeout{5};

  bool read_param_float(uint16_t index, float & out_value, std::chrono::milliseconds timeout)
  {
    if (can_socket_ < 0) {
      return false;
    }

    const uint32_t can_id =
      make_can_id(kCommTypeParamRead, kHostCanId, static_cast<uint8_t>(motor_id_));
    if (!send_frame(can_id, encode_param_index(index))) {
      return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + timeout;
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

      std::array<uint8_t, 8> reply_data;
      std::memcpy(reply_data.data(), reply.data, 8);

      if (decode_param_index(reply_data) != index) {
        continue;
      }

      const uint8_t responding_motor_id = (arb >> 8) & 0xFF;
      if (responding_motor_id != static_cast<uint8_t>(motor_id_)) {
        continue;
      }

      out_value = decode_param_float(reply_data);
      return true;
    }

    return false;
  }

  void poll_feedback()
  {
    float position = 0.0f;
    float velocity = 0.0f;
    if (!read_param_float(kParamMechPos, position, kPollReadTimeout) ||
      !read_param_float(kParamMechVel, velocity, kPollReadTimeout) ||
      !std::isfinite(position) || !std::isfinite(velocity))
    {
      return;
    }

    std_msgs::msg::Float64 msg;
    msg.data = position;
    actual_position_publisher_->publish(msg);
    msg.data = velocity;
    actual_velocity_publisher_->publish(msg);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Actual position: %.3f rad, velocity: %.3f rad/s, target: %.3f rad",
      position, velocity, target_position_);
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

    target_position_ = clamp_position(
      message->data, motor_limits_.position_min, motor_limits_.position_max);
    if (!write_param_float(kParamLocRef, static_cast<float>(target_position_))) {
      return;
    }

    RCLCPP_DEBUG(
      this->get_logger(),
      "New target: %.3f rad",
      target_position_);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<RobstrideMotorNode>();
    rclcpp::spin(node);
    node->stop();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("robstride_motor_node"), "Motor node failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
