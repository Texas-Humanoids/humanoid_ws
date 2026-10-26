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

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "robstride_motor/can_protocol.hpp"

using namespace robstride_motor;

namespace
{

// Symmetric ranges used to clamp targets and decode status frames, from the RobStride manuals.
struct MotorLimits
{
  double position_max;  // rad
  double velocity_max;  // rad/s
  double torque_max;  // Nm
};

const std::map<std::string, MotorLimits> kMotorLimitsByModel = {
  {"RS03", {4.0 * M_PI, 20.0, 60.0}},
  {"RS06", {4.0 * M_PI, 50.0, 36.0}},
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
    // The motor stops itself if it hears no command for this long; 0 disables it.
    can_timeout_ = this->declare_parameter<double>("can_timeout", 0.5);
    if (!(can_timeout_ >= 0.0 && can_timeout_ <= 10.0) ||
      (can_timeout_ > 0.0 && can_timeout_ * feedback_rate < 2.0))
    {
      throw std::invalid_argument(
              "can_timeout must be 0 (off) or 0..10 s and at least two feedback periods");
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
    torque_publisher_ =
      this->create_publisher<std_msgs::msg::Float64>("motor/actual_torque", 10);
    temperature_publisher_ =
      this->create_publisher<std_msgs::msg::Float64>("motor/temperature", 10);
    status_publisher_ =
      this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("motor/status", 10);
    clear_faults_service_ = this->create_service<std_srvs::srv::Trigger>(
      "motor/clear_faults",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        this->clear_faults(*response);
      });

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

      start_motor();

      RCLCPP_WARN(
        this->get_logger(),
        "Sending real CAN commands to %s motor %ld on %s (CSP mode, %.2f rad/s, %.1f A, "
        "CAN timeout %.2f s). Starting position: %.3f rad.",
        motor_model_.c_str(), static_cast<long>(motor_id_), can_interface_.c_str(),
        speed_limit_, current_limit_, can_timeout_, target_position_);
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
  // Clears latched faults, configures CSP mode and enables at the current position.
  // Throws, leaving the motor stopped, if any step fails.
  void start_motor()
  {
    if (!send_frame(
        make_can_id(kCommTypeStop, kHostCanId, static_cast<uint8_t>(motor_id_)),
        encode_stop(true)) ||
      !write_param_u8(kParamRunMode, kCspRunMode) ||
      !write_param_float(kParamSpeedLimit, static_cast<float>(speed_limit_)) ||
      !write_param_float(kParamCurrentLimit, static_cast<float>(current_limit_)) ||
      !write_param_u32(kParamCanTimeout, can_timeout_counts(can_timeout_)) ||
      !send_frame(
        make_can_id(kCommTypeActiveReport, kHostCanId, static_cast<uint8_t>(motor_id_)),
        encode_active_report(true)))
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
  }

  void clear_faults(std_srvs::srv::Trigger::Response & response)
  {
    try {
      start_motor();
      fault_report_.clear();
      response.success = true;
      response.message = "Faults cleared; re-enabled at " + std::to_string(target_position_) +
        " rad";
      RCLCPP_WARN(this->get_logger(), "%s", response.message.c_str());
    } catch (const std::exception & error) {
      send_stop_frame();
      response.success = false;
      response.message = error.what();
      RCLCPP_ERROR(this->get_logger(), "Clearing faults failed: %s", error.what());
    }
  }

  void close_can_socket() noexcept
  {
    if (can_socket_ >= 0) {
      // Keep cleanup independent of ROS logging and other throwing operations.
      struct can_frame frame{};
      frame.can_id = make_can_id(
        kCommTypeActiveReport, kHostCanId, static_cast<uint8_t>(motor_id_)) | CAN_EFF_FLAG;
      frame.can_dlc = 8;
      const auto report_off = encode_active_report(false);
      std::memcpy(frame.data, report_off.data(), 8);
      (void)write(can_socket_, &frame, sizeof(frame));
      frame.can_id =
        make_can_id(kCommTypeStop, kHostCanId, static_cast<uint8_t>(motor_id_)) | CAN_EFF_FLAG;
      std::memset(frame.data, 0, 8);
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
  double can_timeout_ = 0.0;
  MotorLimits motor_limits_{};
  int can_socket_ = -1;

  double target_position_ = 0.0;

  MotorStatus status_{};
  std::chrono::steady_clock::time_point status_time_{};
  bool have_status_ = false;
  std::string fault_report_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
    position_subscription_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    actual_position_publisher_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    actual_velocity_publisher_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torque_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temperature_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_faults_service_;

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

  bool write_param_u32(uint16_t index, uint32_t value)
  {
    const uint32_t can_id =
      make_can_id(kCommTypeParamWrite, kHostCanId, static_cast<uint8_t>(motor_id_));
    return send_frame(can_id, encode_param_u32(index, value));
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
      if (frame_motor_id(arb) != static_cast<uint8_t>(motor_id_)) {
        continue;
      }

      std::array<uint8_t, 8> reply_data;
      std::memcpy(reply_data.data(), reply.data, 8);

      // Status and fault frames queue up between polls; handle them while waiting.
      const uint8_t type = frame_comm_type(arb);
      if (type == kCommTypeStatus || type == kCommTypeActiveReport) {
        on_status(decode_status(
            arb, reply_data, motor_limits_.position_max, motor_limits_.velocity_max,
            motor_limits_.torque_max));
      } else if (type == kCommTypeFaultReport) {
        on_fault_report(describe_fault_report(reply_data));
      } else if (type == kCommTypeParamRead && decode_param_index(reply_data) == index) {
        out_value = decode_param_float(reply_data);
        return true;
      }
    }

    return false;
  }

  void on_status(const MotorStatus & status)
  {
    if (!have_status_ || status.mode != status_.mode || status.faults != status_.faults) {
      if (status.faults != 0) {
        RCLCPP_ERROR(
          this->get_logger(), "Motor faults: %s (%s). Call motor/clear_faults to recover.",
          describe_status_faults(status.faults).c_str(), mode_name(status.mode));
      } else if (have_status_ && status_.mode == kModeRunning && status.mode != kModeRunning) {
        RCLCPP_ERROR(
          this->get_logger(), "Motor left run mode (%s), e.g. after a CAN timeout. "
          "Call motor/clear_faults to re-enable.", mode_name(status.mode));
      } else {
        RCLCPP_INFO(this->get_logger(), "Motor mode: %s", mode_name(status.mode));
      }
    }
    status_ = status;
    status_time_ = std::chrono::steady_clock::now();
    have_status_ = true;
  }

  void on_fault_report(const std::string & faults)
  {
    if (faults != fault_report_) {
      RCLCPP_ERROR(this->get_logger(), "Motor fault report: %s", faults.c_str());
      fault_report_ = faults;
    }
  }

  void publish_status()
  {
    if (!have_status_ ||
      std::chrono::steady_clock::now() - status_time_ > std::chrono::milliseconds(200))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "No status frames from the motor; torque, temperature and faults are unavailable");
      return;
    }
    std_msgs::msg::Float64 msg;
    msg.data = status_.torque;
    torque_publisher_->publish(msg);
    msg.data = status_.temperature;
    temperature_publisher_->publish(msg);

    diagnostic_msgs::msg::DiagnosticStatus diagnostic;
    diagnostic.name = this->get_fully_qualified_name();
    diagnostic.hardware_id = motor_model_ + " id " + std::to_string(motor_id_);
    if (status_.faults != 0) {
      diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diagnostic.message = describe_status_faults(status_.faults);
    } else {
      diagnostic.level = status_.mode == kModeRunning ?
        diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
      diagnostic.message = mode_name(status_.mode);
    }
    const auto value = [&diagnostic](const std::string & key, const std::string & text) {
        diagnostic_msgs::msg::KeyValue entry;
        entry.key = key;
        entry.value = text;
        diagnostic.values.push_back(entry);
      };
    value("mode", mode_name(status_.mode));
    value("faults", status_.faults ? describe_status_faults(status_.faults) : "none");
    value("fault_report", fault_report_.empty() ? "none" : fault_report_);
    value("temperature_c", std::to_string(status_.temperature));
    value("torque_nm", std::to_string(status_.torque));
    status_publisher_->publish(diagnostic);
  }

  void poll_feedback()
  {
    // Re-sending the current target keeps the motor's CAN timeout from expiring.
    write_param_float(kParamLocRef, static_cast<float>(target_position_));

    float position = 0.0f;
    float velocity = 0.0f;
    const bool have_feedback =
      read_param_float(kParamMechPos, position, kPollReadTimeout) &&
      read_param_float(kParamMechVel, velocity, kPollReadTimeout) &&
      std::isfinite(position) && std::isfinite(velocity);
    publish_status();
    if (!have_feedback) {
      return;
    }

    std_msgs::msg::Float64 msg;
    msg.data = position;
    actual_position_publisher_->publish(msg);
    msg.data = velocity;
    actual_velocity_publisher_->publish(msg);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Actual position: %.3f rad, velocity: %.3f rad/s, target: %.3f rad, %.1f C, %.2f Nm",
      position, velocity, target_position_, status_.temperature, status_.torque);
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
      message->data, -motor_limits_.position_max, motor_limits_.position_max);
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
