#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "th_policy_bridge/joint_mapping.hpp"

namespace tb = th_policy_bridge;
using SteadyClock = std::chrono::steady_clock;

namespace
{

double seconds_since(const SteadyClock::time_point & stamp)
{
  return std::chrono::duration<double>(SteadyClock::now() - stamp).count();
}

}  // namespace

// Splits named policy targets into one robstride_motor command per joint and
// merges motor feedback back into /joint_states. Targets only reach the motors
// while armed from the gamepad.
class JointMotorBridge : public rclcpp::Node
{
public:
  JointMotorBridge()
  : Node("joint_motor_bridge")
  {
    const auto names = declare_parameter<std::vector<std::string>>(
      "joint_names", std::vector<std::string>{});
    const auto namespaces = declare_parameter<std::vector<std::string>>(
      "motor_namespaces", std::vector<std::string>{});
    const auto directions = declare_parameter<std::vector<double>>(
      "directions", std::vector<double>{});
    const auto offsets = declare_parameter<std::vector<double>>("offsets", std::vector<double>{});
    const auto lowers = declare_parameter<std::vector<double>>(
      "lower_limits", std::vector<double>{});
    const auto uppers = declare_parameter<std::vector<double>>(
      "upper_limits", std::vector<double>{});
    arm_event_ = declare_parameter<std::string>("arm_event", "START pressed");
    disarm_event_ = declare_parameter<std::string>("disarm_event", "BACK pressed");
    target_timeout_ = declare_parameter<double>("target_timeout", 0.2);
    feedback_timeout_ = declare_parameter<double>("feedback_timeout", 0.3);
    const double rate = declare_parameter<double>("publish_rate", 50.0);

    const size_t n = names.size();
    if (namespaces.size() != n || directions.size() != n || offsets.size() != n ||
      lowers.size() != n || uppers.size() != n)
    {
      throw std::invalid_argument(
              "joint_names, motor_namespaces, directions, offsets, lower_limits and "
              "upper_limits must have the same length");
    }
    if (!(target_timeout_ > 0.0) || !(feedback_timeout_ > 0.0) ||
      !std::isfinite(rate) || rate <= 0.0 || arm_event_.empty() || disarm_event_.empty())
    {
      throw std::invalid_argument("Invalid joint_motor_bridge parameters");
    }

    std::unordered_set<std::string> unique_namespaces;
    for (size_t i = 0; i < n; ++i) {
      const tb::JointMotor calibration{directions[i], offsets[i], lowers[i], uppers[i]};
      tb::validate(calibration, names[i]);
      if (names[i].empty() || !index_.emplace(names[i], i).second) {
        throw std::invalid_argument("Joint names must be nonempty and unique");
      }
      if (namespaces[i].size() < 2 || namespaces[i].front() != '/' ||
        !unique_namespaces.insert(namespaces[i]).second)
      {
        throw std::invalid_argument("Motor namespaces must be unique and start with '/'");
      }
      Joint joint;
      joint.name = names[i];
      joint.calibration = calibration;
      joint.command = create_publisher<std_msgs::msg::Float64>(
        namespaces[i] + "/motor/target_position", 10);
      joint.feedback = create_subscription<std_msgs::msg::Float64>(
        namespaces[i] + "/motor/actual_position", 10,
        [this, i](const std_msgs::msg::Float64 & msg) {on_feedback(i, msg.data);});
      joints_.push_back(std::move(joint));
    }
    if (joints_.empty()) {
      RCLCPP_WARN(get_logger(), "No motors configured; joint targets go nowhere");
    }

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      declare_parameter<std::string>("joint_state_topic", "/joint_states"), 10);
    target_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      declare_parameter<std::string>("joint_target_topic", "/policy/joint_targets"), 10,
      [this](const sensor_msgs::msg::JointState & msg) {on_targets(msg);});
    button_sub_ = create_subscription<std_msgs::msg::String>(
      declare_parameter<std::string>("button_topic", "/gamepad/button_event"), 10,
      [this](const std_msgs::msg::String & msg) {on_button(msg.data);});
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate)),
      [this]() {on_timer();});

    RCLCPP_INFO(
      get_logger(), "Bridging %zu motors. Disarmed; '%s' arms, '%s' disarms.",
      joints_.size(), arm_event_.c_str(), disarm_event_.c_str());
  }

private:
  struct Joint
  {
    std::string name;
    tb::JointMotor calibration{};
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr command;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr feedback;
    double position{};
    double velocity{};
    SteadyClock::time_point stamp{};
  };

  bool feedback_fresh(const Joint & joint) const
  {
    return joint.stamp != SteadyClock::time_point{} &&
           seconds_since(joint.stamp) <= feedback_timeout_;
  }

  bool targets_fresh() const
  {
    return target_time_ != SteadyClock::time_point{} &&
           seconds_since(target_time_) <= target_timeout_;
  }

  void on_feedback(size_t i, double motor_position)
  {
    if (!std::isfinite(motor_position)) {
      return;
    }
    auto & joint = joints_[i];
    const auto now = SteadyClock::now();
    const double position = tb::motor_to_joint(joint.calibration, motor_position);
    // The motor node reports position only; estimate velocity between samples.
    if (feedback_fresh(joint)) {
      const double dt = std::chrono::duration<double>(now - joint.stamp).count();
      if (dt > 1e-4) {
        joint.velocity = (position - joint.position) / dt;
      }
    } else {
      joint.velocity = 0.0;
    }
    joint.position = position;
    joint.stamp = now;
  }

  void on_targets(const sensor_msgs::msg::JointState & msg)
  {
    if (msg.position.size() != msg.name.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Ignoring malformed joint targets");
      return;
    }
    target_time_ = SteadyClock::now();
    if (!armed_) {
      return;
    }
    for (size_t i = 0; i < msg.name.size(); ++i) {
      const auto it = index_.find(msg.name[i]);
      if (it == index_.end() || !std::isfinite(msg.position[i])) {
        continue;
      }
      auto & joint = joints_[it->second];
      std_msgs::msg::Float64 command;
      command.data = tb::joint_to_motor(joint.calibration, msg.position[i]);
      joint.command->publish(command);
    }
  }

  void on_button(const std::string & event)
  {
    if (event == disarm_event_) {
      disarm("disarm button");
    } else if (event == arm_event_ && !armed_) {
      for (const auto & joint : joints_) {
        if (!feedback_fresh(joint)) {
          RCLCPP_WARN(get_logger(), "Not arming: no fresh feedback from %s", joint.name.c_str());
          return;
        }
      }
      if (!targets_fresh()) {
        RCLCPP_WARN(get_logger(), "Not arming: the policy is not publishing targets");
        return;
      }
      armed_ = true;
      RCLCPP_WARN(get_logger(), "ARMED: forwarding policy targets to %zu motors", joints_.size());
    }
  }

  void disarm(const char * reason)
  {
    if (armed_) {
      armed_ = false;
      RCLCPP_WARN(get_logger(), "Disarmed (%s); motors hold their last target", reason);
    }
  }

  void on_timer()
  {
    if (armed_ && !targets_fresh()) {
      disarm("policy targets timed out");
    }
    sensor_msgs::msg::JointState state;
    state.header.stamp = now();
    for (const auto & joint : joints_) {
      if (feedback_fresh(joint)) {
        state.name.push_back(joint.name);
        state.position.push_back(joint.position);
        state.velocity.push_back(joint.velocity);
      } else {
        disarm("motor feedback timed out");
      }
    }
    if (!state.name.empty()) {
      joint_state_pub_->publish(state);
    }
  }

  std::vector<Joint> joints_;
  std::unordered_map<std::string, size_t> index_;
  std::string arm_event_;
  std::string disarm_event_;
  double target_timeout_{};
  double feedback_timeout_{};
  bool armed_{false};
  SteadyClock::time_point target_time_{};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr button_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<JointMotorBridge>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("joint_motor_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
