#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "th_policy_bridge/joint_mapping.hpp"
#include "th_policy_bridge/observation.hpp"

namespace tb = th_policy_bridge;
using SteadyClock = std::chrono::steady_clock;

namespace
{

bool fresh(const SteadyClock::time_point & stamp, double timeout)
{
  return stamp != SteadyClock::time_point{} &&
         std::chrono::duration<double>(SteadyClock::now() - stamp).count() <= timeout;
}

}  // namespace

// Connects policy_node to the robot. Each tick it builds an observation from
// /cmd_vel, the IMU and motor feedback; it forwards the policy's joint targets
// to the motor nodes only while armed from the gamepad.
class PolicyBridge : public rclcpp::Node
{
public:
  PolicyBridge()
  : Node("policy_bridge")
  {
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", std::vector<std::string>{});
    defaults_ = declare_parameter<std::vector<double>>(
      "default_joint_positions", std::vector<double>{});
    action_scale_ = declare_parameter<double>("action_scale", 0.25);
    const auto motor_joints = declare_parameter<std::vector<std::string>>(
      "motor_joints", std::vector<std::string>{});
    const auto namespaces = declare_parameter<std::vector<std::string>>(
      "motor_namespaces", std::vector<std::string>{});
    const auto directions = declare_parameter<std::vector<double>>(
      "directions", std::vector<double>{});
    const auto offsets = declare_parameter<std::vector<double>>("offsets", std::vector<double>{});
    const auto lowers = declare_parameter<std::vector<double>>(
      "lower_limits", std::vector<double>{});
    const auto uppers = declare_parameter<std::vector<double>>(
      "upper_limits", std::vector<double>{});
    const double rate = declare_parameter<double>("rate", 25.0);
    cmd_vel_timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.5);
    imu_timeout_ = declare_parameter<double>("imu_timeout", 0.1);
    feedback_timeout_ = declare_parameter<double>("feedback_timeout", 0.1);
    target_timeout_ = declare_parameter<double>("target_timeout", 0.2);
    require_imu_ = declare_parameter<bool>("require_imu", true);
    arm_event_ = declare_parameter<std::string>("arm_event", "START pressed");
    disarm_event_ = declare_parameter<std::string>("disarm_event", "BACK pressed");

    if (joint_names_.empty() || joint_names_.size() != defaults_.size() ||
      !std::all_of(defaults_.begin(), defaults_.end(), [](double v) {return std::isfinite(v);}) ||
      !std::isfinite(action_scale_) || action_scale_ == 0.0 ||
      !std::isfinite(rate) || rate <= 0.0 || !(cmd_vel_timeout_ > 0.0) ||
      !(imu_timeout_ > 0.0) || !(feedback_timeout_ > 0.0) || !(target_timeout_ > 0.0))
    {
      throw std::invalid_argument("Invalid policy_bridge parameters");
    }
    std::unordered_map<std::string, size_t> policy_index;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (!policy_index.emplace(joint_names_[i], i).second) {
        throw std::invalid_argument("Duplicate policy joint " + joint_names_[i]);
      }
    }

    const size_t n = motor_joints.size();
    if (namespaces.size() != n || directions.size() != n || offsets.size() != n ||
      lowers.size() != n || uppers.size() != n)
    {
      throw std::invalid_argument(
              "motor_joints, motor_namespaces, directions, offsets, lower_limits and "
              "upper_limits must have the same length");
    }
    std::unordered_set<std::string> used;
    for (size_t m = 0; m < n; ++m) {
      const auto it = policy_index.find(motor_joints[m]);
      if (it == policy_index.end()) {
        throw std::invalid_argument(motor_joints[m] + " is not a policy joint");
      }
      if (namespaces[m].size() < 2 || namespaces[m].front() != '/' ||
        !used.insert(motor_joints[m]).second || !used.insert(namespaces[m]).second)
      {
        throw std::invalid_argument(
                "Motor joints and namespaces must be unique; namespaces start with '/'");
      }
      Motor motor;
      motor.joint = it->second;
      motor.calibration = {directions[m], offsets[m], lowers[m], uppers[m]};
      tb::validate(motor.calibration, motor_joints[m]);
      motor.command = create_publisher<std_msgs::msg::Float64>(
        namespaces[m] + "/motor/target_position", 10);
      motor.position_sub = create_subscription<std_msgs::msg::Float64>(
        namespaces[m] + "/motor/actual_position", 10,
        [this, m](const std_msgs::msg::Float64 & msg) {
          if (std::isfinite(msg.data)) {
            motors_[m].position = tb::motor_to_joint(motors_[m].calibration, msg.data);
            motors_[m].stamp = SteadyClock::now();
          }
        });
      motor.velocity_sub = create_subscription<std_msgs::msg::Float64>(
        namespaces[m] + "/motor/actual_velocity", 10,
        [this, m](const std_msgs::msg::Float64 & msg) {
          if (std::isfinite(msg.data)) {
            motors_[m].velocity = motors_[m].calibration.direction * msg.data;
          }
        });
      motors_.push_back(std::move(motor));
    }
    previous_actions_.assign(joint_names_.size(), 0.0);

    if (!require_imu_) {
      RCLCPP_WARN(get_logger(), "require_imu is false: assuming upright and not rotating");
    }
    std::vector<bool> has_motor(joint_names_.size(), false);
    for (const auto & motor : motors_) {has_motor[motor.joint] = true;}
    const auto missing = std::count(has_motor.begin(), has_motor.end(), false);
    if (missing > 0) {
      RCLCPP_WARN(
        get_logger(), "%ld of %zu policy joints have no motor and read as the default pose",
        static_cast<long>(missing), joint_names_.size());
    }

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel"), 10,
      [this](const geometry_msgs::msg::Twist & msg) {
        command_ = {msg.linear.x, msg.linear.y, msg.angular.z};
        command_time_ = SteadyClock::now();
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      declare_parameter<std::string>("imu_topic", "/imu/data"), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu & msg) {on_imu(msg);});
    target_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      declare_parameter<std::string>("joint_target_topic", "/policy/joint_targets"), 10,
      [this](const sensor_msgs::msg::JointState & msg) {on_targets(msg);});
    button_sub_ = create_subscription<std_msgs::msg::String>(
      declare_parameter<std::string>("button_topic", "/gamepad/button_event"), 10,
      [this](const std_msgs::msg::String & msg) {on_button(msg.data);});
    observation_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      declare_parameter<std::string>("observation_topic", "/policy/observation"), 10);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate)),
      [this]() {on_tick();});

    RCLCPP_INFO(
      get_logger(), "Bridging %zu motors. Disarmed; '%s' arms, '%s' disarms.",
      motors_.size(), arm_event_.c_str(), disarm_event_.c_str());
  }

private:
  struct Motor
  {
    size_t joint{};  // index into joint_names_
    tb::JointMotor calibration{};
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr command;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub;
    double position{};  // joint space
    double velocity{};
    SteadyClock::time_point stamp{};
  };

  void on_imu(const sensor_msgs::msg::Imu & msg)
  {
    try {
      const auto gravity = tb::projected_gravity(
        {msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z});
      const std::array<double, 3> angular{
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z};
      if (!std::all_of(angular.begin(), angular.end(), [](double v) {return std::isfinite(v);})) {
        throw std::invalid_argument("IMU angular velocity is non-finite");
      }
      gravity_ = gravity;
      angular_velocity_ = angular;
      imu_time_ = SteadyClock::now();
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Ignoring IMU: %s", error.what());
    }
  }

  void on_targets(const sensor_msgs::msg::JointState & msg)
  {
    if (msg.name != joint_names_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Ignoring targets with a different joint order");
      return;
    }
    try {
      previous_actions_ = tb::actions_from_targets(msg.position, defaults_, action_scale_);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Ignoring targets: %s", error.what());
      return;
    }
    target_time_ = SteadyClock::now();
    if (!armed_) {
      return;
    }
    for (const auto & motor : motors_) {
      std_msgs::msg::Float64 command;
      command.data = tb::joint_to_motor(motor.calibration, msg.position[motor.joint]);
      motor.command->publish(command);
    }
  }

  // Why the robot is not ready to run, or nullptr when it is.
  const char * not_ready() const
  {
    for (const auto & motor : motors_) {
      if (!fresh(motor.stamp, feedback_timeout_)) {
        return "motor feedback is stale";
      }
    }
    if (require_imu_ && !fresh(imu_time_, imu_timeout_)) {
      return "IMU data is stale";
    }
    return nullptr;
  }

  void on_button(const std::string & event)
  {
    if (event == disarm_event_) {
      disarm("disarm button");
    } else if (event == arm_event_ && !armed_) {
      const char * reason = not_ready();
      if (reason == nullptr && !fresh(target_time_, target_timeout_)) {
        reason = "the policy is not publishing targets";
      }
      if (reason != nullptr) {
        RCLCPP_WARN(get_logger(), "Not arming: %s", reason);
        return;
      }
      armed_ = true;
      RCLCPP_WARN(get_logger(), "ARMED: forwarding policy targets to %zu motors", motors_.size());
    }
  }

  void disarm(const char * reason)
  {
    if (armed_) {
      armed_ = false;
      RCLCPP_WARN(get_logger(), "Disarmed (%s); motors hold their last target", reason);
    }
  }

  void on_tick()
  {
    if (armed_ && !fresh(target_time_, target_timeout_)) {
      disarm("policy targets timed out");
    }
    if (const char * reason = not_ready()) {
      disarm(reason);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting: %s", reason);
      return;
    }

    tb::ObservationInputs in;
    // The gamepad only publishes while input changes or is held, so a stale command means stop.
    in.command = fresh(command_time_, cmd_vel_timeout_) ? command_ : std::array<double, 3>{};
    in.angular_velocity = require_imu_ ? angular_velocity_ : std::array<double, 3>{};
    in.gravity = require_imu_ ? gravity_ : std::array<double, 3>{0.0, 0.0, -1.0};
    in.joint_positions = defaults_;
    in.joint_velocities.assign(joint_names_.size(), 0.0);
    for (const auto & motor : motors_) {
      in.joint_positions[motor.joint] = motor.position;
      in.joint_velocities[motor.joint] = motor.velocity;
    }
    in.previous_actions = previous_actions_;

    std_msgs::msg::Float32MultiArray msg;
    try {
      msg.data = tb::build_observation(in, defaults_);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "%s", error.what());
      return;
    }
    observation_pub_->publish(msg);
  }

  std::vector<std::string> joint_names_;
  std::vector<double> defaults_;
  double action_scale_{};
  std::vector<Motor> motors_;
  double cmd_vel_timeout_{};
  double imu_timeout_{};
  double feedback_timeout_{};
  double target_timeout_{};
  bool require_imu_{true};
  std::string arm_event_;
  std::string disarm_event_;
  bool armed_{false};

  std::array<double, 3> command_{};
  SteadyClock::time_point command_time_{};
  std::array<double, 3> angular_velocity_{};
  std::array<double, 3> gravity_{};
  SteadyClock::time_point imu_time_{};
  std::vector<double> previous_actions_;
  SteadyClock::time_point target_time_{};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr button_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr observation_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PolicyBridge>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("policy_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
