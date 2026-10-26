#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
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

// Assembles policy observations from /cmd_vel, the IMU, joint feedback and the
// policy's own previous targets, and publishes them at the policy rate.
class ObservationBuilder : public rclcpp::Node
{
public:
  ObservationBuilder()
  : Node("observation_builder")
  {
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", std::vector<std::string>{});
    defaults_ = declare_parameter<std::vector<double>>(
      "default_joint_positions", std::vector<double>{});
    action_scale_ = declare_parameter<double>("action_scale", 0.25);
    const double rate = declare_parameter<double>("rate", 25.0);
    cmd_vel_timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.5);
    imu_timeout_ = declare_parameter<double>("imu_timeout", 0.1);
    joint_state_timeout_ = declare_parameter<double>("joint_state_timeout", 0.1);
    require_imu_ = declare_parameter<bool>("require_imu", true);
    require_all_joints_ = declare_parameter<bool>("require_all_joints", true);

    if (joint_names_.empty() || joint_names_.size() != defaults_.size() ||
      !std::all_of(defaults_.begin(), defaults_.end(), [](double v) {return std::isfinite(v);}) ||
      !std::isfinite(action_scale_) || action_scale_ == 0.0 ||
      !std::isfinite(rate) || rate <= 0.0 ||
      !(cmd_vel_timeout_ > 0.0) || !(imu_timeout_ > 0.0) || !(joint_state_timeout_ > 0.0))
    {
      throw std::invalid_argument("Invalid observation_builder parameters");
    }
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (!index_.emplace(joint_names_[i], i).second) {
        throw std::invalid_argument("Duplicate joint name " + joint_names_[i]);
      }
    }
    joints_.resize(joint_names_.size());
    previous_actions_.assign(joint_names_.size(), 0.0);

    if (!require_imu_) {
      RCLCPP_WARN(get_logger(), "require_imu is false: assuming upright and not rotating");
    }
    if (!require_all_joints_) {
      RCLCPP_WARN(
        get_logger(), "require_all_joints is false: joints without feedback read as default pose");
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
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      declare_parameter<std::string>("joint_state_topic", "/joint_states"), 10,
      [this](const sensor_msgs::msg::JointState & msg) {on_joint_state(msg);});
    target_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      declare_parameter<std::string>("joint_target_topic", "/policy/joint_targets"), 10,
      [this](const sensor_msgs::msg::JointState & msg) {on_targets(msg);});
    publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      declare_parameter<std::string>("observation_topic", "/policy/observation"), 10);

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate)),
      [this]() {publish_observation();});
  }

private:
  struct JointFeedback
  {
    double position{};
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

  void on_joint_state(const sensor_msgs::msg::JointState & msg)
  {
    if (msg.position.size() != msg.name.size() ||
      (!msg.velocity.empty() && msg.velocity.size() != msg.name.size()))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Ignoring malformed joint state");
      return;
    }
    const auto now = SteadyClock::now();
    for (size_t i = 0; i < msg.name.size(); ++i) {
      const auto it = index_.find(msg.name[i]);
      const double velocity = msg.velocity.empty() ? 0.0 : msg.velocity[i];
      if (it == index_.end() || !std::isfinite(msg.position[i]) || !std::isfinite(velocity)) {
        continue;
      }
      joints_[it->second] = {msg.position[i], velocity, now};
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
    }
  }

  void publish_observation()
  {
    tb::ObservationInputs in;
    // The gamepad only publishes while input changes or is held, so a stale command means stop.
    in.command = fresh(command_time_, cmd_vel_timeout_) ? command_ : std::array<double, 3>{};
    if (fresh(imu_time_, imu_timeout_)) {
      in.angular_velocity = angular_velocity_;
      in.gravity = gravity_;
    } else if (!require_imu_) {
      in.angular_velocity = {};
      in.gravity = {0.0, 0.0, -1.0};
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for fresh IMU data");
      return;
    }
    in.joint_positions.reserve(joints_.size());
    in.joint_velocities.reserve(joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (fresh(joints_[i].stamp, joint_state_timeout_)) {
        in.joint_positions.push_back(joints_[i].position);
        in.joint_velocities.push_back(joints_[i].velocity);
      } else if (!require_all_joints_) {
        in.joint_positions.push_back(defaults_[i]);
        in.joint_velocities.push_back(0.0);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Waiting for fresh feedback from %s",
          joint_names_[i].c_str());
        return;
      }
    }
    in.previous_actions = previous_actions_;

    std_msgs::msg::Float32MultiArray msg;
    try {
      msg.data = tb::build_observation(in, defaults_);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "%s", error.what());
      return;
    }
    publisher_->publish(msg);
  }

  std::vector<std::string> joint_names_;
  std::vector<double> defaults_;
  std::unordered_map<std::string, size_t> index_;
  double action_scale_{};
  double cmd_vel_timeout_{};
  double imu_timeout_{};
  double joint_state_timeout_{};
  bool require_imu_{true};
  bool require_all_joints_{true};

  std::array<double, 3> command_{};
  SteadyClock::time_point command_time_{};
  std::array<double, 3> angular_velocity_{};
  std::array<double, 3> gravity_{};
  SteadyClock::time_point imu_time_{};
  std::vector<JointFeedback> joints_;
  std::vector<double> previous_actions_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ObservationBuilder>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("observation_builder"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
