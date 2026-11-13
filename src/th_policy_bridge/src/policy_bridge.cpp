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
#include "geometry_msgs/msg/vector3.hpp"
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
//
// With walk_event set, arming is two-step: arm_event ramps every joint from
// where it is to the default pose and holds it (STAND), then walk_event hands
// control to the policy (WALK). Without it, arm_event goes straight to WALK.
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
    const double rate = declare_parameter<double>("rate", 50.0);
    layout_ = tb::parse_layout(
      declare_parameter<std::vector<std::string>>("observation_terms", std::vector<std::string>{}),
      declare_parameter<std::vector<double>>("observation_scales", std::vector<double>{}));
    gait_period_ = declare_parameter<double>("gait_period", 0.0);
    const auto imu_rotation = declare_parameter<std::vector<double>>(
      "imu_rotation", std::vector<double>{1.0, 0.0, 0.0, 0.0});
    const auto imu_world_frame = declare_parameter<std::string>("imu_world_frame", "enu");
    max_tilt_ = declare_parameter<double>("max_tilt", 1.0);
    walk_event_ = declare_parameter<std::string>("walk_event", "");
    stand_ramp_time_ = declare_parameter<double>("stand_ramp_time", 2.0);
    if (imu_rotation.size() != 4 || (imu_world_frame != "enu" && imu_world_frame != "ned") ||
      !(gait_period_ >= 0.0) || !(stand_ramp_time_ > 0.0) || !(max_tilt_ >= 0.0))
    {
      throw std::invalid_argument(
              "imu_rotation is [w, x, y, z], imu_world_frame is enu or ned, gait_period >= 0, "
              "stand_ramp_time > 0, max_tilt >= 0 (0 disables)");
    }
    imu_to_base_ = {imu_rotation[0], imu_rotation[1], imu_rotation[2], imu_rotation[3]};
    tb::rotate(imu_to_base_, {0.0, 0.0, 1.0});  // validates the quaternion
    world_z_down_ = imu_world_frame == "ned";
    if (std::find(layout_.terms.begin(), layout_.terms.end(), tb::Term::kGaitPhase) !=
      layout_.terms.end() && gait_period_ <= 0.0)
    {
      throw std::invalid_argument("gait_phase observation needs gait_period > 0");
    }
    cmd_vel_timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.5);
    imu_timeout_ = declare_parameter<double>("imu_timeout", 0.1);
    feedback_timeout_ = declare_parameter<double>("feedback_timeout", 0.1);
    // Once armed, a short sensor gap does not disarm: MIT motors keep tracking their last
    // target and the policy runs on the last readings. Disarming mid-walk drops the robot,
    // so only an outage longer than this does.
    armed_dropout_timeout_ = declare_parameter<double>("armed_dropout_timeout", 0.3);
    target_timeout_ = declare_parameter<double>("target_timeout", 0.2);
    require_imu_ = declare_parameter<bool>("require_imu", true);
    arm_event_ = declare_parameter<std::string>("arm_event", "START pressed");
    disarm_event_ = declare_parameter<std::string>("disarm_event", "BACK pressed");

    if (joint_names_.empty() || joint_names_.size() != defaults_.size() ||
      !std::all_of(defaults_.begin(), defaults_.end(), [](double v) {return std::isfinite(v);}) ||
      !std::isfinite(action_scale_) || action_scale_ == 0.0 ||
      !std::isfinite(rate) || rate <= 0.0 || !(cmd_vel_timeout_ > 0.0) ||
      !(imu_timeout_ > 0.0) || !(feedback_timeout_ > 0.0) || !(target_timeout_ > 0.0) ||
      !(armed_dropout_timeout_ >= feedback_timeout_) || !(armed_dropout_timeout_ >= imu_timeout_))
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
    gravity_pub_ = create_publisher<geometry_msgs::msg::Vector3>("~/projected_gravity", 10);
    mode_pub_ = create_publisher<std_msgs::msg::String>("~/mode", rclcpp::QoS(1).transient_local());
    publish_mode();
    observation_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      declare_parameter<std::string>("observation_topic", "/policy/observation"), 10);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 /
      rate)),
      [this]() {on_tick();});

    RCLCPP_INFO(
      get_logger(), "Bridging %zu motors at %.0f Hz, observation size %zu. Disarmed; '%s' %s, "
      "'%s' disarms.", motors_.size(), rate, layout_.size(joint_names_.size()),
      arm_event_.c_str(), walk_event_.empty() ? "arms" : "stands", disarm_event_.c_str());
    if (!walk_event_.empty()) {
      RCLCPP_INFO(get_logger(), "'%s' starts walking once standing.", walk_event_.c_str());
    }
  }

private:
  enum class Mode { kDisarmed, kStand, kWalk };

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
      // Both vectors arrive in the IMU frame; imu_to_base_ rotates them into the base frame.
      const auto gravity = tb::rotate(imu_to_base_, tb::projected_gravity(
            {msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z},
          world_z_down_));
      const auto angular = tb::rotate(imu_to_base_, {
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z});
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
    std::vector<double> actions;
    try {
      actions = tb::actions_from_targets(msg.position, defaults_, action_scale_);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Ignoring targets: %s", error.what());
      return;
    }
    target_time_ = SteadyClock::now();
    // While standing the ramp, not the policy, drives the joints, so the last action stays
    // zero and WALK starts from a zero last action, as a training episode does.
    if (mode_ != Mode::kStand) {
      previous_actions_ = std::move(actions);
    }
    if (mode_ != Mode::kWalk) {
      return;
    }
    send_to_motors(msg.position);
  }

  // During the STAND ramp the limits are widened to include where the joint started, so a
  // joint found outside its limits (bench, or a calibration error) eases back in instead of
  // jumping to the limit. The ramp only moves toward the default pose, never further out.
  void send_to_motors(const std::vector<double> & joint_targets, bool from_ramp_start = false)
  {
    for (const auto & motor : motors_) {
      tb::JointMotor calibration = motor.calibration;
      if (from_ramp_start) {
        calibration.lower = std::min(calibration.lower, ramp_start_[motor.joint]);
        calibration.upper = std::max(calibration.upper, ramp_start_[motor.joint]);
      }
      std_msgs::msg::Float64 command;
      command.data = tb::joint_to_motor(calibration, joint_targets[motor.joint]);
      motor.command->publish(command);
    }
  }

  std::vector<double> measured_joints() const
  {
    std::vector<double> q = defaults_;
    for (const auto & motor : motors_) {q[motor.joint] = motor.position;}
    return q;
  }

  void set_mode(Mode mode)
  {
    mode_ = mode;
    publish_mode();
  }

  void publish_mode()
  {
    std_msgs::msg::String msg;
    msg.data = mode_ == Mode::kDisarmed ? "DISARMED" : mode_ == Mode::kStand ? "STAND" : "WALK";
    mode_pub_->publish(msg);
  }

  // Why the robot is not ready to run, or nullptr when it is. While armed, gaps up to
  // armed_dropout_timeout are tolerated (and reported through *gap).
  const char * not_ready(bool armed = false, const char ** gap = nullptr) const
  {
    const auto check = [&](const SteadyClock::time_point & stamp, double strict,
      const char * stale, const char * late) -> const char * {
        if (fresh(stamp, strict)) {
          return nullptr;
        }
        if (armed && fresh(stamp, armed_dropout_timeout_)) {
          if (gap != nullptr) {*gap = late;}
          return nullptr;
        }
        return stale;
      };
    for (const auto & motor : motors_) {
      if (const char * r = check(motor.stamp, feedback_timeout_, "motor feedback is stale",
          "motor feedback is late"))
      {
        return r;
      }
    }
    if (require_imu_) {
      return check(imu_time_, imu_timeout_, "IMU data is stale", "IMU data is late");
    }
    return nullptr;
  }

  void on_button(const std::string & event)
  {
    if (event == disarm_event_) {
      disarm("disarm button");
    } else if (event == arm_event_ && mode_ == Mode::kDisarmed) {
      const char * reason = not_ready();
      if (reason == nullptr && walk_event_.empty() && !fresh(target_time_, target_timeout_)) {
        reason = "the policy is not publishing targets";
      }
      if (reason == nullptr && tilted()) {
        reason = "the base is tilted past max_tilt";
      }
      if (reason != nullptr) {
        RCLCPP_WARN(get_logger(), "Not arming: %s", reason);
        return;
      }
      if (walk_event_.empty()) {
        start_walking();
      } else {
        ramp_start_ = measured_joints();
        ramp_time_ = SteadyClock::now();
        previous_actions_.assign(joint_names_.size(), 0.0);
        set_mode(Mode::kStand);
        RCLCPP_WARN(
          get_logger(), "STAND: ramping %zu motors to the default pose over %.1f s; '%s' walks",
          motors_.size(), stand_ramp_time_, walk_event_.c_str());
      }
    } else if (!walk_event_.empty() && event == walk_event_ && mode_ == Mode::kStand) {
      if (!ramp_done()) {
        RCLCPP_WARN(get_logger(), "Not walking yet: still ramping to the default pose");
      } else if (!fresh(target_time_, target_timeout_)) {
        RCLCPP_WARN(get_logger(), "Not walking: the policy is not publishing targets");
      } else {
        start_walking();
      }
    }
  }

  void start_walking()
  {
    walk_time_ = SteadyClock::now();
    set_mode(Mode::kWalk);
    RCLCPP_WARN(get_logger(), "WALK: forwarding policy targets to %zu motors", motors_.size());
  }

  bool ramp_done() const
  {
    return std::chrono::duration<double>(SteadyClock::now() - ramp_time_).count() >=
           stand_ramp_time_;
  }

  // Angle between the base z axis and vertical exceeds max_tilt (only with an IMU).
  bool tilted() const
  {
    return require_imu_ && max_tilt_ > 0.0 &&
           std::acos(std::clamp(-gravity_[2], -1.0, 1.0)) > max_tilt_;
  }

  void disarm(const char * reason)
  {
    if (mode_ != Mode::kDisarmed) {
      set_mode(Mode::kDisarmed);
      RCLCPP_WARN(
        get_logger(), "Disarmed (%s); targets stop (CSP motors hold, MIT motors damp)", reason);
    }
  }

  void on_tick()
  {
    if (mode_ == Mode::kWalk && !fresh(target_time_, target_timeout_)) {
      disarm("policy targets timed out");
    }
    if (mode_ != Mode::kDisarmed && tilted()) {
      disarm("base tilt exceeded max_tilt (fall)");
    }
    const char * gap = nullptr;
    if (const char * reason = not_ready(mode_ != Mode::kDisarmed, &gap)) {
      disarm(reason);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting: %s", reason);
      return;
    }

    if (gap != nullptr) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "%s; running on the last readings", gap);
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
    in.gait_phase = 0.0;
    if (mode_ == Mode::kWalk && gait_period_ > 0.0) {
      const double t = std::chrono::duration<double>(SteadyClock::now() - walk_time_).count();
      in.gait_phase = std::fmod(t / gait_period_, 1.0);
    }

    if (mode_ == Mode::kStand) {
      const double alpha =
        std::chrono::duration<double>(SteadyClock::now() - ramp_time_).count() / stand_ramp_time_;
      send_to_motors(tb::interpolate(ramp_start_, defaults_, alpha), true);
    }
    geometry_msgs::msg::Vector3 g;
    g.x = in.gravity[0];
    g.y = in.gravity[1];
    g.z = in.gravity[2];
    gravity_pub_->publish(g);

    std_msgs::msg::Float32MultiArray msg;
    try {
      msg.data = tb::build_observation(in, defaults_, layout_);
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
  double armed_dropout_timeout_{};
  double target_timeout_{};
  bool require_imu_{true};
  std::string arm_event_;
  std::string disarm_event_;
  std::string walk_event_;
  Mode mode_{Mode::kDisarmed};
  tb::ObservationLayout layout_;
  double gait_period_{};
  tb::Quaternion imu_to_base_{1.0, 0.0, 0.0, 0.0};
  bool world_z_down_{false};
  double max_tilt_{};
  double stand_ramp_time_{};
  std::vector<double> ramp_start_;
  SteadyClock::time_point ramp_time_{};
  SteadyClock::time_point walk_time_{};

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
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gravity_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
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
