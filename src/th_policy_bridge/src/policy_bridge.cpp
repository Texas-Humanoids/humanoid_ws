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

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "th_policy_bridge/heading_hold.hpp"
#include "th_policy_bridge/joint_mapping.hpp"
#include "th_policy_bridge/observation.hpp"
#include "th_policy_bridge/safety.hpp"
#include "th_policy_bridge/stand_balance.hpp"

namespace tb = th_policy_bridge;
using SteadyClock = std::chrono::steady_clock;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

namespace
{

bool fresh(const SteadyClock::time_point & stamp, double timeout)
{
  return stamp != SteadyClock::time_point{} &&
         std::chrono::duration<double>(SteadyClock::now() - stamp).count() <= timeout;
}

double seconds(const SteadyClock::time_point & t)
{
  return std::chrono::duration<double>(t.time_since_epoch()).count();
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
    // Per-motor MIT gains (same order as motor_joints): the trained gains used while
    // walking and disarmed, and the gains held during STAND. STAND can be stiffer, so the
    // robot holds itself up before the policy takes over. Empty = gains are not managed.
    const auto policy_kp = declare_parameter<std::vector<double>>(
      "policy_kp", std::vector<double>{});
    const auto policy_kd = declare_parameter<std::vector<double>>(
      "policy_kd", std::vector<double>{});
    const auto stand_kp = declare_parameter<std::vector<double>>(
      "stand_kp", std::vector<double>{});
    const auto stand_kd = declare_parameter<std::vector<double>>(
      "stand_kd", std::vector<double>{});
    layout_ = tb::parse_layout(
      declare_parameter<std::vector<std::string>>("observation_terms", std::vector<std::string>{}),
      declare_parameter<std::vector<double>>("observation_scales", std::vector<double>{}));
    gait_period_ = declare_parameter<double>("gait_period", 0.0);
    // "free": phase = (t since WALK) mod period (v1 policies). "held": the clock stops under a
    // stand command (HeldGaitClock); must match how the policy was trained (policy_spec.json).
    const auto gait_clock = declare_parameter<std::string>("gait_clock", "free");
    gait_stand_threshold_ = declare_parameter<double>("gait_stand_threshold", 0.1);
    if (gait_clock != "free" && gait_clock != "held") {
      throw std::invalid_argument("gait_clock is free or held");
    }
    held_gait_clock_ = gait_clock == "held";
    policy_dt_ = 1.0 / rate;
    const auto imu_rotation = declare_parameter<std::vector<double>>(
      "imu_rotation", std::vector<double>{1.0, 0.0, 0.0, 0.0});
    const auto imu_world_frame = declare_parameter<std::string>("imu_world_frame", "enu");
    max_tilt_ = declare_parameter<double>("max_tilt", 1.0);
    walk_event_ = declare_parameter<std::string>("walk_event", "");
    stand_ramp_time_ = declare_parameter<double>("stand_ramp_time", 2.0);
    // Heading hold while walking without turning (see heading_hold.hpp); gain 0 disables it.
    heading_hold_ = tb::HeadingHold(
      declare_parameter<double>("heading_hold_gain", 0.0),
      declare_parameter<double>("heading_hold_max_rate", 0.3));
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
    // Every motor must report OK on <ns>/motor/status (robstride_motor_node: ERROR = a drive
    // fault, WARN = the drive left run mode). No status for motor_status_timeout (no status
    // frames: CAN bus down, motor unpowered or unplugged, motor node gone) also counts; while
    // armed, like feedback, only after armed_dropout_timeout. Any of these blocks arming and
    // disarms while armed. Off for sim, whose motors publish no status.
    require_motor_status_ = declare_parameter<bool>("require_motor_status", true);
    motor_status_timeout_ = declare_parameter<double>("motor_status_timeout", 0.25);
    // While walking, two freezes disarm: a motor whose feedback stays bit-identical for
    // freeze_timeout while its target moves by more than freeze_min_target_change (stale or
    // stuck feedback), and policy targets that stay bit-identical for freeze_timeout (a stuck
    // policy; the gait clock makes a live policy's output change every step). 0 disables both.
    freeze_timeout_ = declare_parameter<double>("freeze_timeout", 0.5);
    freeze_min_target_change_ = declare_parameter<double>("freeze_min_target_change", 0.1);
    // A joint faster than its limit for joint_velocity_hold disarms (STAND and WALK): a policy
    // out of its depth (e.g. thrashing in the air) or a runaway joint. Per motor in
    // max_joint_velocities (motor_joints order, rad/s); 0 there = max_joint_velocity; 0 = off.
    const auto max_velocities = declare_parameter<std::vector<double>>(
      "max_joint_velocities", std::vector<double>{});
    const double max_velocity = declare_parameter<double>("max_joint_velocity", 0.0);
    joint_velocity_hold_ = declare_parameter<double>("joint_velocity_hold", 0.05);
    if (!(max_velocity >= 0.0) || !(joint_velocity_hold_ >= 0.0)) {
      throw std::invalid_argument("max_joint_velocity >= 0, joint_velocity_hold >= 0");
    }
    arm_event_ = declare_parameter<std::string>("arm_event", "START pressed");
    disarm_event_ = declare_parameter<std::string>("disarm_event", "BACK pressed");
    // More buttons that disarm, on top of disarm_event (e.g. "B pressed": easier to find than
    // the small View/BACK button in a hurry).
    disarm_events_ = declare_parameter<std::vector<std::string>>(
      "disarm_events", std::vector<std::string>{});
    if (std::find(disarm_events_.begin(), disarm_events_.end(), disarm_event_) ==
      disarm_events_.end())
    {
      disarm_events_.push_back(disarm_event_);
    }
    if (!(motor_status_timeout_ > 0.0) || !(freeze_timeout_ >= 0.0) ||
      !(freeze_min_target_change_ > 0.0))
    {
      throw std::invalid_argument(
              "motor_status_timeout > 0, freeze_timeout >= 0 (0 disables), "
              "freeze_min_target_change > 0");
    }

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
      motor.name = namespaces[m];
      motor.joint = it->second;
      motor.calibration = {directions[m], offsets[m], lowers[m], uppers[m]};
      tb::validate(motor.calibration, motor_joints[m]);
      motor.command = create_publisher<std_msgs::msg::Float64>(
        namespaces[m] + "/motor/target_position", 10);
      motor.gains = create_publisher<std_msgs::msg::Float64MultiArray>(
        namespaces[m] + "/motor/gains", 10);
      motor.position_sub = create_subscription<std_msgs::msg::Float64>(
        namespaces[m] + "/motor/actual_position", 10,
        [this, m](const std_msgs::msg::Float64 & msg) {
          if (std::isfinite(msg.data)) {
            auto & motor = motors_[m];
            motor.position = tb::motor_to_joint(motor.calibration, msg.data);
            motor.stamp = SteadyClock::now();
            motor.freeze.update(msg.data, seconds(motor.stamp));
          }
        });
      motor.status_sub = create_subscription<DiagnosticStatus>(
        namespaces[m] + "/motor/status", 10,
        [this, m](const DiagnosticStatus & msg) {
          motors_[m].status_level = msg.level;
          motors_[m].status_message = msg.message;
          motors_[m].status_stamp = SteadyClock::now();
        });
      motor.velocity_sub = create_subscription<std_msgs::msg::Float64>(
        namespaces[m] + "/motor/actual_velocity", 10,
        [this, m](const std_msgs::msg::Float64 & msg) {
          if (std::isfinite(msg.data)) {
            auto & motor = motors_[m];
            motor.velocity = motor.calibration.direction * msg.data;
            motor.speed.update(motor.velocity, motor.max_velocity, seconds(SteadyClock::now()));
          }
        });
      motors_.push_back(std::move(motor));
    }
    previous_actions_.assign(joint_names_.size(), 0.0);
    if (!max_velocities.empty() && max_velocities.size() != n) {
      throw std::invalid_argument("max_joint_velocities needs one entry per motor (or none)");
    }
    std::string speed_limits;
    for (size_t m = 0; m < n; ++m) {
      const double v = max_velocities.empty() ? 0.0 : max_velocities[m];
      if (!std::isfinite(v) || v < 0.0) {
        throw std::invalid_argument("max_joint_velocities must be finite and >= 0");
      }
      motors_[m].max_velocity = v > 0.0 ? v : max_velocity;
      if (motors_[m].max_velocity > 0.0) {
        speed_limits += (speed_limits.empty() ? "" : ", ") + motor_joints[m] + " " +
          std::to_string(motors_[m].max_velocity).substr(0, 4);
      }
    }
    if (!speed_limits.empty()) {
      RCLCPP_INFO(
        get_logger(), "Joint speed trip (rad/s, %.0f ms hold): %s", joint_velocity_hold_ * 1000.0,
        speed_limits.c_str());
    }

    manage_gains_ = !policy_kp.empty();
    if (manage_gains_) {
      if (policy_kd.size() != n || policy_kp.size() != n || stand_kp.size() != n ||
        stand_kd.size() != n)
      {
        throw std::invalid_argument(
                "policy_kp, policy_kd, stand_kp and stand_kd need one entry per motor");
      }
      for (size_t m = 0; m < n; ++m) {
        for (double g : {policy_kp[m], policy_kd[m], stand_kp[m], stand_kd[m]}) {
          if (!std::isfinite(g) || g < 0.0) {
            throw std::invalid_argument("Gains must be finite and >= 0");
          }
        }
        motors_[m].policy_gains = {policy_kp[m], policy_kd[m]};
        motors_[m].stand_gains = {stand_kp[m], stand_kd[m]};
        if (stand_kp[m] != policy_kp[m] || stand_kd[m] != policy_kd[m]) {
          RCLCPP_INFO(
            get_logger(), "%s: STAND gains kp %.1f kd %.2f, policy gains kp %.1f kd %.2f",
            motor_joints[m].c_str(), stand_kp[m], stand_kd[m], policy_kp[m], policy_kd[m]);
        }
      }
    }

    if (!require_imu_) {
      RCLCPP_WARN(get_logger(), "require_imu is false: assuming upright and not rotating");
    }
    if (!require_motor_status_) {
      RCLCPP_WARN(get_logger(), "require_motor_status is false: motor faults will NOT disarm");
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
    // Measured joint angles in joint space (calibration applied), for robot_state_publisher
    // and the dashboard. Joints without a motor read as the default pose, as in the policy.
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      declare_parameter<std::string>("joint_state_topic", "/joint_states"), 10);
    // STAND-mode COM centering (stand_balance.hpp); stand_balance_gain 0 = off (the default).
    stand_trim_.gain = declare_parameter<double>("stand_balance_gain", 0.0);
    stand_trim_.max_lean = declare_parameter<double>("stand_balance_max_lean", 0.06);
    stand_trim_.filter_time = declare_parameter<double>("stand_balance_filter_time", 1.0);
    stand_trim_.freeze_above = declare_parameter<double>("stand_balance_freeze_above", 2.5);
    // Mean ankle torque the trim holds (N m, + = weight ahead of the ankles); 0 = COM over the
    // ankles. The real robot balanced at +1.4..+2.0 N m (auto-lean: 1.5).
    stand_trim_.target_torque = declare_parameter<double>("stand_balance_target_torque", 0.0);
    if (!std::isfinite(stand_trim_.target_torque) || std::abs(stand_trim_.target_torque) > 5.0) {
      throw std::invalid_argument("stand_balance_target_torque must be within +-5 N m");
    }
    const auto balance_joints = declare_parameter<std::vector<std::string>>(
      "stand_balance_joints",
      std::vector<std::string>{"left_ankle_pitch_joint", "right_ankle_pitch_joint"});
    if (!(stand_trim_.gain >= 0.0) || !(stand_trim_.max_lean >= 0.0) ||
      !(stand_trim_.max_lean <= 0.15) || !(stand_trim_.filter_time > 0.0) ||
      !(stand_trim_.freeze_above > 0.0))
    {
      throw std::invalid_argument(
              "stand_balance: gain >= 0, 0 <= max_lean <= 0.15 rad, filter_time > 0, freeze_above > 0");
    }
    if (balance_joints.size() != 2) {
      throw std::invalid_argument("stand_balance_joints is [left ankle, right ankle]");
    }
    std::string missing_ankle;
    for (size_t side = 0; side < 2; ++side) {
      bool found = false;
      for (size_t m = 0; m < motors_.size(); ++m) {
        if (joint_names_[motors_[m].joint] == balance_joints[side]) {
          balance_motor_[side] = m;
          found = true;
        }
      }
      if (!found) {missing_ankle = balance_joints[side];}
    }
    have_ankles_ = missing_ankle.empty();
    // Operator lean in STAND (D-pad, stand_balance.hpp ManualLean), starting at stand_lean.
    manual_lean_.target = declare_parameter<double>("stand_lean", 0.0);
    manual_lean_.step = declare_parameter<double>("stand_lean_step", 0.01);
    manual_lean_.max = declare_parameter<double>("stand_lean_max", 0.25);
    manual_lean_.rate = declare_parameter<double>("stand_lean_rate", 0.05);
    lean_forward_event_ = declare_parameter<std::string>("stand_lean_forward_event", "DPAD_UP pressed");
    lean_back_event_ = declare_parameter<std::string>("stand_lean_back_event", "DPAD_DOWN pressed");
    if (!(manual_lean_.step > 0.0 && manual_lean_.step <= 0.05) ||
      !(manual_lean_.max >= 0.0 && manual_lean_.max <= 0.3) || !(manual_lean_.rate > 0.0) ||
      !(std::abs(manual_lean_.target) <= manual_lean_.max))
    {
      throw std::invalid_argument(
              "stand_lean: 0 < step <= 0.05, 0 <= max <= 0.3, rate > 0, |stand_lean| <= max (rad)");
    }
    lean_pub_ = create_publisher<std_msgs::msg::Float64>("~/stand_lean", 10);
    if (have_ankles_) {
      RCLCPP_INFO(
        get_logger(), "STAND lean: %+.3f rad (+ = back); '%s' leans forward, '%s' back, %.3f rad "
        "per press, max %.2f", manual_lean_.target, lean_forward_event_.c_str(),
        lean_back_event_.c_str(), manual_lean_.step, manual_lean_.max);
    }
    if (stand_trim_.gain > 0.0) {
      if (!manage_gains_) {
        throw std::invalid_argument("stand_balance needs the stand gains (control_mode mit + policy_spec)");
      }
      if (!have_ankles_) {
        throw std::invalid_argument("stand_balance joint without a motor: " + missing_ankle);
      }
      RCLCPP_INFO(
        get_logger(), "STAND balance on: gain %.4f rad/s/Nm, max lean %.3f rad, filter %.1f s, "
        "frozen above %.1f Nm, holds mean ankle torque %+.2f Nm", stand_trim_.gain, stand_trim_.max_lean,
        stand_trim_.filter_time, stand_trim_.freeze_above, stand_trim_.target_torque);
    }
    stand_balance_ = stand_trim_.gain > 0.0;
    // Optional: hold the trim until this button event (stand_balance.hpp TrimGate), e.g. "Y pressed"
    // once the robot stands on its own weight. Empty = trim runs right after the ramp.
    trim_arm_event_ = declare_parameter<std::string>("stand_balance_arm_event", "");
    trim_gate_.enabled = !trim_arm_event_.empty();
    if (trim_gate_.enabled &&
      (trim_arm_event_ == arm_event_ || trim_arm_event_ == walk_event_ ||
      trim_arm_event_ == lean_forward_event_ || trim_arm_event_ == lean_back_event_ ||
      std::find(disarm_events_.begin(), disarm_events_.end(), trim_arm_event_) !=
      disarm_events_.end()))
    {
      throw std::invalid_argument(
              "stand_balance_arm_event '" + trim_arm_event_ +
              "' is already the arm, walk, disarm or D-pad lean event");
    }
    if (trim_gate_.enabled && stand_balance_) {
      RCLCPP_INFO(
        get_logger(), "STAND balance gated: it starts only on '%s' (press once the robot carries "
        "its own weight)", trim_arm_event_.c_str());
    }
    // Scripted balanced squat in STAND (stand_balance.hpp SquatMotion), one cycle per press of
    // stand_squat_event (e.g. "X pressed"). Empty = off.
    squat_event_ = declare_parameter<std::string>("stand_squat_event", "");
    squat_.depth = declare_parameter<double>("stand_squat_depth", 0.02);
    squat_.down_time = declare_parameter<double>("stand_squat_down_time", 3.0);
    squat_.hold_time = declare_parameter<double>("stand_squat_hold_time", 1.0);
    if (!squat_event_.empty()) {
      if (!(squat_.depth > 0.0 && squat_.depth <= 0.05) || !(squat_.down_time >= 1.0) ||
        !(squat_.hold_time >= 0.0 && squat_.hold_time <= 30.0))
      {
        throw std::invalid_argument(
                "stand_squat: 0 < depth <= 0.05 m, down_time >= 1 s, 0 <= hold_time <= 30 s");
      }
      if (squat_event_ == arm_event_ || squat_event_ == walk_event_ ||
        squat_event_ == lean_forward_event_ || squat_event_ == lean_back_event_ ||
        squat_event_ == trim_arm_event_ ||
        std::find(disarm_events_.begin(), disarm_events_.end(), squat_event_) !=
        disarm_events_.end())
      {
        throw std::invalid_argument(
                "stand_squat_event '" + squat_event_ +
                "' is already the arm, walk, disarm, D-pad lean or auto-lean event");
      }
      static const char * kJoints[2][3] = {
        {"left_hip_pitch_joint", "left_knee_pitch_joint", "left_ankle_pitch_joint"},
        {"right_hip_pitch_joint", "right_knee_pitch_joint", "right_ankle_pitch_joint"}};
      for (size_t side = 0; side < 2; ++side) {
        for (size_t k = 0; k < 3; ++k) {
          bool found = false;
          for (size_t m = 0; m < motors_.size(); ++m) {
            if (joint_names_[motors_[m].joint] == kJoints[side][k]) {
              squat_motor_[side][k] = m;
              found = true;
            }
          }
          if (!found) {
            throw std::invalid_argument(
                    std::string("stand_squat needs a motor on ") + kJoints[side][k]);
          }
        }
      }
      RCLCPP_INFO(
        get_logger(), "STAND squat on '%s': %.1f cm, %.1f s down, %.1f s hold, %.1f s up",
        squat_event_.c_str(), squat_.depth * 100.0, squat_.down_time, squat_.hold_time,
        squat_.down_time);
    }

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 /
      rate)),
      [this]() {on_tick();});

    RCLCPP_INFO(
      get_logger(), "Bridging %zu motors at %.0f Hz, observation size %zu. Disarmed; '%s' %s, "
      "%s disarms.", motors_.size(), rate, layout_.size(joint_names_.size()),
      arm_event_.c_str(), walk_event_.empty() ? "arms" : "stands", [this]() {
        std::string names;
        for (const auto & e : disarm_events_) {names += (names.empty() ? "'" : " or '") + e + "'";}
        return names;
      }().c_str());
    if (!walk_event_.empty()) {
      RCLCPP_INFO(get_logger(), "'%s' starts walking once standing.", walk_event_.c_str());
    }
    if (heading_hold_.enabled()) {
      if (!require_imu_) {
        RCLCPP_WARN(get_logger(), "heading hold needs the IMU; it is off with require_imu false");
      } else {
        RCLCPP_INFO(get_logger(), "Heading hold on while walking without turning");
      }
    }
  }

private:
  enum class Mode { kDisarmed, kStand, kWalk };

  struct Motor
  {
    std::string name;  // its namespace, for messages
    size_t joint{};  // index into joint_names_
    tb::JointMotor calibration{};
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr command;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub;
    rclcpp::Subscription<DiagnosticStatus>::SharedPtr status_sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gains;
    std::array<double, 2> policy_gains{};
    std::array<double, 2> stand_gains{};
    double position{};  // joint space
    double velocity{};
    SteadyClock::time_point stamp{};
    uint8_t status_level{DiagnosticStatus::STALE};  // until the first status arrives
    std::string status_message;
    SteadyClock::time_point status_stamp{};
    double last_target{};  // joint space, as sent (after clamping)
    double max_velocity{};  // rad/s, 0 = no speed trip
    tb::OverLimitWatch speed;
    tb::FreezeWatch freeze;  // raw feedback vs. the targets sent
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
    target_freeze_.update(msg.position, seconds(target_time_));
    send_to_motors(msg.position);
  }

  // During the STAND ramp the limits are widened to include where the joint started, so a
  // joint found outside its limits (bench, or a calibration error) eases back in instead of
  // jumping to the limit. The ramp only moves toward the default pose, never further out.
  void send_to_motors(const std::vector<double> & joint_targets, bool from_ramp_start = false)
  {
    for (auto & motor : motors_) {
      tb::JointMotor calibration = motor.calibration;
      if (from_ramp_start) {
        calibration.lower = std::min(calibration.lower, ramp_start_[motor.joint]);
        calibration.upper = std::max(calibration.upper, ramp_start_[motor.joint]);
      }
      std_msgs::msg::Float64 command;
      command.data = tb::joint_to_motor(calibration, joint_targets[motor.joint]);
      motor.command->publish(command);
      // The clamped target, so a joint held at its limit is not mistaken for frozen feedback.
      motor.last_target = tb::motor_to_joint(calibration, command.data);
      motor.freeze.observe_ref(motor.last_target);
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
    publish_gains();
    publish_mode();
  }

  // STAND uses the stand gains; WALK and DISARMED use the policy's trained gains (when
  // disarmed the motors damp anyway, but the next WALK must start on the trained gains).
  void publish_gains()
  {
    if (!manage_gains_) {
      return;
    }
    for (const auto & motor : motors_) {
      const auto & g = mode_ == Mode::kStand ? motor.stand_gains : motor.policy_gains;
      std_msgs::msg::Float64MultiArray msg;
      msg.data = {g[0], g[1]};
      motor.gains->publish(msg);
    }
  }

  void publish_mode()
  {
    std_msgs::msg::String msg;
    msg.data = mode_ == Mode::kDisarmed ? "DISARMED" : mode_ == Mode::kStand ? "STAND" : "WALK";
    mode_pub_->publish(msg);
  }

  // Why the robot is not ready to run, or empty when it is. While armed, gaps up to
  // armed_dropout_timeout in feedback, IMU or motor status are tolerated (and reported
  // through *gap); a motor fault or a drive out of run mode fails at once.
  std::string not_ready(bool armed = false, const char ** gap = nullptr) const
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
    if (require_motor_status_) {
      for (const auto & motor : motors_) {
        if (check(motor.status_stamp, motor_status_timeout_, "stale", "motor status is late")) {
          return motor.name + ": no motor status (CAN bus down, motor off or unplugged, or "
                 "motor node stopped)";
        }
        if (motor.status_level == DiagnosticStatus::ERROR) {
          return motor.name + ": motor fault: " + motor.status_message;
        }
        if (motor.status_level != DiagnosticStatus::OK) {
          return motor.name + ": motor not running: " + motor.status_message;
        }
      }
    }
    if (require_imu_) {
      if (const char * r = check(imu_time_, imu_timeout_, "IMU data is stale", "IMU data is late")) {
        return r;
      }
    }
    return {};
  }

  void on_button(const std::string & event)
  {
    if (std::find(disarm_events_.begin(), disarm_events_.end(), event) != disarm_events_.end()) {
      disarm("disarm button: " + event);
    } else if (event == arm_event_ && mode_ == Mode::kDisarmed) {
      std::string reason = not_ready();
      if (reason.empty() && walk_event_.empty() && !fresh(target_time_, target_timeout_)) {
        reason = "the policy is not publishing targets";
      }
      if (reason.empty() && tilted()) {
        reason = "the base is tilted past max_tilt";
      }
      if (!reason.empty()) {
        RCLCPP_WARN(get_logger(), "Not arming: %s", reason.c_str());
        return;
      }
      if (walk_event_.empty()) {
        start_walking();
      } else {
        ramp_start_ = measured_joints();
        ramp_time_ = SteadyClock::now();
        stand_trim_.reset();
        trim_gate_.reset();
        squat_.stop();
        manual_lean_.snap();
        previous_actions_.assign(joint_names_.size(), 0.0);
        set_mode(Mode::kStand);
        RCLCPP_WARN(
          get_logger(), "STAND: ramping %zu motors to the default pose over %.1f s; '%s' walks",
          motors_.size(), stand_ramp_time_, walk_event_.c_str());
        if (stand_balance_ && trim_gate_.enabled) {
          RCLCPP_WARN(
            get_logger(), "STAND balance waiting for '%s' (press when it stands on its own weight)",
            trim_arm_event_.c_str());
        }
      }
    } else if (trim_gate_.enabled && event == trim_arm_event_) {
      if (mode_ != Mode::kStand || !stand_balance_) {
        RCLCPP_INFO(
          get_logger(), "Ignoring '%s': the STAND auto-lean only arms while standing%s",
          event.c_str(), stand_balance_ ? "" : " (and stand_balance_gain is 0)");
      } else if (!ramp_done()) {
        RCLCPP_WARN(get_logger(), "STAND balance not armed: still ramping to the stand pose");
      } else if (trim_gate_.armed) {
        RCLCPP_INFO(get_logger(), "STAND balance already armed");
      } else {
        stand_trim_.reset();
        trim_gate_.arm();
        RCLCPP_WARN(
          get_logger(), "STAND balance ARMED: holding mean ankle torque %+.2f Nm, max %.3f rad",
          stand_trim_.target_torque, stand_trim_.max_lean);
      }
    } else if (!squat_event_.empty() && event == squat_event_) {
      if (mode_ != Mode::kStand) {
        RCLCPP_INFO(get_logger(), "Ignoring '%s': the squat only runs in STAND", event.c_str());
      } else if (!ramp_done()) {
        RCLCPP_WARN(get_logger(), "SQUAT not started: still ramping to the stand pose");
      } else if (squat_.running) {
        RCLCPP_INFO(get_logger(), "SQUAT already running; ignoring '%s'", event.c_str());
      } else {
        squat_.begin(seconds(SteadyClock::now()));
        RCLCPP_WARN(
          get_logger(), "SQUAT start depth %.1f cm (%.1f s down, %.1f s hold, %.1f s up)",
          squat_.depth * 100.0, squat_.down_time, squat_.hold_time, squat_.down_time);
      }
    } else if ((event == lean_forward_event_ || event == lean_back_event_) && mode_ == Mode::kStand) {
      if (!have_ankles_) {
        RCLCPP_WARN(get_logger(), "STAND lean needs both ankle motors; ignoring %s", event.c_str());
        return;
      }
      manual_lean_.nudge(event == lean_forward_event_ ? -1 : +1);
      RCLCPP_WARN(
        get_logger(), "STAND lean -> %+.3f rad (%.1f deg %s). To start here next time: "
        "stand_balance: lean: %+.3f in the robot yaml", manual_lean_.target,
        std::abs(manual_lean_.target) * 180.0 / M_PI,
        manual_lean_.target < 0.0 ? "forward" : manual_lean_.target > 0.0 ? "back" : "level",
        manual_lean_.target);
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
    gait_clock_.reset();
    squat_.stop();
    target_freeze_.reset();
    for (auto & motor : motors_) {motor.freeze.reset();}
    heading_hold_.reset();
    last_hold_time_ = walk_time_;
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

  void disarm(const std::string & reason)
  {
    trim_gate_.reset();
    squat_.stop();
    if (mode_ != Mode::kDisarmed) {
      set_mode(Mode::kDisarmed);
      RCLCPP_WARN(
        get_logger(), "Disarmed (%s); targets stop (CSP motors hold, MIT motors damp)",
        reason.c_str());
    }
  }

  // While armed: a joint over its speed limit for joint_velocity_hold. Returns the reason, or empty.
  std::string overspeed() const
  {
    if (mode_ == Mode::kDisarmed) {
      return {};
    }
    const double t = seconds(SteadyClock::now());
    for (const auto & motor : motors_) {
      if (motor.speed.tripped(t, joint_velocity_hold_)) {
        return motor.name + ": joint speed " + std::to_string(motor.speed.peak).substr(0, 5) +
               " rad/s over its " + std::to_string(motor.max_velocity).substr(0, 4) + " rad/s limit";
      }
    }
    return {};
  }

  // While walking: stuck policy output, or motor feedback that stopped changing while its
  // target moved. Returns the reason, or empty.
  std::string frozen() const
  {
    if (mode_ != Mode::kWalk || freeze_timeout_ <= 0.0) {
      return {};
    }
    const double t = seconds(SteadyClock::now());
    if (target_freeze_.frozen(t, freeze_timeout_)) {
      return "policy output frozen (identical targets for " + std::to_string(freeze_timeout_) +
             " s)";
    }
    for (const auto & motor : motors_) {
      if (motor.freeze.frozen(t, freeze_timeout_, freeze_min_target_change_)) {
        return motor.name + ": feedback frozen (position unchanged for " +
               std::to_string(freeze_timeout_) + " s while its target moved)";
      }
    }
    return {};
  }

  void publish_joint_states()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = joint_names_;
    msg.position = measured_joints();
    msg.velocity.assign(joint_names_.size(), 0.0);
    for (const auto & motor : motors_) {msg.velocity[motor.joint] = motor.velocity;}
    joint_state_pub_->publish(msg);
  }

  void on_tick()
  {
    // Before the readiness checks, so the dashboard shows the legs even while disarmed
    // or waiting on a sensor.
    publish_joint_states();
    // Re-send the gains now and then, so a lost message cannot leave a motor on the
    // wrong gains (e.g. stiff STAND gains while the policy walks).
    if (++gain_refresh_ticks_ >= 25) {
      gain_refresh_ticks_ = 0;
      publish_gains();
    }
    if (mode_ == Mode::kWalk && !fresh(target_time_, target_timeout_)) {
      disarm("policy targets timed out");
    }
    if (mode_ != Mode::kDisarmed && tilted()) {
      disarm("base tilt exceeded max_tilt (fall)");
    }
    if (const std::string reason = frozen(); !reason.empty()) {
      disarm(reason);
    }
    if (const std::string reason = overspeed(); !reason.empty()) {
      disarm(reason);
    }
    const char * gap = nullptr;
    if (const std::string reason = not_ready(mode_ != Mode::kDisarmed, &gap); !reason.empty()) {
      disarm(reason);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting: %s", reason.c_str());
      return;
    }

    if (gap != nullptr) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "%s; running on the last readings", gap);
    }

    tb::ObservationInputs in;
    // The gamepad only publishes while input changes or is held, so a stale command means stop.
    in.command = fresh(command_time_, cmd_vel_timeout_) ? command_ : std::array<double, 3>{};
    if (mode_ == Mode::kWalk && require_imu_ && heading_hold_.enabled()) {
      const auto now = SteadyClock::now();
      in.command = heading_hold_.update(
        in.command, gravity_, angular_velocity_,
        std::chrono::duration<double>(now - last_hold_time_).count());
      last_hold_time_ = now;
    }
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
      if (held_gait_clock_) {
        in.gait_phase = gait_clock_.step(in.command, policy_dt_, gait_period_, gait_stand_threshold_);
      } else {
        const double t = std::chrono::duration<double>(SteadyClock::now() - walk_time_).count();
        in.gait_phase = std::fmod(t / gait_period_, 1.0);
      }
    }

    if (mode_ == Mode::kStand) {
      const double alpha =
        std::chrono::duration<double>(SteadyClock::now() - ramp_time_).count() / stand_ramp_time_;
      std::vector<double> pose = defaults_;
      double total_lean = 0.0;
      if (have_ankles_) {
        total_lean = manual_lean_.update(policy_dt_);
        for (const size_t m : balance_motor_) {pose[motors_[m].joint] += total_lean;}
      }
      if (squat_.running) {
        // Scripted squat (only ever started after the ramp): hip/knee/ankle pitch offsets per leg.
        const auto off = squat_.offsets(seconds(SteadyClock::now()));
        for (size_t side = 0; side < 2; ++side) {
          for (size_t k = 0; k < 3; ++k) {pose[motors_[squat_motor_[side][k]].joint] += off[k];}
        }
        if (!squat_.running) {RCLCPP_WARN(get_logger(), "SQUAT done");}
      }
      std::vector<double> targets = tb::interpolate(ramp_start_, pose, alpha);
      const bool trim_on = stand_balance_ && trim_gate_.active();
      if (trim_on && alpha >= 1.0) {
        // The torque each ankle drive is applying now: kp (target - q) - kd v, on the stand gains.
        std::array<double, 2> tau{};
        for (size_t side = 0; side < 2; ++side) {
          const auto & motor = motors_[balance_motor_[side]];
          tau[side] = motor.stand_gains[0] * (motor.last_target - motor.position) -
            motor.stand_gains[1] * motor.velocity;
        }
        const double lean = stand_trim_.update(tau[0], tau[1], policy_dt_);
        for (const size_t m : balance_motor_) {targets[motors_[m].joint] += lean;}
        total_lean += lean;
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "STAND balance: lean %+.3f rad (+ = back), ankle torque %+.1f / %+.1f Nm (filtered %+.2f)",
          lean, tau[0], tau[1], stand_trim_.filtered);
      }
      if (have_ankles_ && !trim_on && alpha >= 1.0 && manage_gains_) {
        // Balance readout for the D-pad lean (and while the gated trim waits for its button).
        std::array<double, 2> tau{};
        for (size_t side = 0; side < 2; ++side) {
          const auto & motor = motors_[balance_motor_[side]];
          tau[side] = motor.stand_gains[0] * (motor.last_target - motor.position) -
            motor.stand_gains[1] * motor.velocity;
        }
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "STAND lean %+.3f rad (+ = back), ankle torque %+.1f / %+.1f Nm (+ = weight ahead of the "
          "ankles, - = behind)%s", total_lean, tau[0], tau[1],
          stand_balance_ ? "; auto-lean waiting for its button" : "");
      }
      send_to_motors(targets, true);
      std_msgs::msg::Float64 lean_msg;
      lean_msg.data = total_lean;
      lean_pub_->publish(lean_msg);
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
  bool require_motor_status_{true};
  double motor_status_timeout_{};
  double freeze_timeout_{};
  double freeze_min_target_change_{};
  tb::VectorFreezeWatch target_freeze_;
  double joint_velocity_hold_{};
  std::string arm_event_;
  std::string disarm_event_;
  std::vector<std::string> disarm_events_;
  std::string walk_event_;
  Mode mode_{Mode::kDisarmed};
  bool manage_gains_{false};
  bool stand_balance_{false};
  tb::CenteringTrim stand_trim_;
  tb::TrimGate trim_gate_;
  std::string trim_arm_event_;
  tb::SquatMotion squat_;
  std::string squat_event_;
  std::array<std::array<size_t, 3>, 2> squat_motor_{};  // [left, right][hip, knee, ankle] motor index
  std::array<size_t, 2> balance_motor_{};
  bool have_ankles_{false};
  tb::ManualLean manual_lean_;
  std::string lean_forward_event_;
  std::string lean_back_event_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lean_pub_;
  int gain_refresh_ticks_{0};
  tb::ObservationLayout layout_;
  double gait_period_{};
  bool held_gait_clock_{false};
  double gait_stand_threshold_{0.1};
  double policy_dt_{0.02};
  tb::HeldGaitClock gait_clock_;
  tb::Quaternion imu_to_base_{1.0, 0.0, 0.0, 0.0};
  bool world_z_down_{false};
  double max_tilt_{};
  double stand_ramp_time_{};
  std::vector<double> ramp_start_;
  SteadyClock::time_point ramp_time_{};
  SteadyClock::time_point walk_time_{};
  tb::HeadingHold heading_hold_{0.0, 0.3};
  SteadyClock::time_point last_hold_time_{};

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
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
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
