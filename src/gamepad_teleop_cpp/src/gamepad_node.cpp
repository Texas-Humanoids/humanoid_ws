#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

static const std::vector<std::string> BUTTON_NAMES = {
  "A", "B", "X", "Y", "LB", "RB", "BACK", "START", "GUIDE", "L_STICK", "R_STICK"};

class GamepadNode : public rclcpp::Node
{
public:
  GamepadNode() : Node("gamepad_node")
  {
    axis_linear_ = declare_parameter<int>("axis_linear", 1);
    axis_angular_ = declare_parameter<int>("axis_angular", 0);
    scale_linear_ = declare_parameter<double>("scale_linear", 0.5);
    scale_angular_ = declare_parameter<double>("scale_angular", 1.0);
    enable_button_ = declare_parameter<int>("enable_button", 4);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    btn_pub_ = create_publisher<std_msgs::msg::String>("gamepad/button_event", 10);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&GamepadNode::joy_cb, this, std::placeholders::_1));
  }

private:
  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Button press events (rising edge only)
    if (prev_buttons_.size() != msg->buttons.size()) {
      prev_buttons_.assign(msg->buttons.size(), 0);
    }
    for (size_t i = 0; i < msg->buttons.size(); ++i) {
      if (msg->buttons[i] && !prev_buttons_[i]) {
        std::string name = i < BUTTON_NAMES.size() ? BUTTON_NAMES[i]
                                                   : "BUTTON_" + std::to_string(i);
        std_msgs::msg::String out;
        out.data = name + " pressed";
        btn_pub_->publish(out);
      }
    }
    prev_buttons_ = msg->buttons;

    // Velocity command
    bool enabled = enable_button_ >= 0 &&
      static_cast<size_t>(enable_button_) < msg->buttons.size() &&
      msg->buttons[enable_button_] == 1;

    geometry_msgs::msg::Twist twist;
    if (enabled) {
      if (static_cast<size_t>(axis_linear_) < msg->axes.size()) {
        twist.linear.x = msg->axes[axis_linear_] * scale_linear_;
      }
      if (static_cast<size_t>(axis_angular_) < msg->axes.size()) {
        twist.angular.z = msg->axes[axis_angular_] * scale_angular_;
      }
      cmd_pub_->publish(twist);
    } else if (was_enabled_) {
      cmd_pub_->publish(twist);  // one stop message on release
    }
    was_enabled_ = enabled;
  }

  int axis_linear_, axis_angular_, enable_button_;
  double scale_linear_, scale_angular_;
  bool was_enabled_{false};
  std::vector<int32_t> prev_buttons_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr btn_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GamepadNode>());
  rclcpp::shutdown();
  return 0;
}
