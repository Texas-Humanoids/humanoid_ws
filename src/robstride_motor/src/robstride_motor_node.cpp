#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class RobstrideMotorNode : public rclcpp::Node
{
public:
  RobstrideMotorNode()
  : rclcpp::Node("robstride_motor_node")
  {
    position_publisher_ =
      this->create_publisher<std_msgs::msg::Float64>(
                "motor/simulated_position", 10);

    position_subscription_ =
      this->create_subscription<std_msgs::msg::Float64>(
                "motor/target_position",
                10,
      [this](std_msgs::msg::Float64::ConstSharedPtr message)
      {
        this->on_position_command(message);
                });

    feedback_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
      [this]()
      {
        this->update_simulation();
            });

    RCLCPP_WARN(
            this->get_logger(),
            "Simulation only: no physical motor is connected.");
  }

  void stop()
  {
    feedback_timer_->cancel();
    target_position_ = simulated_position_;

    RCLCPP_INFO(
            this->get_logger(),
            "Simulation stopped. No hardware stop command was sent.");
  }

private:
  double target_position_ = 0.0;
  double simulated_position_ = 0.0;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
    position_subscription_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    position_publisher_;

  rclcpp::TimerBase::SharedPtr feedback_timer_;

  void on_position_command(
    std_msgs::msg::Float64::ConstSharedPtr message)
  {
    if (!std::isfinite(message->data)) {
      RCLCPP_WARN(
                this->get_logger(),
                "Ignoring a non-finite target position.");
      return;
    }

    target_position_ = message->data;

    RCLCPP_INFO(
            this->get_logger(),
            "New target: %.3f rad",
            target_position_);
  }

  void update_simulation()
  {
        // Toy simulation: move at most 0.02 radians per timer tick.
    const double error = target_position_ - simulated_position_;
    const double max_step = 0.02;

    if (std::abs(error) <= max_step) {
      simulated_position_ = target_position_;
    } else {
      simulated_position_ +=
        (error > 0.0) ? max_step : -max_step;
    }

    std_msgs::msg::Float64 feedback;
    feedback.data = simulated_position_;
    position_publisher_->publish(feedback);
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
