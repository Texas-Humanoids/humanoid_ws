// Copyright 2026 Texas Humanoids. MIT License, see LICENSE at the repo root.

#include <chrono>
#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "th_basic_cv/basic_cv.hpp"
#include "th_basic_cv/image_conversions.hpp"

class BasicCvNode : public rclcpp::Node
{
public:
  BasicCvNode()
  : rclcpp::Node("basic_cv_node")
  {
    // Read on every frame, so `ros2 param set` tunes the pipeline live.
    declare_parameter<int>("blur_kernel", 5);
    declare_parameter<double>("canny_low", 50.0);
    declare_parameter<double>("canny_high", 150.0);
    declare_parameter<double>("min_contour_area", 200.0);

    edges_pub_ = create_publisher<sensor_msgs::msg::Image>("image_edges", 5);
    annotated_pub_ = create_publisher<sensor_msgs::msg::Image>("image_annotated", 5);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "image_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_image(*msg);});
  }

private:
  void on_image(const sensor_msgs::msg::Image & msg)
  {
    th_basic_cv::PipelineParams params;
    params.blur_kernel = static_cast<int>(get_parameter("blur_kernel").as_int());
    params.canny_low = get_parameter("canny_low").as_double();
    params.canny_high = get_parameter("canny_high").as_double();
    params.min_contour_area = get_parameter("min_contour_area").as_double();

    try {
      const auto start = std::chrono::steady_clock::now();
      const auto result = th_basic_cv::run_pipeline(th_basic_cv::to_bgr_mat(msg), params);
      const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();

      edges_pub_->publish(th_basic_cv::to_image_msg(result.edges, msg.header));
      annotated_pub_->publish(th_basic_cv::to_image_msg(result.annotated, msg.header));

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%zu objects, mean brightness %.1f, processed in %.1f ms",
        result.detections.size(), result.mean_brightness, ms);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Skipping frame: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr edges_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasicCvNode>());
  rclcpp::shutdown();
  return 0;
}
