// Copyright 2026 Texas Humanoids. MIT License, see LICENSE at the repo root.

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "th_basic_cv/image_conversions.hpp"

namespace fs = std::filesystem;

// Publishes every .png/.jpg in a folder on image_raw, one after another, on a loop.
class SampleImagePublisher : public rclcpp::Node
{
public:
  SampleImagePublisher()
  : rclcpp::Node("sample_image_publisher")
  {
    const std::string default_dir =
      ament_index_cpp::get_package_share_directory("th_basic_cv") + "/sample_images";
    const std::string image_dir = declare_parameter<std::string>("image_dir", default_dir);
    const double rate_hz = declare_parameter<double>("rate_hz", 1.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "camera");
    if (rate_hz <= 0.0) {
      throw std::invalid_argument("rate_hz must be positive");
    }

    load_images(image_dir);

    pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz), [this]() {publish_next();});
  }

private:
  void load_images(const std::string & image_dir)
  {
    if (!fs::is_directory(image_dir)) {
      throw std::runtime_error("image_dir is not a directory: " + image_dir);
    }
    std::vector<fs::path> paths;
    for (const auto & entry : fs::directory_iterator(image_dir)) {
      const std::string ext = entry.path().extension().string();
      if (ext == ".png" || ext == ".jpg" || ext == ".jpeg") {
        paths.push_back(entry.path());
      }
    }
    std::sort(paths.begin(), paths.end());

    for (const auto & path : paths) {
      cv::Mat image = cv::imread(path.string(), cv::IMREAD_COLOR);
      if (image.empty()) {
        RCLCPP_WARN(get_logger(), "Could not read %s, skipping", path.c_str());
        continue;
      }
      images_.push_back(image);
      names_.push_back(path.filename().string());
    }
    if (images_.empty()) {
      throw std::runtime_error("no readable .png or .jpg images in " + image_dir);
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu images from %s", images_.size(), image_dir.c_str());
  }

  void publish_next()
  {
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = frame_id_;
    pub_->publish(th_basic_cv::to_image_msg(images_[next_], header));
    RCLCPP_INFO(get_logger(), "Published %s", names_[next_].c_str());
    next_ = (next_ + 1) % images_.size();
  }

  std::vector<cv::Mat> images_;
  std::vector<std::string> names_;
  size_t next_ = 0;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SampleImagePublisher>());
  rclcpp::shutdown();
  return 0;
}
