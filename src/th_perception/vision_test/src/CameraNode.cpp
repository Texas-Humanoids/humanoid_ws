#include <algorithm>
#include <cctype>
#include <chrono>
#include <memory>
#include <stdexcept>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <filesystem>
#include <string>

#include <opencv2/imgcodecs.hpp>

class CameraNode : public rclcpp::Node

// make sure to run this command instead of just the ros2 run <pkg> <exec> structure to save photos into correct folder:
// ros2 run vision_test camera_node --ros-args -p output_dir:=$HOME/humanoid_ws/captured_images

{
public:
  CameraNode()
  : rclcpp::Node("camera_node")
  {
#if defined(__linux__)
    const std::string default_device = "/dev/video0";
    const int backend = cv::CAP_V4L2;
#elif defined(__APPLE__)
    // macOS has no /dev/video*; AVFoundation numbers cameras from 0.
    const std::string default_device = "0";
    const int backend = cv::CAP_AVFOUNDATION;
#else
    const std::string default_device = "0";
    const int backend = cv::CAP_ANY;
#endif
    const std::string device =
      this->declare_parameter<std::string>("device", default_device);

    // "0", "1", ... open a camera by index; anything else is a path like /dev/video2.
    const bool is_index = !device.empty() && std::all_of(
      device.begin(), device.end(), [](unsigned char c) {return std::isdigit(c);});
    const bool opened = is_index ?
      capture_.open(std::stoi(device), backend) :
      capture_.open(device, backend);

    if (!opened) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Could not open %s. On macOS, allow camera access for your terminal in "
        "System Settings > Privacy & Security > Camera.", device.c_str());
      throw std::runtime_error("camera open failed");
    }

    // Mac cameras default to 1920x1080 (6 MB per image), so ask for the same
    // size on every OS. The camera may pick the closest size it supports.
    const int width = this->declare_parameter<int>("width", 640);
    const int height = this->declare_parameter<int>("height", 480);
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height);

    RCLCPP_INFO(
      this->get_logger(), "Opened %s at %dx%d", device.c_str(),
      static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH)),
      static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT)));

    output_dir_ =
      this->declare_parameter<std::string>("output_dir", "captured_images");
    std::filesystem::create_directories(output_dir_);

    image_publisher_ =
      this->create_publisher<sensor_msgs::msg::Image>(
      "image_raw", rclcpp::SensorDataQoS());

    capture_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      [this]()
      {
        this->on_timer();
      });

    }
    
    private:
    cv::VideoCapture capture_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::TimerBase::SharedPtr capture_timer_;
    std::string output_dir_;
    rclcpp::Time last_save_time_{0, 0, RCL_ROS_TIME};

  void on_timer()
  {
    cv::Mat frame;
    if (!capture_.read(frame) || frame.empty()) {
      RCLCPP_WARN(this->get_logger(), "Failed to grab a frame.");
      return;
    }

    // Publish the frame as a ROS image (OpenCV frames are BGR, 8 bits per channel).
    sensor_msgs::msg::Image message;
    message.header.stamp = this->now();
    message.header.frame_id = "camera";
    message.height = frame.rows;
    message.width = frame.cols;
    message.encoding = "bgr8";
    message.is_bigendian = false;
    message.step = static_cast<uint32_t>(frame.step);
    message.data.assign(frame.datastart, frame.dataend);
    image_publisher_->publish(message);

        // Save a JPEG at most once every 2 seconds.
    const rclcpp::Time stamp = message.header.stamp;
    if ((stamp - last_save_time_).seconds() >= 2.0) {
      const std::string path =
        output_dir_ + "/frame_" + std::to_string(stamp.nanoseconds()) + ".jpg";
      if (cv::imwrite(path, frame)) {
        last_save_time_ = stamp;
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to write %s", path.c_str());
      }
    }

    // cv::mean gives the average of each channel; average those together.
    const cv::Scalar channel_means = cv::mean(frame);
    double average = 0.0;
    for (int c = 0; c < frame.channels(); ++c) {
      average += channel_means[c];
    }
    average /= frame.channels();

    RCLCPP_INFO(this->get_logger(), "Average pixel value: %.2f", average);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}