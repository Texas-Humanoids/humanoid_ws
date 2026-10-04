#include <chrono>
#include <memory>
#include <stdexcept>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class AsciiCameraNode : public rclcpp::Node
{
public:
  AsciiCameraNode()
  : rclcpp::Node("ascii_camera_node")
  {
    const std::string device =
      this->declare_parameter<std::string>("device", "/dev/video0");

    if (!capture_.open(device, cv::CAP_V4L2)) {
      RCLCPP_ERROR(this->get_logger(), "Could not open %s", device.c_str());
      throw std::runtime_error("camera open failed");
    }

    RCLCPP_INFO(this->get_logger(), "Opened %s", device.c_str());

    image_publisher_ =
      this->create_publisher<sensor_msgs::msg::Image>(
      "image_raw", rclcpp::SensorDataQoS());

    capture_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]()
      {
        this->on_timer();
      });
  }

private:
  cv::VideoCapture capture_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::TimerBase::SharedPtr capture_timer_;

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
  rclcpp::spin(std::make_shared<AsciiCameraNode>());
  rclcpp::shutdown();
  return 0;
}