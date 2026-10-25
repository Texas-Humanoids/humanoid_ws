// Copyright 2026 Texas Humanoids. MIT License, see LICENSE at the repo root.

#include "th_basic_cv/image_conversions.hpp"

#include <stdexcept>
#include <string>

#include <opencv2/imgproc.hpp>

namespace th_basic_cv
{

cv::Mat to_bgr_mat(const sensor_msgs::msg::Image & msg)
{
  int type;
  if (msg.encoding == "bgr8" || msg.encoding == "rgb8") {
    type = CV_8UC3;
  } else if (msg.encoding == "mono8") {
    type = CV_8UC1;
  } else {
    throw std::invalid_argument("unsupported image encoding: " + msg.encoding);
  }
  const size_t row_bytes = static_cast<size_t>(msg.width) * (type == CV_8UC3 ? 3 : 1);
  if (msg.step < row_bytes || msg.data.size() < static_cast<size_t>(msg.height) * msg.step) {
    throw std::invalid_argument("image data is smaller than height * step");
  }

  const cv::Mat view(
    static_cast<int>(msg.height), static_cast<int>(msg.width), type,
    const_cast<uint8_t *>(msg.data.data()), msg.step);

  cv::Mat bgr;
  if (msg.encoding == "rgb8") {
    cv::cvtColor(view, bgr, cv::COLOR_RGB2BGR);
  } else if (msg.encoding == "mono8") {
    cv::cvtColor(view, bgr, cv::COLOR_GRAY2BGR);
  } else {
    bgr = view.clone();
  }
  return bgr;
}

sensor_msgs::msg::Image to_image_msg(
  const cv::Mat & mat, const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::Image msg;
  msg.header = header;
  if (mat.type() == CV_8UC3) {
    msg.encoding = "bgr8";
  } else if (mat.type() == CV_8UC1) {
    msg.encoding = "mono8";
  } else {
    throw std::invalid_argument("to_image_msg supports CV_8UC3 and CV_8UC1 only");
  }

  const cv::Mat continuous = mat.isContinuous() ? mat : mat.clone();
  msg.height = static_cast<uint32_t>(continuous.rows);
  msg.width = static_cast<uint32_t>(continuous.cols);
  msg.is_bigendian = false;
  msg.step = static_cast<uint32_t>(continuous.cols * continuous.elemSize());
  msg.data.assign(continuous.datastart, continuous.dataend);
  return msg;
}

}  // namespace th_basic_cv
