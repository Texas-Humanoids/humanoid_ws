// Copyright 2026 Texas Humanoids. MIT License, see LICENSE at the repo root.

#ifndef TH_BASIC_CV__IMAGE_CONVERSIONS_HPP_
#define TH_BASIC_CV__IMAGE_CONVERSIONS_HPP_

#include <opencv2/core.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

namespace th_basic_cv
{

// Converts bgr8, rgb8, or mono8 to an 8-bit BGR cv::Mat (always a copy).
// Throws std::invalid_argument for any other encoding.
cv::Mat to_bgr_mat(const sensor_msgs::msg::Image & msg);

// Encodes CV_8UC3 as bgr8 and CV_8UC1 as mono8.
sensor_msgs::msg::Image to_image_msg(
  const cv::Mat & mat, const std_msgs::msg::Header & header);

}  // namespace th_basic_cv

#endif  // TH_BASIC_CV__IMAGE_CONVERSIONS_HPP_
