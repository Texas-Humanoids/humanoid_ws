// Copyright 2026 Texas Humanoids. MIT License, see LICENSE at the repo root.

#include "th_basic_cv/basic_cv.hpp"

#include <algorithm>
#include <climits>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace th_basic_cv
{

namespace
{

// Average color of the pixels inside a contour, staying a few pixels in from
// the outline so edge and background pixels don't skew it.
cv::Scalar mean_color_inside(
  const cv::Mat & bgr, const std::vector<cv::Point> & contour, const cv::Rect & box)
{
  cv::Mat mask = cv::Mat::zeros(box.size(), CV_8UC1);
  cv::drawContours(
    mask, std::vector<std::vector<cv::Point>>{contour}, 0, cv::Scalar(255), cv::FILLED,
    cv::LINE_8, cv::noArray(), INT_MAX, -box.tl());
  cv::Mat inner;
  cv::erode(mask, inner, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7)));
  return cv::mean(bgr(box), cv::countNonZero(inner) > 0 ? inner : mask);
}

}  // namespace

std::string classify_shape(const std::vector<cv::Point> & contour)
{
  const double perimeter = cv::arcLength(contour, true);
  if (perimeter <= 0.0) {
    return "polygon";
  }

  std::vector<cv::Point> corners;
  cv::approxPolyDP(contour, corners, 0.03 * perimeter, true);
  if (corners.size() == 3) {
    return "triangle";
  }
  if (corners.size() == 4) {
    // minAreaRect measures the sides even when the shape is rotated.
    const cv::Size2f size = cv::minAreaRect(contour).size;
    const double longest = std::max(size.width, size.height);
    const double ratio = longest > 0.0 ? std::min(size.width, size.height) / longest : 0.0;
    return ratio > 0.9 ? "square" : "rectangle";
  }

  // 1.0 for a perfect circle, lower for anything with corners or dents.
  const double circularity = 4.0 * CV_PI * cv::contourArea(contour) / (perimeter * perimeter);
  return circularity > 0.8 ? "circle" : "polygon";
}

std::string name_color(const cv::Scalar & bgr)
{
  const cv::Mat pixel(1, 1, CV_8UC3, cv::Scalar(bgr[0], bgr[1], bgr[2]));
  cv::Mat hsv;
  cv::cvtColor(pixel, hsv, cv::COLOR_BGR2HSV);
  const cv::Vec3b value = hsv.at<cv::Vec3b>(0, 0);
  const int hue = value[0];  // OpenCV hue runs 0-179, half of degrees.
  const int saturation = value[1];
  const int brightness = value[2];

  if (brightness < 50) {
    return "black";
  }
  if (saturation < 50) {
    return brightness > 200 ? "white" : "gray";
  }
  if (hue < 8 || hue >= 160) {
    return "red";
  }
  if (hue < 20) {
    return "orange";
  }
  if (hue < 35) {
    return "yellow";
  }
  if (hue < 85) {
    return "green";
  }
  if (hue < 130) {
    return "blue";
  }
  return "purple";
}

PipelineResult run_pipeline(const cv::Mat & bgr, const PipelineParams & params)
{
  if (bgr.empty() || bgr.type() != CV_8UC3) {
    throw std::invalid_argument("run_pipeline expects a non-empty 8-bit BGR image");
  }

  PipelineResult result;

  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  result.mean_brightness = cv::mean(gray)[0];

  // GaussianBlur needs an odd kernel size.
  int kernel = params.blur_kernel < 1 ? 1 : params.blur_kernel;
  if (kernel % 2 == 0) {
    kernel += 1;
  }
  cv::Mat blurred;
  cv::GaussianBlur(bgr, blurred, cv::Size(kernel, kernel), 0.0);

  // Edges from each color channel, combined. Grayscale alone misses objects that
  // differ from the background in color but not brightness, like yellow on white.
  std::vector<cv::Mat> channels;
  cv::split(blurred, channels);
  result.edges = cv::Mat::zeros(bgr.size(), CV_8UC1);
  for (const auto & channel : channels) {
    cv::Mat channel_edges;
    cv::Canny(channel, channel_edges, params.canny_low, params.canny_high);
    result.edges |= channel_edges;
  }

  // Close small gaps so each object's outline becomes one contour.
  cv::Mat closed;
  cv::morphologyEx(
    result.edges, closed, cv::MORPH_CLOSE,
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(closed, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const auto & contour : contours) {
    const double area = cv::contourArea(contour);
    if (area >= params.min_contour_area) {
      const cv::Rect box = cv::boundingRect(contour);
      result.detections.push_back(
        {box, area, classify_shape(contour), name_color(mean_color_inside(bgr, contour, box))});
    }
  }

  result.annotated = bgr.clone();
  const cv::Scalar green(0, 200, 0);
  for (const auto & detection : result.detections) {
    cv::rectangle(result.annotated, detection.box, green, 2);
    // Label above the box, or inside it when the box touches the top edge.
    const int label_y = detection.box.y > 20 ? detection.box.y - 6 : detection.box.y + 18;
    cv::putText(
      result.annotated, detection.color + " " + detection.shape,
      cv::Point(detection.box.x, label_y), cv::FONT_HERSHEY_SIMPLEX, 0.55, green, 2);
  }
  const std::string label = std::to_string(result.detections.size()) + " objects";
  cv::putText(
    result.annotated, label, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.9,
    green, 2);

  return result;
}

}  // namespace th_basic_cv
