// Copyright 2026 Texas Humanoids. MIT License, see LICENSE at the repo root.

#include "th_basic_cv/basic_cv.hpp"

#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace th_basic_cv
{

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
  cv::GaussianBlur(gray, blurred, cv::Size(kernel, kernel), 0.0);

  cv::Canny(blurred, result.edges, params.canny_low, params.canny_high);

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
      result.detections.push_back({cv::boundingRect(contour), area});
    }
  }

  result.annotated = bgr.clone();
  const cv::Scalar green(0, 200, 0);
  for (const auto & detection : result.detections) {
    cv::rectangle(result.annotated, detection.box, green, 2);
  }
  const std::string label = std::to_string(result.detections.size()) + " objects";
  cv::putText(
    result.annotated, label, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.9,
    green, 2);

  return result;
}

}  // namespace th_basic_cv
