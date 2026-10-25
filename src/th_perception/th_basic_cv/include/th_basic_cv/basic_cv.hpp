// Copyright 2026 Texas Humanoids. MIT License, see LICENSE at the repo root.

#ifndef TH_BASIC_CV__BASIC_CV_HPP_
#define TH_BASIC_CV__BASIC_CV_HPP_

#include <vector>

#include <opencv2/core.hpp>

namespace th_basic_cv
{

struct PipelineParams
{
  int blur_kernel = 5;
  double canny_low = 50.0;
  double canny_high = 150.0;
  double min_contour_area = 200.0;
};

struct Detection
{
  cv::Rect box;
  double area;
};

struct PipelineResult
{
  cv::Mat edges;
  cv::Mat annotated;
  std::vector<Detection> detections;
  double mean_brightness;
};

// Grayscale, blur, Canny edges, then outer contours larger than min_contour_area.
// Input must be an 8-bit BGR image.
PipelineResult run_pipeline(const cv::Mat & bgr, const PipelineParams & params);

}  // namespace th_basic_cv

#endif  // TH_BASIC_CV__BASIC_CV_HPP_
