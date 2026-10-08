// Copyright 2026 Texas Humanoids. MIT License, see LICENSE at the repo root.

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "th_basic_cv/basic_cv.hpp"
#include "th_basic_cv/image_conversions.hpp"

namespace
{

cv::Mat blank(uint8_t value = 230)
{
  return cv::Mat(240, 320, CV_8UC3, cv::Scalar(value, value, value));
}

}  // namespace

TEST(Pipeline, FindsEachSolidRectangle)
{
  cv::Mat image = blank();
  cv::rectangle(image, cv::Rect(20, 20, 60, 60), cv::Scalar(0, 0, 200), cv::FILLED);
  cv::rectangle(image, cv::Rect(120, 40, 70, 50), cv::Scalar(200, 0, 0), cv::FILLED);
  cv::rectangle(image, cv::Rect(220, 120, 60, 80), cv::Scalar(0, 150, 0), cv::FILLED);

  const auto result = th_basic_cv::run_pipeline(image, th_basic_cv::PipelineParams{});

  EXPECT_EQ(result.detections.size(), 3u);
  EXPECT_EQ(result.edges.type(), CV_8UC1);
  EXPECT_EQ(result.annotated.size(), image.size());
}

TEST(Pipeline, BlankImageHasNoDetections)
{
  const auto result = th_basic_cv::run_pipeline(blank(), th_basic_cv::PipelineParams{});
  EXPECT_TRUE(result.detections.empty());
  EXPECT_NEAR(result.mean_brightness, 230.0, 1.0);
}

TEST(Pipeline, IgnoresSpecksBelowMinArea)
{
  cv::Mat image = blank();
  cv::rectangle(image, cv::Rect(100, 100, 4, 4), cv::Scalar(0, 0, 0), cv::FILLED);
  const auto result = th_basic_cv::run_pipeline(image, th_basic_cv::PipelineParams{});
  EXPECT_TRUE(result.detections.empty());
}

TEST(Pipeline, AcceptsEvenBlurKernel)
{
  th_basic_cv::PipelineParams params;
  params.blur_kernel = 4;
  EXPECT_NO_THROW(th_basic_cv::run_pipeline(blank(), params));
}

TEST(Pipeline, RejectsNonBgrInput)
{
  const cv::Mat gray(10, 10, CV_8UC1, cv::Scalar(0));
  EXPECT_THROW(
    th_basic_cv::run_pipeline(gray, th_basic_cv::PipelineParams{}), std::invalid_argument);
}

TEST(SampleImages, ShapesAndBlocksGiveExpectedCounts)
{
  const std::string dir = SAMPLE_IMAGE_DIR;
  const cv::Mat shapes = cv::imread(dir + "/01_shapes.png", cv::IMREAD_COLOR);
  const cv::Mat blocks = cv::imread(dir + "/02_blocks.jpg", cv::IMREAD_COLOR);
  ASSERT_FALSE(shapes.empty());
  ASSERT_FALSE(blocks.empty());

  const th_basic_cv::PipelineParams params;
  EXPECT_EQ(th_basic_cv::run_pipeline(shapes, params).detections.size(), 3u);
  EXPECT_EQ(th_basic_cv::run_pipeline(blocks, params).detections.size(), 5u);
}

TEST(Conversions, BgrRoundTrip)
{
  cv::Mat image = blank();
  image.at<cv::Vec3b>(5, 7) = cv::Vec3b(1, 2, 3);
  const auto msg = th_basic_cv::to_image_msg(image, std_msgs::msg::Header{});
  EXPECT_EQ(msg.encoding, "bgr8");

  const cv::Mat back = th_basic_cv::to_bgr_mat(msg);
  EXPECT_EQ(cv::norm(image, back, cv::NORM_INF), 0.0);
}

TEST(Conversions, Rgb8IsSwappedToBgr)
{
  sensor_msgs::msg::Image msg;
  msg.encoding = "rgb8";
  msg.width = 1;
  msg.height = 1;
  msg.step = 3;
  msg.data = {10, 20, 30};

  const cv::Mat bgr = th_basic_cv::to_bgr_mat(msg);
  EXPECT_EQ(bgr.at<cv::Vec3b>(0, 0), cv::Vec3b(30, 20, 10));
}

TEST(Conversions, RejectsShortDataAndUnknownEncoding)
{
  sensor_msgs::msg::Image msg;
  msg.encoding = "bgr8";
  msg.width = 4;
  msg.height = 4;
  msg.step = 12;
  msg.data.resize(10);
  EXPECT_THROW(th_basic_cv::to_bgr_mat(msg), std::invalid_argument);

  msg.encoding = "16UC1";
  EXPECT_THROW(th_basic_cv::to_bgr_mat(msg), std::invalid_argument);
}
