// Copyright 2026 Texas Humanoids. MIT License, see LICENSE at the repo root.

#include <gtest/gtest.h>

#include <set>
#include <stdexcept>
#include <string>
#include <vector>

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

// "color shape" for every detection, order-independent.
std::multiset<std::string> labels(const th_basic_cv::PipelineResult & result)
{
  std::multiset<std::string> out;
  for (const auto & detection : result.detections) {
    out.insert(detection.color + " " + detection.shape);
  }
  return out;
}

std::multiset<std::string> colors(const th_basic_cv::PipelineResult & result)
{
  std::multiset<std::string> out;
  for (const auto & detection : result.detections) {
    out.insert(detection.color);
  }
  return out;
}

std::vector<cv::Point> circle_points(cv::Point center, int radius)
{
  std::vector<cv::Point> points;
  cv::ellipse2Poly(center, cv::Size(radius, radius), 0, 0, 360, 2, points);
  return points;
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

TEST(SampleImages, ShapesAndColorsAreLabeled)
{
  const std::string dir = SAMPLE_IMAGE_DIR;
  const th_basic_cv::PipelineParams params;

  const auto shapes = th_basic_cv::run_pipeline(
    cv::imread(dir + "/01_shapes.png", cv::IMREAD_COLOR), params);
  EXPECT_EQ(
    labels(shapes),
    (std::multiset<std::string>{"red rectangle", "blue circle", "green triangle"}));

  const auto blocks = th_basic_cv::run_pipeline(
    cv::imread(dir + "/02_blocks.jpg", cv::IMREAD_COLOR), params);
  EXPECT_EQ(
    colors(blocks),
    (std::multiset<std::string>{"red", "blue", "yellow", "green", "purple"}));
}

TEST(Shape, Triangle)
{
  EXPECT_EQ(th_basic_cv::classify_shape({{10, 10}, {110, 10}, {60, 100}}), "triangle");
}

TEST(Shape, SquareAndRectangle)
{
  EXPECT_EQ(th_basic_cv::classify_shape({{0, 0}, {100, 0}, {100, 100}, {0, 100}}), "square");
  EXPECT_EQ(th_basic_cv::classify_shape({{0, 0}, {200, 0}, {200, 80}, {0, 80}}), "rectangle");
}

TEST(Shape, RotatedSquareIsStillASquare)
{
  EXPECT_EQ(th_basic_cv::classify_shape({{50, 0}, {100, 50}, {50, 100}, {0, 50}}), "square");
}

TEST(Shape, Circle)
{
  EXPECT_EQ(th_basic_cv::classify_shape(circle_points({100, 100}, 60)), "circle");
}

TEST(Shape, OtherOutlinesArePolygons)
{
  // An L shape: six corners and far from round.
  EXPECT_EQ(
    th_basic_cv::classify_shape({{0, 0}, {40, 0}, {40, 120}, {120, 120}, {120, 160}, {0, 160}}),
    "polygon");
}

TEST(Color, NamesHues)
{
  // cv::Scalar is BGR.
  EXPECT_EQ(th_basic_cv::name_color(cv::Scalar(30, 30, 220)), "red");
  EXPECT_EQ(th_basic_cv::name_color(cv::Scalar(0, 140, 255)), "orange");
  EXPECT_EQ(th_basic_cv::name_color(cv::Scalar(20, 220, 230)), "yellow");
  EXPECT_EQ(th_basic_cv::name_color(cv::Scalar(40, 180, 40)), "green");
  EXPECT_EQ(th_basic_cv::name_color(cv::Scalar(220, 90, 30)), "blue");
  EXPECT_EQ(th_basic_cv::name_color(cv::Scalar(170, 50, 140)), "purple");
}

TEST(Color, RedWrapsAroundTheHueCircle)
{
  // Red sits at both ends of the hue range (near 0 and near 179).
  EXPECT_EQ(th_basic_cv::name_color(cv::Scalar(20, 0, 200)), "red");
  EXPECT_EQ(th_basic_cv::name_color(cv::Scalar(70, 0, 200)), "red");
}

TEST(Color, NamesNeutrals)
{
  EXPECT_EQ(th_basic_cv::name_color(cv::Scalar(20, 20, 20)), "black");
  EXPECT_EQ(th_basic_cv::name_color(cv::Scalar(245, 245, 245)), "white");
  EXPECT_EQ(th_basic_cv::name_color(cv::Scalar(128, 128, 128)), "gray");
}

TEST(Pipeline, LabelsDrawnShapesWithTheirColors)
{
  cv::Mat image(360, 480, CV_8UC3, cv::Scalar(235, 235, 235));
  const std::vector<std::vector<cv::Point>> triangle{{{40, 140}, {140, 140}, {90, 40}}};
  cv::fillPoly(image, triangle, cv::Scalar(30, 30, 220));                          // red
  cv::circle(image, cv::Point(250, 90), 50, cv::Scalar(220, 90, 30), cv::FILLED);  // blue
  cv::rectangle(image, cv::Rect(340, 40, 100, 100), cv::Scalar(40, 180, 40), cv::FILLED);
  cv::rectangle(image, cv::Rect(40, 220, 180, 80), cv::Scalar(20, 220, 230), cv::FILLED);
  const std::vector<std::vector<cv::Point>> diamond{
    {{330, 200}, {390, 260}, {330, 320}, {270, 260}}};
  cv::fillPoly(image, diamond, cv::Scalar(0, 140, 255));                           // orange

  const auto result = th_basic_cv::run_pipeline(image, th_basic_cv::PipelineParams{});
  EXPECT_EQ(
    labels(result),
    (std::multiset<std::string>{
    "red triangle", "blue circle", "green square", "yellow rectangle", "orange square"}));
}

TEST(Pipeline, FindsYellowOnWhite)
{
  // Yellow and white are nearly the same brightness, so grayscale edges alone miss it.
  cv::Mat image = blank(240);
  cv::rectangle(image, cv::Rect(100, 60, 120, 80), cv::Scalar(30, 230, 240), cv::FILLED);
  EXPECT_EQ(
    labels(th_basic_cv::run_pipeline(image, th_basic_cv::PipelineParams{})),
    (std::multiset<std::string>{"yellow rectangle"}));
}

TEST(Pipeline, LabelsBlackAndWhiteObjects)
{
  cv::Mat light = blank();
  cv::rectangle(light, cv::Rect(100, 60, 100, 100), cv::Scalar(15, 15, 15), cv::FILLED);
  EXPECT_EQ(
    labels(th_basic_cv::run_pipeline(light, th_basic_cv::PipelineParams{})),
    (std::multiset<std::string>{"black square"}));

  cv::Mat dark = blank(25);
  cv::rectangle(dark, cv::Rect(100, 60, 100, 100), cv::Scalar(245, 245, 245), cv::FILLED);
  EXPECT_EQ(
    labels(th_basic_cv::run_pipeline(dark, th_basic_cv::PipelineParams{})),
    (std::multiset<std::string>{"white square"}));
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
