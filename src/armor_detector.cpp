// Copyright 2022 Chen Jun

#include "rm_auto_aim/armor_detector.hpp"

// ROS
#include <opencv2/core/types.hpp>
#include <rclcpp/logging.hpp>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <vector>

namespace rm_auto_aim
{
Light::Light(cv::RotatedRect box) : cv::RotatedRect(box)
{
  cv::Point2f p[4];
  box.points(p);
  if (box.angle < 90) {
    bottom = (p[0] + p[3]) / 2, top = (p[1] + p[2]) / 2;
  } else {
    top = (p[0] + p[3]) / 2, bottom = (p[1] + p[2]) / 2;
  }
}

ArmorDetector::ArmorDetector(rclcpp::Node & node) : node_(node)
{
  // TODO(chenjun): Dynamic configure the following params
  r_params = {.hmin = 150, .hmax = 25, .vmin = 120, .vmax = 220};
  b_params = {.hmin = 75, .hmax = 125, .vmin = 160, .vmax = 255};
}

std::vector<Armor> ArmorDetector::detectArmors(const cv::Mat & img)
{
  auto binary_img = preprocessImage(img);

  auto lights = findLights(binary_img);

  std::vector<Armor> armors;
  return armors;
}

cv::Mat ArmorDetector::preprocessImage(const cv::Mat & img)
{
  auto start_time = node_.now();

  cv::Mat hsv_img;
  cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);

  cv::Mat binary_img;
  if (detect_color == RED) {
    cv::Mat red_img_1, red_img_2;
    cv::inRange(
      hsv_img, cv::Scalar(r_params.hmin, 1, r_params.vmin), cv::Scalar(179, 255, r_params.vmax),
      red_img_1);
    cv::inRange(
      hsv_img, cv::Scalar(0, 1, r_params.vmin), cv::Scalar(r_params.hmax, 255, r_params.vmax),
      red_img_2);
    binary_img = red_img_1 + red_img_2;
  } else if (detect_color == BULE) {
    cv::inRange(
      hsv_img, cv::Scalar(b_params.hmin, 1, b_params.vmin),
      cv::Scalar(b_params.hmax, 255, b_params.vmax), binary_img);
  }

  auto preprocess_time = (node_.now() - start_time).seconds();
  RCLCPP_DEBUG_STREAM(
    node_.get_logger(), "preprocessImage used: " << preprocess_time * 1000.0 << "ms");

  return binary_img;
}

std::vector<Light> ArmorDetector::findLights(const cv::Mat & binary_img)
{
  auto start_time = node_.now();

  using cv::Point;
  using cv::Vec4i;
  using std::vector;
  vector<vector<cv::Point> > contours;
  vector<Vec4i> hierarchy;
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  vector<Light> lights;
  for (const auto & contour : contours) {
    // There should be at least 5 points to fit the ellipse.
    // If the size of the contour is less than 5,
    // then it should not be considered as light anyway
    if (!contour.empty() && contour.size() < 5) continue;

    auto r_rect = cv::fitEllipse(contour);
    if (isLight(r_rect)) {
      lights.emplace_back(Light(r_rect));
    }
  }

  auto findlights_time = (node_.now() - start_time).seconds();
  RCLCPP_DEBUG_STREAM(node_.get_logger(), "findLights used: " << findlights_time * 1000.0 << "ms");

  return lights;
}

bool ArmorDetector::isLight(const cv::RotatedRect & rect)
{
  // TODO(chenjun): need more judgement
  // The ratio of light is about 1/4 (width / height)
  auto ratio = rect.size.aspectRatio();
  bool ratio_fits = 0.2 < ratio && ratio < 0.55;

  // The angle of light is smaller than 30 degree.
  // The rotation angle in a clockwise direction.
  auto angle = rect.angle;
  bool angle_fits = angle < 30 || angle > 150;

  return ratio_fits && angle_fits;
}

}  // namespace rm_auto_aim
