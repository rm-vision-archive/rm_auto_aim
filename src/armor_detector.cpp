// Copyright 2022 Chen Jun

#include "rm_auto_aim/armor_detector.hpp"

// ROS
#include <rclcpp/logging.hpp>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <vector>

namespace rm_auto_aim
{
ArmorDetector::ArmorDetector(rclcpp::Node & node) : node_(node)
{
  // TODO Dynamic configure the following params
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

}  // namespace rm_auto_aim
