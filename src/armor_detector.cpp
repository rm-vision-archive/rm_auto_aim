// Copyright 2022 Chen Jun

#include "rm_auto_aim/armor_detector.hpp"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <vector>

namespace rm_auto_aim
{
ArmorDetector::ArmorDetector(rclcpp::Node & node) : node_(node) {}

std::vector<Armor> ArmorDetector::detectArmors(const cv::Mat & img)
{
  preprocessImage(img);

  std::vector<Armor> armors;
  return armors;
}

cv::Mat ArmorDetector::preprocessImage(const cv::Mat & img)
{
  auto start_time = node_.now();

  cv::Mat hsv_img;
  cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);
  auto cvt_time = (node_.now() - start_time).seconds();
  RCLCPP_INFO_STREAM(node_.get_logger(), "cvtColor used: " << cvt_time * 1000.0 << "ms");

  cv::Mat tmp;
  return tmp;
}

}  // namespace rm_auto_aim
