// Copyright 2022 Chen Jun

#include "rm_auto_aim/armor_detector.hpp"

// ROS
#include <rclcpp/logging.hpp>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <cmath>
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

  length = box.size.height;
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
  vector<vector<cv::Point>> contours;
  vector<Vec4i> hierarchy;
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  vector<Light> lights;
  for (const auto & contour : contours) {
    // There should be at least 5 points to fit the ellipse.
    // If the size of the contour is less than 5,
    // then it should not be considered as light anyway
    if (!contour.empty() && contour.size() < 5) {
      continue;
    }

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
  // TODO(chenjun): may need more judgement
  // The ratio of light is about 1/4 (width / height)
  float ratio = rect.size.aspectRatio();
  if (!(0.2f < ratio && ratio < 0.55f)) {
    return false;
  }

  // The angle of light is smaller than 30 degree.
  // The rotation angle in a clockwise direction.
  float angle = rect.angle;
  return angle < 30.0f || angle > 150.0f;
}

std::vector<Armor> ArmorDetector::matchLights(const std::vector<Light> & lights)
{
  auto start_time = node_.now();

  std::vector<Armor> armors;

  // Loop all the pairing of lights
  for (auto light = lights.begin(); light != lights.end(); light++) {
    for (auto sec_light = light + 1; sec_light != lights.end(); sec_light++) {
      if (isArmor(*light, *sec_light)) {
        armors.emplace_back(Armor(*light, *sec_light));
      }
    }
  }

  auto matchlights_time = (node_.now() - start_time).seconds();
  RCLCPP_DEBUG_STREAM(node_.get_logger(), "findLights used: " << matchlights_time * 1000.0 << "ms");

  return armors;
}

bool ArmorDetector::isArmor(const Light & light_1, const Light & light_2)
{
  // TODO(chenjun): need more judgement
  // FIXME(chenjun): this following data is only for test!!!
  // Ratio of the length of 2 lights
  float ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                : light_2.length / light_1.length;
  if (!(ratio > 0.7f)) {
    return false;
  }

  // Distance between the center of 2 lights
  float center_distance = cv::norm(light_1.center - light_2.center);
  float center_length_ratio = center_distance / (light_1.length + light_2.length);
  if (!(0.5f < center_length_ratio && center_length_ratio < 1.8f)) {
    return false;
  }

  // Angle of light center connection
  cv::Point2f diff = light_1.center - light_2.center;
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  return angle < 30;
}

Armor::Armor(const Light & l1, const Light & l2)
{
  if (l1.center.x < l2.center.x) {
    left_light = l1, right_light = l2;
  } else {
    left_light = l2, right_light = l1;
  }
}

}  // namespace rm_auto_aim
