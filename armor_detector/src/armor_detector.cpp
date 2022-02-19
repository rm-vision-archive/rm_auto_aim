// Copyright 2022 Chen Jun

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <vector>

#include "armor_detector/armor_detector.hpp"
#include "auto_aim_interfaces/msg/debug_armor.hpp"
#include "auto_aim_interfaces/msg/debug_light.hpp"

namespace rm_auto_aim
{
Light::Light(cv::RotatedRect box) : cv::RotatedRect(box)
{
  cv::Point2f p[4];
  box.points(p);
  if (box.angle > 45) {
    top = (p[0] + p[1]) / 2, bottom = (p[2] + p[3]) / 2;
    length = box.size.width;
  } else {
    top = (p[1] + p[2]) / 2, bottom = (p[0] + p[3]) / 2;
    length = box.size.height;
  }
}

Armor::Armor(const Light & l1, const Light & l2)
{
  if (l1.center.x < l2.center.x) {
    left_light = l1, right_light = l2;
  } else {
    left_light = l2, right_light = l1;
  }
  center = (left_light.center + right_light.center) / 2;
}

ArmorDetector::ArmorDetector(rclcpp::Node * node)
{
  // TODO(chenjun): Dynamic configure some of the following params
  // 0-BLUE 1-Red
  detect_color = node->declare_parameter("default_detect_color", 0) ? RED : BULE;

  b = {
    .hmin = node->declare_parameter("preprocess.b.hmin", 75),
    .hmax = node->declare_parameter("preprocess.b.hmax", 120),
    .lmin = node->declare_parameter("preprocess.b.lmin", 150),
    .smin = node->declare_parameter("preprocess.b.smin", 160)};
  r = {
    .hmin = node->declare_parameter("preprocess.r.hmin", 150),
    .hmax = node->declare_parameter("preprocess.r.hmax", 25),
    .lmin = node->declare_parameter("preprocess.r.lmin", 140),
    .smin = node->declare_parameter("preprocess.r.smin", 100)};

  l = {
    .min_ratio = node->declare_parameter("light.min_ratio", 0.1),
    .max_ratio = node->declare_parameter("light.max_ratio", 0.55),
    .max_angle = node->declare_parameter("light.max_angle", 40.0)};

  a = {
    .min_light_ratio = node->declare_parameter("armor.min_light_ratio", 0.6),
    .min_center_ratio = node->declare_parameter("armor.min_center_ratio", 0.4),
    .max_center_ratio = node->declare_parameter("armor.max_center_ratio", 1.6),
    .max_angle = node->declare_parameter("armor.max_angle", 35.0)};
}

cv::Mat ArmorDetector::preprocessImage(const cv::Mat & img)
{
  cv::Mat hsv_img;
  cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HLS);

  cv::Mat binary_img;
  if (detect_color == RED) {
    cv::Mat red_img_1, red_img_2;
    cv::inRange(hsv_img, cv::Scalar(r.hmin, r.lmin, r.smin), cv::Scalar(179, 255, 255), red_img_1);
    cv::inRange(hsv_img, cv::Scalar(0, r.lmin, r.smin), cv::Scalar(r.hmax, 255, 255), red_img_2);
    binary_img = red_img_1 + red_img_2;
  } else if (detect_color == BULE) {
    cv::inRange(
      hsv_img, cv::Scalar(b.hmin, b.lmin, b.smin), cv::Scalar(b.hmax, 255, 255), binary_img);
  }

  return binary_img;
}

std::vector<Light> ArmorDetector::findLights(const cv::Mat & binary_img)
{
  using cv::Point;
  using cv::Vec4i;
  using std::vector;
  vector<vector<cv::Point>> contours;
  vector<Vec4i> hierarchy;
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  vector<Light> lights;
  this->debug_lights.data.clear();
  for (const auto & contour : contours) {
    if (contour.size() < 10) continue;
    auto r_rect = cv::minAreaRect(contour);

    if (isLight(r_rect)) {
      lights.emplace_back(Light(r_rect));
    }
  }

  return lights;
}

bool ArmorDetector::isLight(const cv::RotatedRect & rect)
{
  // TODO(chenjun): may need more judgement

  // The rotation angle in a clockwise direction.
  // The angle of rect returned by minAreaRect is in [0, 90]
  // If angle > 45, the width is longer than height
  float angle = rect.angle;

  // The ratio of light (short size / long size)
  float ratio = angle < 45 ? rect.size.aspectRatio() : 1.0 / rect.size.aspectRatio();
  bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio;

  angle = angle < 45 ? angle : 90 - angle;
  bool angle_ok = angle < l.max_angle || angle > (180.f - l.max_angle);

  bool is_light = ratio_ok && angle_ok;

  // Fill in debug information
  auto_aim_interfaces::msg::DebugLight light_data;
  light_data.ratio = ratio;
  light_data.angle = angle;
  light_data.is_light = is_light;
  this->debug_lights.data.emplace_back(light_data);

  return is_light;
}

std::vector<Armor> ArmorDetector::matchLights(const std::vector<Light> & lights)
{
  std::vector<Armor> armors;
  this->debug_armors.data.clear();

  // Loop all the pairing of lights
  for (auto light = lights.begin(); light != lights.end(); light++) {
    for (auto sec_light = light + 1; sec_light != lights.end(); sec_light++) {
      if (containLight(*light, *sec_light, lights)) {
        continue;
      }

      if (isArmor(*light, *sec_light)) {
        armors.emplace_back(Armor(*light, *sec_light));
      }
    }
  }

  return armors;
}

// Check if there is another light in the quadrilateral formed by the 2 lights
bool ArmorDetector::containLight(
  const Light & light_1, const Light & light_2, const std::vector<Light> & lights)
{
  // Y-axis is positive downward
  float top = std::min(light_1.top.y, light_2.top.y);
  float bottom = std::max(light_1.bottom.y, light_2.bottom.y);
  float left = std::min(light_1.center.x, light_2.center.x);
  float right = std::max(light_1.center.x, light_2.center.x);

  for (const auto & test_light : lights) {
    if (test_light.center == light_1.center || test_light.center == light_2.center) {
      continue;
    }

    if (
      (left < test_light.center.x && test_light.center.x < right) &&
      ((top < test_light.top.y && test_light.top.y < bottom) ||
       (top < test_light.center.y && test_light.center.y < bottom) ||
       (top < test_light.bottom.y && test_light.bottom.y < bottom))) {
      return true;
    }
  }

  return false;
}

bool ArmorDetector::isArmor(const Light & light_1, const Light & light_2)
{
  // Ratio of the length of 2 lights
  float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                             : light_2.length / light_1.length;
  bool light_ratio_ok = light_length_ratio > a.min_light_ratio;

  // Distance between the center of 2 lights
  float center_distance = cv::norm(light_1.center - light_2.center);
  float center_length_ratio = center_distance / (light_1.length + light_2.length);
  bool center_ratio_ok =
    a.min_center_ratio < center_length_ratio && center_length_ratio < a.max_center_ratio;

  // Angle of light center connection
  cv::Point2f diff = light_1.center - light_2.center;
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  bool angle_ok = angle < a.max_angle;

  bool is_armor = light_ratio_ok && center_ratio_ok && angle_ok;

  // Fill in debug information
  auto_aim_interfaces::msg::DebugArmor armor_data;
  armor_data.light_ratio = light_length_ratio;
  armor_data.center_ratio = center_length_ratio;
  armor_data.angle = angle;
  armor_data.is_armor = is_armor;
  this->debug_armors.data.emplace_back(armor_data);

  return is_armor;
}

}  // namespace rm_auto_aim
