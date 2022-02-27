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
  std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });
  top = (p[0] + p[1]) / 2;
  bottom = (p[2] + p[3]) / 2;

  length = box.size.height > box.size.width ? box.size.height : box.size.width;

  tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
  tilt_angle = tilt_angle / CV_PI * 180;
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

ArmorDetector::ArmorDetector(
  const PreprocessParams & init_p, const LightParams & init_l, const ArmorParams & init_a,
  const Color & init_color)
: p(init_p), l(init_l), a(init_a), detect_color(init_color)
{
}

cv::Mat ArmorDetector::preprocessImage(const cv::Mat & rgb_img)
{
  cv::Mat hsv_img;
  cv::cvtColor(rgb_img, hsv_img, cv::COLOR_RGB2HLS);

  cv::Mat binary_img;
  cv::inRange(hsv_img, cv::Scalar(p.hmin, p.lmin, p.smin), cv::Scalar(180, 255, 255), binary_img);

  return binary_img;
}

std::vector<Light> ArmorDetector::findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img)
{
  using std::vector;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  vector<Light> lights;
  this->debug_lights.data.clear();

  for (const auto & contour : contours) {
    if (contour.size() < 5) continue;

    auto r_rect = cv::minAreaRect(contour);
    auto light = Light(r_rect);

    if (isLight(light)) {
      auto roi = light.boundingRect();
      if (
        0 <= roi.x && 0 <= roi.width && roi.x + roi.width <= rbg_img.cols && 0 <= roi.y &&
        0 <= roi.height && roi.y + roi.height <= rbg_img.rows) {
        auto output = rbg_img(roi);
        // Sum of red pixels > sum of blue pixels ?
        light.color = cv::sum(output)[0] > cv::sum(output)[2] ? Color::RED : Color::BULE;
        lights.emplace_back(light);
      }
    }
  }

  return lights;
}

bool ArmorDetector::isLight(const Light & light)
{
  // TODO(chenjun): may need more judgement
  // The ratio of light (short size / long size)
  float ratio = light.size.height < light.size.width ? light.size.height / light.size.width
                                                     : light.size.width / light.size.height;
  bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio;

  bool angle_ok = light.tilt_angle < l.max_angle;

  bool is_light = ratio_ok && angle_ok;

  // Fill in debug information
  auto_aim_interfaces::msg::DebugLight light_data;
  light_data.ratio = ratio;
  light_data.angle = light.tilt_angle;
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
    if (light->color != detect_color) continue;

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
