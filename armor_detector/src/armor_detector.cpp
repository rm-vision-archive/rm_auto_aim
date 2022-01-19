// Copyright 2022 Chen Jun

#include "armor_detector/armor_detector.hpp"

#include <auto_aim_interfaces/msg/armor_data.hpp>
#include <auto_aim_interfaces/msg/light_data.hpp>

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

Armor::Armor(const Light & l1, const Light & l2)
{
  if (l1.center.x < l2.center.x) {
    left_light = l1, right_light = l2;
  } else {
    left_light = l2, right_light = l1;
  }
}

ArmorDetector::ArmorDetector(const DetectColor & color) : detect_color(color)
{
  // TODO(chenjun): Dynamic configure the following params
  r = {.hmin = 150, .hmax = 25, .lmin = 140, .smin = 100};
  b = {.hmin = 75, .hmax = 120, .lmin = 150, .smin = 160};

  l = {.min_ratio = 0.1f, .max_ratio = 0.4f, .max_angle = 40.f};

  a = {
    .min_light_ratio = 0.6f, .min_center_ratio = 0.4f, .max_center_ratio = 1.6f, .max_angle = 30};
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
  this->lights_data.data.clear();
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

  return lights;
}

bool ArmorDetector::isLight(const cv::RotatedRect & rect)
{
  // TODO(chenjun): may need more judgement
  // The ratio of light is about 1/4 (width / height)
  float ratio = rect.size.aspectRatio();
  bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio;

  // The angle of light is smaller than 30 degree.
  // The rotation angle in a clockwise direction.
  float angle = rect.angle;
  bool angle_ok = angle < l.max_angle || angle > (180.f - l.max_angle);

  bool is_light = ratio_ok && angle_ok;

  // Fill in debug information
  auto_aim_interfaces::msg::LightData light_data;
  light_data.ratio = ratio;
  light_data.angle = angle;
  light_data.is_light = is_light;
  this->lights_data.data.emplace_back(light_data);

  return is_light;
}

std::vector<Armor> ArmorDetector::matchLights(const std::vector<Light> & lights)
{
  std::vector<Armor> armors;
  this->armors_data.data.clear();

  // Loop all the pairing of lights
  for (auto light = lights.begin(); light != lights.end(); light++) {
    for (auto sec_light = light + 1; sec_light != lights.end(); sec_light++) {
      if (isArmor(*light, *sec_light)) {
        armors.emplace_back(Armor(*light, *sec_light));
      }
    }
  }

  return armors;
}

bool ArmorDetector::isArmor(const Light & light_1, const Light & light_2)
{
  // TODO(chenjun): need more judgement
  // FIXME(chenjun): this following data is only for test!!!
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
  auto_aim_interfaces::msg::ArmorData armor_data;
  armor_data.light_ratio = light_length_ratio;
  armor_data.center_ratio = center_length_ratio;
  armor_data.angle = angle;
  armor_data.is_armor = is_armor;
  this->armors_data.data.emplace_back(armor_data);

  return is_armor;
}

}  // namespace rm_auto_aim
