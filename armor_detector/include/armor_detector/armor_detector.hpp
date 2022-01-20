// Copyright 2022 Chen Jun

#ifndef ARMOR_DETECTOR__ARMOR_DETECTOR_HPP_
#define ARMOR_DETECTOR__ARMOR_DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <vector>

#include "auto_aim_interfaces/msg/debug_armors.hpp"
#include "auto_aim_interfaces/msg/debug_lights.hpp"

namespace rm_auto_aim
{
enum DetectColor { RED, BULE };
class Light : public cv::RotatedRect
{
public:
  using RotatedRect::RotatedRect;
  explicit Light(cv::RotatedRect box);

  cv::Point2f top, bottom;
  float length;
};

struct Armor
{
  Armor(const Light & l1, const Light & l2);

  Light left_light, right_light;
  cv::Point2f center;
};

class ArmorDetector
{
public:
  explicit ArmorDetector(const DetectColor & color);

  DetectColor detect_color;

  struct PreprocessParams
  {
    double hmin, hmax, lmin, smin;
  } r, b;
  struct LightParams
  {
    // width / height
    float min_ratio;
    float max_ratio;
    // vertical angle
    float max_angle;
  } l;
  struct ArmorParams
  {
    float min_light_ratio;
    float min_center_ratio;
    float max_center_ratio;
    // horizontal angle
    float max_angle;
  } a;

  auto_aim_interfaces::msg::DebugLights debug_lights;
  auto_aim_interfaces::msg::DebugArmors debug_armors;

  cv::Mat preprocessImage(const cv::Mat & img);
  std::vector<Light> findLights(const cv::Mat & binary_img);
  std::vector<Armor> matchLights(const std::vector<Light> & lights);

private:
  bool isLight(const cv::RotatedRect & rect);
  bool containLight(
    const Light & light_1, const Light & light_2, const std::vector<Light> & lights);
  bool isArmor(const Light & light_1, const Light & light_2);
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_DETECTOR_HPP_
