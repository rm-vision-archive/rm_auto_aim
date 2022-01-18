// Copyright 2022 Chen Jun

#ifndef ARMOR_DETECTOR__ARMOR_DETECTOR_HPP_
#define ARMOR_DETECTOR__ARMOR_DETECTOR_HPP_

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <rclcpp/node.hpp>

// STD
#include <cmath>
#include <vector>

namespace rm_auto_aim
{
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
};

class ArmorDetector
{
public:
  explicit ArmorDetector(const bool & debug);

  enum DetectColor { RED, BULE } detect_color;
  struct PreprocessParams
  {
    double hmin, hmax, vmin, vmax;
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

  cv::Mat preprocessImage(const cv::Mat & img);
  std::vector<Light> findLights(const cv::Mat & binary_img);
  std::vector<Armor> matchLights(const std::vector<Light> & lights);

private:
  bool isLight(const cv::RotatedRect & rect);
  bool isArmor(const Light & light_1, const Light & light_2);

  bool debug_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_DETECTOR_HPP_
