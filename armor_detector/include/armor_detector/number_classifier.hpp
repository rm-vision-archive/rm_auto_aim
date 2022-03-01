// Copyright 2022 Chen Jun

#ifndef ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_

#include <opencv2/core.hpp>

// STL
#include <vector>

#include "armor_detector/armor_detector.hpp"

namespace rm_auto_aim
{
class NumberClassifier
{
public:
  NumberClassifier(const double height_ratio, const double width_ratio);

  std::vector<cv::Mat> extractNumbers(const cv::Mat & src, std::vector<Armor> & armors);

  // height_ratio
  double hr;
  // width_ratio
  double wr;
};
}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
