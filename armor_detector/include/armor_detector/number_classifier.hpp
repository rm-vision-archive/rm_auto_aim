// Copyright 2022 Chen Jun

#ifndef ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include "armor_detector/armor_detector.hpp"

namespace rm_auto_aim
{
class NumberClassifier
{
public:
  NumberClassifier(
    const double & height_ratio, const double & width_ratio, const double & ct,
    const std::string & template_path);

  void extractNumbers(const cv::Mat & src, std::vector<Armor> & armors);

  void xorClassify(std::vector<Armor> & armors, cv::Mat & xor_all);

  // height_ratio
  double hr;
  // width_ratio
  double wr;
  double confidence_threshold;

private:
  std::map<char, cv::Mat> templates_;
};
}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
