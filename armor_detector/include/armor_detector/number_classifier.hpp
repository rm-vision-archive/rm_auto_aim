// Copyright 2022 Chen Jun

#ifndef ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_

// OpenCV
#include <opencv2/core.hpp>

// OpenVINO
#include <inference_engine.hpp>

// STL
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "armor_detector/armor_detector.hpp"

namespace rm_auto_aim
{
class NumberClassifier
{
public:
  NumberClassifier(
    const double height_ratio, const double width_ratio, const std::string & model_path);

  std::vector<cv::Mat> extractNumbers(const cv::Mat & src, const std::vector<Armor> & armors);

  void doClassify(const std::vector<cv::Mat> & numbers, std::vector<Armor> & armors);

  void prepareInput(const std::vector<cv::Mat> & imgs, InferenceEngine::Blob::Ptr & blob);

  void processOutput(const InferenceEngine::Blob::Ptr & blob, std::vector<Armor> & armors);

  // height_ratio
  double hr;
  // width_ratio
  double wr;

private:
  InferenceEngine::Core core_;
  InferenceEngine::CNNNetwork network_;
  InferenceEngine::ExecutableNetwork executable_network_;
  InferenceEngine::InferRequest infer_request_;

  std::string input_name_;
  std::string output_name_;

  size_t batch_size_;
  size_t class_num_;

  float confidence_threshold_;
};
}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
