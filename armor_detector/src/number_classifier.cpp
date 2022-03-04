// Copyright 2022 Chen Jun

#include "armor_detector/number_classifier.hpp"

// OpenCV
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <map>
#include <string>
#include <vector>

namespace rm_auto_aim
{
NumberClassifier::NumberClassifier(
  const double height_ratio, const double width_ratio, const std::string & model_path)
: hr(height_ratio), wr(width_ratio), class_num_(6)
{
  const int dynamic_batch_limit = 100;

  network_ = core_.ReadNetwork(model_path);

  auto input_info = network_.getInputsInfo();
  input_name_ = input_info.begin()->first;

  auto output_info = network_.getOutputsInfo();
  output_name_ = output_info.begin()->first;

  const std::map<std::string, std::string> dyn_config = {
    {InferenceEngine::PluginConfigParams::KEY_DYN_BATCH_ENABLED,
     InferenceEngine::PluginConfigParams::YES}};
  network_.setBatchSize(dynamic_batch_limit);

  executable_network_ = core_.LoadNetwork(network_, "CPU", dyn_config);
  infer_request_ = executable_network_.CreateInferRequest();
}

std::vector<cv::Mat> NumberClassifier::extractNumbers(
  const cv::Mat & src, const std::vector<Armor> & armors)
{
  std::vector<cv::Mat> numbers;
  for (const auto & armor : armors) {
    auto top_width_diff = armor.right_light.top - armor.left_light.top;
    auto bottom_width_diff = armor.right_light.bottom - armor.left_light.bottom;
    auto left_height_diff = armor.left_light.bottom - armor.left_light.top;
    auto right_height_diff = armor.right_light.bottom - armor.right_light.top;

    // The order is bottomLeft, topLeft, topRight, bottomRight
    cv::Point2f number_vertices[4] = {
      armor.center + left_height_diff * hr - bottom_width_diff * wr,
      armor.center - left_height_diff * hr - top_width_diff * wr,
      armor.center - right_height_diff * hr + top_width_diff * wr,
      armor.center + right_height_diff * hr + bottom_width_diff * wr};

    const auto output_size = cv::Size(32, 32);
    cv::Point2f target_vertices[4] = {
      cv::Point(0, output_size.height - 1),
      cv::Point(0, 0),
      cv::Point(output_size.width - 1, 0),
      cv::Point(output_size.width - 1, output_size.height - 1),
    };

    auto rotation_matrix = cv::getPerspectiveTransform(number_vertices, target_vertices);

    cv::Mat number_image;
    cv::warpPerspective(src, number_image, rotation_matrix, output_size);

    cv::Mat gray_number_image;
    cv::cvtColor(number_image, gray_number_image, cv::COLOR_RGB2GRAY);
    numbers.emplace_back(gray_number_image);
  }

  return numbers;
}

void NumberClassifier::doClassify(const std::vector<cv::Mat> & numbers, std::vector<Armor> & armors)
{
  batch_size_ = numbers.size();
  infer_request_.SetBatch(batch_size_);

  auto input_blob = infer_request_.GetBlob(input_name_);

  prepareInput(numbers, input_blob);

  infer_request_.Infer();

  auto output_blob = infer_request_.GetBlob(output_name_);

  processOutput(output_blob, armors);
}

void NumberClassifier::prepareInput(
  const std::vector<cv::Mat> & imgs, InferenceEngine::Blob::Ptr & blob)
{
  size_t channels = 1;
  size_t img_w = imgs[0].cols;
  size_t img_h = imgs[0].rows;

  auto input_blob = InferenceEngine::as<InferenceEngine::MemoryBlob>(blob);
  auto input_blob_holder = input_blob->wmap();
  float * blob_data = input_blob_holder.as<float *>();

  for (size_t batch_id = 0; batch_id < batch_size_; ++batch_id) {
    auto img = imgs[batch_id];
    auto img_data = img.data;
    for (size_t h = 0; h < img_h; ++h) {
      for (size_t w = 0; w < img_w; ++w) {
        blob_data[batch_id * channels * img_h * img_w + h * img_w + w] =
          static_cast<float>(img_data[h * img_w + w]);
      }
    }
  }
}

void NumberClassifier::processOutput(
  const InferenceEngine::Blob::Ptr & blob, std::vector<Armor> & armors)
{
  auto output_blob = InferenceEngine::as<InferenceEngine::MemoryBlob>(blob);
  auto output_blob_holder = output_blob->rmap();
  const float * output_blob_data = output_blob_holder.as<const float *>();

  float max_confidence = 0.0;
  for (size_t batch_id = 0; batch_id < batch_size_; ++batch_id) {
    auto & armor = armors[batch_id];
    for (size_t class_id = 0; class_id < class_num_; ++class_id) {
      auto confidence = output_blob_data[batch_id * class_num_ + class_id];
      if (confidence > max_confidence) {
        max_confidence = confidence;
        armor.number = class_id;
      }
    }
    armor.confidence = max_confidence;
  }
}

}  // namespace rm_auto_aim
