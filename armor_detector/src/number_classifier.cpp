// Copyright 2022 Chen Jun
// Licensed under the MIT License.

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include "armor_detector/armor_detector.hpp"
#include "armor_detector/number_classifier.hpp"

namespace rm_auto_aim
{
NumberClassifier::NumberClassifier(
  const double & hf, const double & swf, const double & lwf, const std::map<char, double> & st,
  const std::string & template_path)
: height_factor(hf), small_width_factor(swf), large_width_factor(lwf), similarity_threshold(st)
{  
  small_armor_templates_ = {
    {'2', cv::imread(template_path + "2.png", cv::IMREAD_GRAYSCALE)},
    {'3', cv::imread(template_path + "3.png", cv::IMREAD_GRAYSCALE)},
    {'4', cv::imread(template_path + "4.png", cv::IMREAD_GRAYSCALE)},
    {'5', cv::imread(template_path + "5.png", cv::IMREAD_GRAYSCALE)},
    {'O', cv::imread(template_path + "outpost.png", cv::IMREAD_GRAYSCALE)},
  };
  large_armor_templates_ = {
    {'1', cv::imread(template_path + "1.png", cv::IMREAD_GRAYSCALE)},
    {'G', cv::imread(template_path + "guard.png", cv::IMREAD_GRAYSCALE)},
    {'B', cv::imread(template_path + "base.png", cv::IMREAD_GRAYSCALE)},
  };
}

void NumberClassifier::extractNumbers(const cv::Mat & src, std::vector<Armor> & armors)
{
  for (auto & armor : armors) {
    auto width_factor = armor.armor_type == LARGE ? large_width_factor : small_width_factor;

    // Scaling height
    auto left_height_diff = armor.left_light.bottom - armor.left_light.top;
    auto right_height_diff = armor.right_light.bottom - armor.right_light.top;

    auto left_center = (armor.left_light.top + armor.left_light.bottom) / 2;
    auto right_center = (armor.right_light.top + armor.right_light.bottom) / 2;

    auto top_left = left_center - left_height_diff / 2 * height_factor;
    auto top_right = right_center - right_height_diff / 2 * height_factor;
    auto bottom_left = left_center + left_height_diff / 2 * height_factor;
    auto bottom_right = right_center + right_height_diff / 2 * height_factor;

    // Scaling width
    auto top_width_diff = armor.right_light.top - armor.left_light.top;
    auto bottom_width_diff = armor.right_light.bottom - armor.left_light.bottom;

    auto top_center = (top_left + top_right) / 2;
    auto bottom_center = (bottom_left + bottom_right) / 2;

    top_left = top_center - top_width_diff / 2;
    top_right = top_center + top_width_diff / 2;
    bottom_left = bottom_center - bottom_width_diff / 2;
    bottom_right = bottom_center + bottom_width_diff / 2;

    cv::Point2f number_vertices[4] = {bottom_left, top_left, top_right, bottom_right};
    int output_width = 20 / width_factor;
    const auto output_size = cv::Size(output_width, 28);
    cv::Point2f target_vertices[4] = {
      cv::Point(0, output_size.height - 1),
      cv::Point(0, 0),
      cv::Point(output_size.width - 1, 0),
      cv::Point(output_size.width - 1, output_size.height - 1),
    };

    cv::Mat number_image;
    auto rotation_matrix = cv::getPerspectiveTransform(number_vertices, target_vertices);
    cv::warpPerspective(src, number_image, rotation_matrix, output_size);
    number_image = number_image(cv::Rect(output_width / 2 - 10, 0, 20, 28));

    cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);

    cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    armor.number_img = number_image;
  }
}

void NumberClassifier::fcClassify(std::vector<Armor> & armors)
{
  std::vector<std::string> class_names;
  std::ifstream ifs(
    std::string("/root/ros_ws/src/rm_pioneer_vision/rm_auto_aim/armor_detector/src/number.txt")
      .c_str());
  std::string line;
  while (getline(ifs, line)) {
    class_names.push_back(line);
  }
  // load model
  std::string model = "/root/ros_ws/model/number_classification13-r.onnx";
  cv::dnn::Net net = cv::dnn::readNetFromONNX(model);

  for (auto & armor : armors) {
    // load the image from disk
    cv::Mat image = armor.number_img;
    cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);
    cv::resize(image, image, cv::Size(28, 20));
    image = image / 255.0;
    // create blob from image
    cv::Mat blob;
    float scale = 1.0;
    cv::dnn::blobFromImage(image, blob, scale, cv::Size(28, 20), cv::Scalar(0), false, false);
    std::cout << "blob: " << blob.size << std::endl;

    // set the input blob for the neural network
    double t = cv::getTickCount();
    net.setInput(blob);
    // forward pass the image blob through the model
    cv::Mat outputs = net.forward();
    t = (cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << "Output shape: " << outputs.size() << ", Time-cost: " << t << std::endl;
    cv::Point class_id_point;
    double final_prob;
    std::cout << "outputs: " << outputs << std::endl;
    minMaxLoc(outputs, nullptr, &final_prob, nullptr, &class_id_point);
    int label_id = class_id_point.x;

    float max_prob = 0.0;
    float sum = 0.0;
    cv::Mat softmax_prob;
    max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
    cv::exp(outputs - max_prob, softmax_prob);
    sum = static_cast<float>(cv::sum(softmax_prob)[0]);
    softmax_prob /= sum;
    std::cout << "softmaxoutputs: " << softmax_prob << std::endl;

    minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &final_prob, nullptr, &class_id_point);
    label_id = class_id_point.x;

    if (final_prob > 0.8) {
      armor.confidence = final_prob;
      armor.number = *class_names[label_id].c_str();
    }

    std::stringstream result_ss;
    result_ss << armor.number << ":_" << std::fixed << std::setprecision(1)
              << armor.confidence * 100.0 << "%";
    armor.classfication_result = result_ss.str();

    //////////////// DEBUG
    std::cout << "number: " << armor.number << std::endl;
    std::cout << "confidence: " << armor.confidence << std::endl;
    std::cout << "classfication_result: " << armor.classfication_result << std::endl;

    armors.erase(
      std::remove_if(
        armors.begin(), armors.end(),
        [this](const Armor & armor) {
          return armor.confidence < similarity_threshold[armor.number];
        }),
      armors.end());
  }
}
}  // namespace rm_auto_aim
