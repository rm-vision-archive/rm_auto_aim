// Copyright 2022 Chen Jun

#include "armor_detector/number_classifier.hpp"

#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// STL
#include <vector>

namespace rm_auto_aim
{
NumberClassifier::NumberClassifier(const double height_ratio, const double width_ratio)
: hr(height_ratio), wr(width_ratio)
{
}

std::vector<cv::Mat> NumberClassifier::extractNumbers(
  const cv::Mat & src, std::vector<Armor> & armors)
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
}  // namespace rm_auto_aim
