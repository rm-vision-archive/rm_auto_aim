// Copyright 2022 Chen Jun

#include "armor_detector/depth_processor.hpp"

namespace rm_auto_aim
{
//     [fx  0 cx]
// K = [ 0 fy cy]
//     [ 0  0  1]
DepthProcessor::DepthProcessor(const std::array<double, 9> & camera_matrix)
: fx_(camera_matrix[0]), fy_(camera_matrix[4]), cx_(camera_matrix[2]), cy_(camera_matrix[5])
{
}

geometry_msgs::msg::Point DepthProcessor::getPosition(
  const cv::Mat & depth_image, const cv::Point2f & image_point)
{
  auto depth = depth_image.at<cv::uint16_t>(image_point.y, image_point.x);

  geometry_msgs::msg::Point p;
  p.z = depth * 0.001;
  p.x = (image_point.x - cx_) * p.z / fx_;
  p.y = (image_point.y - cy_) * p.z / fy_;

  return p;
}

}  // namespace rm_auto_aim
