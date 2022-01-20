// Copyright 2022 Chen Jun

#ifndef ARMOR_DETECTOR__DEPTH_PROCESSOR_HPP_
#define ARMOR_DETECTOR__DEPTH_PROCESSOR_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>

// STD
#include <array>

namespace rm_auto_aim
{
class DepthProcessor
{
public:
  explicit DepthProcessor(const std::array<double, 9> & camera_matrix);
  virtual ~DepthProcessor() = default;

  // Get 3d position
  geometry_msgs::msg::Point getPosition(
    const cv::Mat & depth_image, const cv::Point2f & image_point);

private:
  // Intrinsic camera matrix
  double fx_;
  double fy_;
  double cx_;
  double cy_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DEPTH_PROCESSOR_HPP_
