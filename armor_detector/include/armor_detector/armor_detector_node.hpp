// Copyright 2022 Chen Jun

#ifndef ARMOR_DETECTOR__ARMOR_DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__ARMOR_DETECTOR_NODE_HPP_

// ROS
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// STD
#include <memory>

#include "armor_detector/armor_detector.hpp"

namespace rm_auto_aim
{
/*!
 * Main class for the node to handle the ROS interfacing.
 */
class ArmorDetectorNode : public rclcpp::Node
{
public:
  explicit ArmorDetectorNode(const rclcpp::NodeOptions & options);
  ~ArmorDetectorNode() override;

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr final_img_pub_;

  std::unique_ptr<ArmorDetector> detector_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_DETECTOR_NODE_HPP_
