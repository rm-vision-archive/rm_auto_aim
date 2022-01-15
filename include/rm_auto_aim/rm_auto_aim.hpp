// Copyright 2022 Chen Jun

#ifndef RM_AUTO_AIM__RM_AUTO_AIM_HPP_
#define RM_AUTO_AIM__RM_AUTO_AIM_HPP_

// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// STD
#include <memory>

#include "rm_auto_aim/armor_detector.hpp"

namespace rm_auto_aim
{
/*!
 * Main class for the node to handle the ROS interfacing.
 */
class RmAutoAimNode : public rclcpp::Node
{
public:
  explicit RmAutoAimNode(const rclcpp::NodeOptions & options);
  ~RmAutoAimNode() override;

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  std::unique_ptr<ArmorDetector> detector_;
};

}  // namespace rm_auto_aim

#endif  // RM_AUTO_AIM__RM_AUTO_AIM_HPP_
