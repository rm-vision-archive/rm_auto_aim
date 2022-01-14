#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rm_auto_aim
{

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class RmAutoAimNode : public rclcpp::Node
{
public:
  explicit RmAutoAimNode(const rclcpp::NodeOptions & options);
  virtual ~RmAutoAimNode();

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
};

}  // namespace rm_auto_aim
