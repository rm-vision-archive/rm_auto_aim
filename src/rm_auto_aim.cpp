#include "rm_auto_aim/rm_auto_aim.hpp"

#include <cv_bridge/cv_bridge.h>

// STD
#include <string>

namespace rm_auto_aim
{

RmAutoAimNode::RmAutoAimNode(const rclcpp::NodeOptions & options) : Node("RmAutoAimNode", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting RmAutoAimNode!");

  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&RmAutoAimNode::imageCallback, this, std::placeholders::_1));
}

RmAutoAimNode::~RmAutoAimNode() = default;

void RmAutoAimNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  auto image = cv_bridge::toCvShare(msg, "rgb8")->image;
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::RmAutoAimNode)