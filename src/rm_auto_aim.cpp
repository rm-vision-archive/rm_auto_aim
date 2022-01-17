// Copyright 2022 Chen Jun

#include "rm_auto_aim/rm_auto_aim.hpp"

#include <cv_bridge/cv_bridge.h>

#include <memory>
#include <string>

namespace rm_auto_aim
{
RmAutoAimNode::RmAutoAimNode(const rclcpp::NodeOptions & options) : Node("RmAutoAimNode", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting RmAutoAimNode!");

  bool debug = this->declare_parameter("debug", true);

  detector_ = std::make_unique<ArmorDetector>(debug);

  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&RmAutoAimNode::imageCallback, this, std::placeholders::_1));
}

RmAutoAimNode::~RmAutoAimNode() = default;

void RmAutoAimNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  auto image = cv_bridge::toCvShare(msg, "rgb8")->image;

  auto start_time = this->now();

  // origin_img->preprocessImage->binary_img
  auto binary_img = detector_->preprocessImage(image);
  auto preprocess_time = this->now();
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "preprocessImage used: " << (preprocess_time - start_time).seconds() * 1000.0 << "ms");

  // binary_image->findLights->lights
  auto lights = detector_->findLights(binary_img);
  auto findlights_time = this->now();
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "findLights used: " << (findlights_time - preprocess_time).seconds() * 1000.0 << "ms");

  // lights->matchLights->armors
  auto armors = detector_->matchLights(lights);
  auto matchlights_time = this->now();
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "matchLights used: " << (matchlights_time - findlights_time).seconds() * 1000.0 << "ms");

  auto final_time = this->now();
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "detectArmors used: " << (final_time - start_time).seconds() * 1000.0 << "ms");
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::RmAutoAimNode)
