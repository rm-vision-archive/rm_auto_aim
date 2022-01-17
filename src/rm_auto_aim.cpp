// Copyright 2022 Chen Jun

#include "rm_auto_aim/rm_auto_aim.hpp"

#include <cv_bridge/cv_bridge.h>

#include <rclcpp/qos.hpp>

// std
#include <memory>
#include <string>

namespace rm_auto_aim
{
RmAutoAimNode::RmAutoAimNode(const rclcpp::NodeOptions & options) : Node("RmAutoAimNode", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting RmAutoAimNode!");

  bool debug = this->declare_parameter("debug", true);

  detector_ = std::make_unique<ArmorDetector>(debug);

  auto qos = debug ? rclcpp::QoS(10) : rclcpp::SensorDataQoS();
  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", qos,
    std::bind(&RmAutoAimNode::imageCallback, this, std::placeholders::_1));

  final_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("debug/final_image", 10);
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

  // TODO(chenjun): test
  // Draw lights
  auto color = detector_->detect_color == ArmorDetector::DetectColor::RED ? cv::Scalar(0, 128, 255)
                                                                          : cv::Scalar(255, 0, 128);
  for (const auto & light : lights) {
    cv::ellipse(image, light, color, 2);
  }
  // Draw armors
  for (const auto & armor : armors) {
    cv::line(image, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0));
    cv::line(image, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0));
  }
  auto final_image_msg = cv_bridge::CvImage(msg->header, "bgr8", image).toImageMsg();
  final_img_pub_->publish(*final_image_msg);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::RmAutoAimNode)
