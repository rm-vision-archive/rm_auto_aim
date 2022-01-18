// Copyright 2022 Chen Jun

#include "armor_detector/armor_detector_node.hpp"

#include <cv_bridge/cv_bridge.h>

#include <rclcpp/qos.hpp>

// std
#include <memory>
#include <string>
#include <vector>

namespace rm_auto_aim
{
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options)
: Node("ArmorDetectorNode", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting ArmorDetectorNode!");

  debug_ = this->declare_parameter("debug", true);

  detector_ = std::make_unique<ArmorDetector>();

  auto qos = debug_ ? rclcpp::QoS(10) : rclcpp::SensorDataQoS();
  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", qos,
    std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));

  if (debug_) {
    lights_data_pub_ =
      this->create_publisher<auto_aim_interfaces::msg::LightDataArray>("debug/lights", 10);
    armors_data_pub_ =
      this->create_publisher<auto_aim_interfaces::msg::ArmorDataArray>("debug/armors", 10);
    binary_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("debug/binary_image", 10);
    final_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("debug/final_image", 10);
  }
}

ArmorDetectorNode::~ArmorDetectorNode() = default;

void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  auto img = cv_bridge::toCvShare(msg, "rgb8")->image;

  auto start_time = this->now();

  // origin_img->preprocessImage->binary_img
  auto binary_img = detector_->preprocessImage(img);
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

  if (debug_) {
    // Publish debug information
    binary_img_pub_->publish(*cv_bridge::CvImage(msg->header, "mono8", binary_img).toImageMsg());

    lights_data_pub_->publish(detector_->lights_data);
    armors_data_pub_->publish(detector_->armors_data);

    drawLightsAndArmors(img, lights, armors);
    final_img_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg());
  }
}

void ArmorDetectorNode::drawLightsAndArmors(
  cv::Mat & img, const std::vector<Light> & lights, const std::vector<Armor> & armors)
{
  // Draw Lights
  auto color = detector_->detect_color == ArmorDetector::DetectColor::RED ? cv::Scalar(0, 128, 255)
                                                                          : cv::Scalar(255, 0, 128);
  for (const auto & light : lights) {
    cv::ellipse(img, light, color, 2);
  }

  // Draw armors
  for (const auto & armor : armors) {
    cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0));
    cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0));
  }
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
