// Copyright 2022 Chen Jun

#include "armor_detector/armor_detector_node.hpp"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc.hpp>
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

  DetectColor default_color = this->declare_parameter("detect_color", 0) == 0 ? RED : BULE;
  std::string transport =
    this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";
  debug_ = this->declare_parameter("debug", true);

  detector_ = std::make_unique<ArmorDetector>(default_color);

  img_sub_ = image_transport::create_subscription(
    this, "/camera/color/image_raw",
    std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1), transport,
    rmw_qos_profile_sensor_data);

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
  cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

  auto start_time = this->now();

  auto binary_img = detector_->preprocessImage(img);
  auto preprocess_time = this->now();
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "preprocessImage used: " << (preprocess_time - start_time).seconds() * 1000.0 << "ms");

  auto lights = detector_->findLights(binary_img);

  auto armors = detector_->matchLights(lights);

  if (debug_) {
    auto final_time = this->now();
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "detectArmors used: " << (final_time - start_time).seconds() * 1000.0 << "ms");

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
  for (const auto & light : lights) {
    cv::ellipse(img, light, cv::Scalar(255, 0, 255), 2);
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
