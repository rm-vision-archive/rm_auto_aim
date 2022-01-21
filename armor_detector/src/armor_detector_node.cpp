// Copyright 2022 Chen Jun

#include "armor_detector/armor_detector_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>

#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/qos.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;
namespace rm_auto_aim
{
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options)
: Node("ArmorDetectorNode", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting ArmorDetectorNode!");

  DetectColor default_color = this->declare_parameter("detect_color", 0) == 0 ? RED : BULE;
  std::string transport =
    this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";
  bool use_depth = this->declare_parameter("use_depth", true);
  debug_ = this->declare_parameter("debug", true);

  detector_ = std::make_unique<ArmorDetector>(default_color);

  if (use_depth) {
    // Init depth_processor
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/aligned_depth_to_color/camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
        depth_processor_ = std::make_unique<DepthProcessor>(camera_info->k);
        cam_info_sub_.reset();
      });

    // Synchronize color and depth image
    color_img_sub_filter_.subscribe(
      this, "/camera/color/image_raw", transport, rmw_qos_profile_sensor_data);
    // Use "raw" because https://github.com/ros-perception/image_common/issues/222
    depth_img_sub_filter_.subscribe(
      this, "/camera/aligned_depth_to_color/image_raw", "raw", rmw_qos_profile_sensor_data);
    sync_ = std::make_unique<ColorDepthSync>(color_img_sub_filter_, depth_img_sub_filter_, 5);
    sync_->registerCallback(std::bind(&ArmorDetectorNode::colorDepthCallback, this, _1, _2));

  } else {
    img_sub_ = image_transport::create_subscription(
      this, "/camera/color/image_raw", std::bind(&ArmorDetectorNode::imageCallback, this, _1),
      transport, rmw_qos_profile_sensor_data);
  }

  if (debug_) {
    lights_data_pub_ =
      this->create_publisher<auto_aim_interfaces::msg::DebugLights>("/debug/lights", 10);
    armors_data_pub_ =
      this->create_publisher<auto_aim_interfaces::msg::DebugArmors>("/debug/armors", 10);
    binary_img_pub_ = image_transport::create_publisher(this, "/debug/binary_img");
    final_img_pub_ = image_transport::create_publisher(this, "/debug/final_img");

    // Visualization marker
    marker_.ns = "armors";
    marker_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker_.action = visualization_msgs::msg::Marker::ADD;
    marker_.scale.x = marker_.scale.y = marker_.scale.z = 0.2;
    marker_.color.a = 1.0;
    marker_.color.r = 1.0;
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/debug/marker", 10);
  }
}

ArmorDetectorNode::~ArmorDetectorNode() = default;

void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  detectArmors(img_msg);
}

void ArmorDetectorNode::colorDepthCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr color_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr depth_msg)
{
  auto armors = detectArmors(color_msg);

  if (depth_processor_ != nullptr) {
    auto depth_img = cv_bridge::toCvShare(depth_msg, "mono16")->image;
    geometry_msgs::msg::Point p;

    marker_.header = color_msg->header;
    marker_.points.clear();

    for (const auto & armor : armors) {
      p = depth_processor_->getPosition(depth_img, armor.center);
      marker_.points.emplace_back(p);
    }

    marker_pub_->publish(marker_);
  }
}

std::vector<Armor> ArmorDetectorNode::detectArmors(
  const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  auto start_time = this->now();
  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
  cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

  auto binary_img = detector_->preprocessImage(img);
  auto lights = detector_->findLights(binary_img);
  auto armors = detector_->matchLights(lights);

  if (debug_) {
    auto final_time = this->now();
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "detectArmors used: " << (final_time - start_time).seconds() * 1000.0 << "ms");

    binary_img_pub_.publish(*cv_bridge::CvImage(img_msg->header, "mono8", binary_img).toImageMsg());

    lights_data_pub_->publish(detector_->debug_lights);
    armors_data_pub_->publish(detector_->debug_armors);

    drawLightsAndArmors(img, lights, armors);
    final_img_pub_.publish(*cv_bridge::CvImage(img_msg->header, "bgr8", img).toImageMsg());
  }
  return armors;
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
    cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
    cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
  }
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
