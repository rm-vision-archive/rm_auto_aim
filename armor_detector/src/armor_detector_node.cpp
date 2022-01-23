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

  // Detector
  detector_ = std::make_unique<ArmorDetector>(this);

  // Subscriptions
  bool use_depth = this->declare_parameter("use_depth", true);
  std::string transport =
    this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";
  if (use_depth) {
    // Init depth_processor
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/aligned_depth_to_color/camera_info", 10,
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
    sync_ = std::make_unique<ColorDepthSync>(
      SyncPolicy(10), color_img_sub_filter_, depth_img_sub_filter_);
    sync_->registerCallback(std::bind(&ArmorDetectorNode::colorDepthCallback, this, _1, _2));

  } else {
    img_sub_ = image_transport::create_subscription(
      this, "/camera/color/image_raw", std::bind(&ArmorDetectorNode::imageCallback, this, _1),
      transport, rmw_qos_profile_sensor_data);
  }

  // Armors Publisher
  armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
    "/detector/armors", rclcpp::SensorDataQoS());

  // Visualization Marker Publisher
  marker_.ns = "armors";
  marker_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker_.scale.x = marker_.scale.y = marker_.scale.z = 0.1;
  marker_.color.a = 1.0;
  marker_.color.r = 1.0;
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/detector/marker", 10);

  // Debug Publishers
  debug_ = this->declare_parameter("debug", true);
  if (debug_) {
    lights_data_pub_ =
      this->create_publisher<auto_aim_interfaces::msg::DebugLights>("/detector/debug/lights", 10);
    armors_data_pub_ =
      this->create_publisher<auto_aim_interfaces::msg::DebugArmors>("/detector/debug/armors", 10);
    binary_img_pub_ = image_transport::create_publisher(this, "/detector/debug/binary_img");
    final_img_pub_ = image_transport::create_publisher(this, "/detector/debug/final_img");
  }
}

ArmorDetectorNode::~ArmorDetectorNode() = default;

void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
{
  detectArmors(img_msg);
}

void ArmorDetectorNode::colorDepthCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & color_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
{
  auto armors = detectArmors(color_msg);

  if (depth_processor_ != nullptr) {
    auto depth_img = cv_bridge::toCvShare(depth_msg, "16UC1")->image;

    armors_msg_.header = depth_msg->header;
    armors_msg_.armors.clear();
    marker_.header = depth_msg->header;
    marker_.points.clear();

    auto_aim_interfaces::msg::Armor armor_msg;
    for (const auto & armor : armors) {
      armor_msg.position = depth_processor_->getPosition(depth_img, armor.center);
      armor_msg.distance_to_image_center =
        depth_processor_->calculateDistanceToCenter(armor.center);

      // If z < 0.4m, the depth would turn to zero
      if (armor_msg.position.z != 0) {
        armors_msg_.armors.emplace_back(armor_msg);
        marker_.points.emplace_back(armor_msg.position);
      }
    }

    // Publishing detected armors
    armors_pub_->publish(armors_msg_);

    // Publishing marker
    marker_.action = armors_msg_.armors.empty() ? visualization_msgs::msg::Marker::DELETE
                                                : visualization_msgs::msg::Marker::ADD;
    marker_pub_->publish(marker_);
  }
}

std::vector<Armor> ArmorDetectorNode::detectArmors(
  const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
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
