// Copyright 2022 Chen Jun

#include "armor_detector/detector_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/qos.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/armor_detector.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
namespace rm_auto_aim
{
BaseDetectorNode::BaseDetectorNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");

  // Detector
  detector_ = initArmorDetector();

  // Number classifier
  double height_ratio = this->declare_parameter("number_classifier.height_ratio", 1.0);
  double weight_ratio = this->declare_parameter("number_classifier.weight_ratio", 0.28);
  double confidence_threshold =
    this->declare_parameter("number_classifier.confidence_threshold", 0.5);
  auto template_path =
    ament_index_cpp::get_package_share_directory("armor_detector") + "/template/";
  classifier_ = std::make_unique<NumberClassifier>(
    height_ratio, weight_ratio, confidence_threshold, template_path);

  // Subscriptions transport type
  transport_ = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";

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
  debug_ = this->declare_parameter("debug", false);
  if (debug_) {
    createDebugPublishers();
  }

  // Debug param change moniter
  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  debug_cb_handle_ =
    debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter & p) {
      debug_ = p.as_bool();
      debug_ ? createDebugPublishers() : destroyDebugPublishers();
    });
}

std::unique_ptr<ArmorDetector> BaseDetectorNode::initArmorDetector()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 255;
  int min_lightness = declare_parameter("min_lightness", 160, param_desc);

  param_desc.description = "0-RED, 1-BLUE";
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 1;
  auto detect_color = static_cast<Color>(declare_parameter("detect_color", 0, param_desc));

  ArmorDetector::LightParams l_params = {
    .min_ratio = declare_parameter("light.min_ratio", 0.1),
    .max_ratio = declare_parameter("light.max_ratio", 0.55),
    .max_angle = declare_parameter("light.max_angle", 40.0)};

  ArmorDetector::ArmorParams a_params = {
    .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.6),
    .min_center_ratio = declare_parameter("armor.min_center_ratio", 0.4),
    .max_center_ratio = declare_parameter("armor.max_center_ratio", 1.6),
    .max_angle = declare_parameter("armor.max_angle", 35.0)};

  return std::make_unique<ArmorDetector>(min_lightness, detect_color, l_params, a_params);
}

std::vector<Armor> BaseDetectorNode::detectArmors(
  const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
{
  auto start_time = this->now();
  // Convert ROS img to cv::Mat
  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

  // Detect armors
  detector_->min_lightness = get_parameter("min_lightness").as_int();
  detector_->detect_color = static_cast<Color>(get_parameter("detect_color").as_int());
  auto binary_img = detector_->preprocessImage(img);
  auto lights = detector_->findLights(img, binary_img);
  auto armors = detector_->matchLights(lights);

  // Extract numbers
  classifier_->hr = get_parameter("number_classifier.height_ratio").as_double();
  classifier_->wr = get_parameter("number_classifier.weight_ratio").as_double();
  classifier_->confidence_threshold =
    get_parameter("number_classifier.confidence_threshold").as_double();
  cv::Mat xor_img;
  if (!armors.empty()) {
    classifier_->extractNumbers(img, armors);
    classifier_->xorClassify(armors, xor_img);
  }

  // Publish debug info
  if (debug_) {
    auto final_time = this->now();
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "detectArmors used: " << (final_time - start_time).seconds() * 1000.0 << "ms");

    binary_img_pub_->publish(
      *cv_bridge::CvImage(img_msg->header, "mono8", binary_img).toImageMsg());

    lights_data_pub_->publish(detector_->debug_lights);
    armors_data_pub_->publish(detector_->debug_armors);

    if (!armors.empty()) {
      // Combine all number images to one
      std::vector<cv::Mat> number_imgs;
      number_imgs.reserve(armors.size());
      for (const auto & armor : armors) {
        number_imgs.emplace_back(armor.number_img);
      }
      cv::Mat all_num_img;
      cv::vconcat(number_imgs, all_num_img);

      number_pub_->publish(*cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
      xor_pub_->publish(*cv_bridge::CvImage(img_msg->header, "mono8", xor_img).toImageMsg());
    }

    drawLightsAndArmors(img, lights, armors);
    final_img_pub_->publish(*cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
  }

  return armors;
}

void BaseDetectorNode::drawLightsAndArmors(
  cv::Mat & img, const std::vector<Light> & lights, const std::vector<Armor> & armors)
{
  // Draw Lights
  for (const auto & light : lights) {
    auto color = light.color == RED ? cv::Scalar(255, 255, 0) : cv::Scalar(255, 0, 255);
    cv::ellipse(img, light, color, 2);
  }

  // Draw armors
  for (const auto & armor : armors) {
    cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
    cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
  }

  // Show numbers and confidence
  for (const auto & armor : armors) {
    std::stringstream text_ss;
    text_ss << armor.number << ": " << std::fixed << std::setprecision(1)
            << armor.similarity * 100.0 << "%";
    cv::putText(
      img, text_ss.str(), armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8,
      cv::Scalar(0, 255, 255), 2);
  }
}

void BaseDetectorNode::createDebugPublishers()
{
  lights_data_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::DebugLights>("/detector/debug/lights", 10);
  armors_data_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::DebugArmors>("/detector/debug/armors", 10);
  binary_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/detector/debug/bin_img", 10);
  final_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/detector/debug/final_img", 10);
  number_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/detector/debug/number", 10);
  xor_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/detector/debug/xor", 10);
}

void BaseDetectorNode::destroyDebugPublishers()
{
  lights_data_pub_.reset();
  armors_data_pub_.reset();
  binary_img_pub_.reset();
  final_img_pub_.reset();
  number_pub_.reset();
  xor_pub_.reset();
}

RgbDetectorNode::RgbDetectorNode(const rclcpp::NodeOptions & options)
: BaseDetectorNode("rgb_detector", options)
{
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
      cam_info_sub_.reset();
    });

  img_sub_ = image_transport::create_subscription(
    this, "/image_raw", std::bind(&RgbDetectorNode::imageCallback, this, _1), transport_,
    rmw_qos_profile_sensor_data);
}

void RgbDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
{
  auto armors = detectArmors(img_msg);

  if (pnp_solver_ != nullptr) {
    armors_msg_.header = img_msg->header;
    armors_msg_.armors.clear();
    marker_.header = img_msg->header;
    marker_.points.clear();

    auto_aim_interfaces::msg::Armor armor_msg;
    armor_msg.position_stamped.header = img_msg->header;
    for (const auto & armor : armors) {
      // Fill the armor msg
      geometry_msgs::msg::Point point;
      bool success = pnp_solver_->solvePnP(armor, point);

      if (success) {
        armor_msg.position_stamped.point = point;
        armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor.center);

        armors_msg_.armors.emplace_back(armor_msg);
        marker_.points.emplace_back(armor_msg.position_stamped.point);
      } else {
        RCLCPP_WARN(this->get_logger(), "PnP failed!");
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

RgbDepthDetectorNode::RgbDepthDetectorNode(const rclcpp::NodeOptions & options)
: BaseDetectorNode("rgb_depth_detector", options)
{
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/aligned_depth_to_color/camera_info", 10,
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      depth_processor_ = std::make_unique<DepthProcessor>(camera_info->k);
      cam_info_sub_.reset();
    });

  // Synchronize color and depth image
  color_img_sub_filter_.subscribe(
    this, "/camera/color/image_raw", transport_, rmw_qos_profile_sensor_data);
  // Use "raw" because https://github.com/ros-perception/image_common/issues/222
  depth_img_sub_filter_.subscribe(
    this, "/camera/aligned_depth_to_color/image_raw", "raw", rmw_qos_profile_sensor_data);
  sync_ =
    std::make_unique<ColorDepthSync>(SyncPolicy(10), color_img_sub_filter_, depth_img_sub_filter_);
  sync_->registerCallback(std::bind(&RgbDepthDetectorNode::colorDepthCallback, this, _1, _2));
}

void RgbDepthDetectorNode::colorDepthCallback(
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
    armor_msg.position_stamped.header = depth_msg->header;
    for (const auto & armor : armors) {
      // Fill the armor msg
      armor_msg.position_stamped.point = depth_processor_->getPosition(depth_img, armor.center);
      armor_msg.distance_to_image_center =
        depth_processor_->calculateDistanceToCenter(armor.center);

      // If z < 0.4m, the depth would turn to zero
      if (armor_msg.position_stamped.point.z != 0) {
        armors_msg_.armors.emplace_back(armor_msg);
        marker_.points.emplace_back(armor_msg.position_stamped.point);
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

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::RgbDetectorNode)
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::RgbDepthDetectorNode)
