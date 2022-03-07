// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

// ROS
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/armor_detector.hpp"
#include "armor_detector/depth_processor.hpp"
#include "armor_detector/number_classifier.hpp"
#include "armor_detector/pnp_solver.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"

namespace rm_auto_aim
{
using SyncPolicy =
  message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
using ColorDepthSync = message_filters::Synchronizer<SyncPolicy>;

class BaseDetectorNode : public rclcpp::Node
{
public:
  BaseDetectorNode(const std::string & node_name, const rclcpp::NodeOptions & options);

protected:
  std::vector<Armor> detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);

  void publishMarkers();

  // Camera info subscription
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

  // Image subscriptions transport type
  std::string transport_;

  // Detected armors publisher
  auto_aim_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker text_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

private:
  std::unique_ptr<ArmorDetector> initArmorDetector();

  void createDebugPublishers();
  void destroyDebugPublishers();

  void drawLightsAndArmors(
    cv::Mat & img, const std::vector<Light> & lights, const std::vector<Armor> & armors);

  // Armor Detector
  std::unique_ptr<ArmorDetector> detector_;

  // Number Classifier
  std::unique_ptr<NumberClassifier> classifier_;

  // Debug information publishers
  bool debug_;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr binary_img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr final_img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr number_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr xor_pub_;
};

class RgbDetectorNode : public BaseDetectorNode
{
public:
  explicit RgbDetectorNode(const rclcpp::NodeOptions & options);

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);

  image_transport::Subscriber img_sub_;
  std::unique_ptr<PnPSolver> pnp_solver_;
};

class RgbDepthDetectorNode : public BaseDetectorNode
{
public:
  explicit RgbDepthDetectorNode(const rclcpp::NodeOptions & options);

private:
  void colorDepthCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & color_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg);

  image_transport::SubscriberFilter color_img_sub_filter_;
  image_transport::SubscriberFilter depth_img_sub_filter_;
  std::unique_ptr<ColorDepthSync> sync_;
  std::unique_ptr<DepthProcessor> depth_processor_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_
