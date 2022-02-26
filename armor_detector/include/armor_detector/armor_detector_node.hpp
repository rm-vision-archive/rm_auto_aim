// Copyright 2022 Chen Jun

#ifndef ARMOR_DETECTOR__ARMOR_DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__ARMOR_DETECTOR_NODE_HPP_

// ROS
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>

// STD
#include <memory>
#include <vector>

#include "armor_detector/armor_detector.hpp"
#include "armor_detector/depth_processor.hpp"
#include "armor_detector/pnp_solver.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"

namespace rm_auto_aim
{
using SyncPolicy =
  message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
using ColorDepthSync = message_filters::Synchronizer<SyncPolicy>;
/*!
 * Main class for the node to handle the ROS interfacing.
 */
class ArmorDetectorNode : public rclcpp::Node
{
public:
  explicit ArmorDetectorNode(const rclcpp::NodeOptions & options);
  ~ArmorDetectorNode() override;

private:
  std::unique_ptr<ArmorDetector> initArmorDetector();

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);

  void colorDepthCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & color_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg);

  std::vector<Armor> detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);

  void drawLightsAndArmors(
    cv::Mat & img, const std::vector<Light> & lights, const std::vector<Armor> & armors);

  void createDebugPublishers();
  void destroyDebugPublishers();

  std::unique_ptr<ArmorDetector> detector_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

  // Synchronize color and depth image if use depth
  image_transport::SubscriberFilter color_img_sub_filter_;
  image_transport::SubscriberFilter depth_img_sub_filter_;
  std::unique_ptr<ColorDepthSync> sync_;
  std::unique_ptr<DepthProcessor> depth_processor_;

  // Color image subscription if use only color image
  image_transport::Subscriber img_sub_;
  std::unique_ptr<PnPSolver> pnp_solver_;

  // Detected armors publisher
  auto_aim_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // Debug information publishers
  bool debug_;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
  image_transport::Publisher binary_img_pub_;
  image_transport::Publisher final_img_pub_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_DETECTOR_NODE_HPP_
