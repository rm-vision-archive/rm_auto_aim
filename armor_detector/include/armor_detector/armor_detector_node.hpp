// Copyright 2022 Chen Jun

#ifndef ARMOR_DETECTOR__ARMOR_DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__ARMOR_DETECTOR_NODE_HPP_

// ROS
#include <message_filters/time_synchronizer.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// STD
#include <memory>
#include <vector>

#include "armor_detector/armor_detector.hpp"
#include "armor_detector/depth_processor.hpp"

namespace rm_auto_aim
{
using ColorDepthSync =
  message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
/*!
 * Main class for the node to handle the ROS interfacing.
 */
class ArmorDetectorNode : public rclcpp::Node
{
public:
  explicit ArmorDetectorNode(const rclcpp::NodeOptions & options);
  ~ArmorDetectorNode() override;

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
  void colorDepthCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr color_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr depth_msg);
  std::vector<Armor> detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
  void drawLightsAndArmors(
    cv::Mat & img, const std::vector<Light> & lights, const std::vector<Armor> & armors);

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  image_transport::Subscriber img_sub_;

  // Synchronize color and depth image if use depth
  image_transport::SubscriberFilter color_img_sub_filter_;
  image_transport::SubscriberFilter depth_img_sub_filter_;
  std::unique_ptr<ColorDepthSync> sync_;

  // Debug info publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
  image_transport::Publisher binary_img_pub_;
  image_transport::Publisher final_img_pub_;

  bool debug_;
  std::unique_ptr<ArmorDetector> detector_;
  std::unique_ptr<DepthProcessor> depth_processor_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_DETECTOR_NODE_HPP_
