// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#include <cv_bridge/cv_bridge.h>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/detector_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace rm_auto_aim
{
RgbDepthDetectorNode::RgbDepthDetectorNode(const rclcpp::NodeOptions & options)
: BaseDetectorNode("rgb_depth_detector", options)
{
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/aligned_depth_to_color/camera_info", 10,
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
      cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
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

    armors_msg_.header = position_marker_.header = text_marker_.header = depth_msg->header;
    armors_msg_.armors.clear();
    marker_array_.markers.clear();
    position_marker_.points.clear();
    text_marker_.id = 0;

    auto_aim_interfaces::msg::Armor armor_msg;
    for (const auto & armor : armors) {
      // Fill the armor msg
      armor_msg.number = armor.number;
      armor_msg.position = depth_processor_->getPosition(depth_img, armor.center);
      armor_msg.distance_to_image_center =
        depth_processor_->calculateDistanceToCenter(armor.center);

      // If z < 0.4m, the depth would turn to zero
      if (armor_msg.position.z != 0) {
        armors_msg_.armors.emplace_back(armor_msg);
        position_marker_.points.emplace_back(armor_msg.position);

        text_marker_.id++;
        text_marker_.pose.position = armor_msg.position;
        text_marker_.pose.position.y -= 0.1;
        text_marker_.text = armor.classfication_result;
        marker_array_.markers.emplace_back(text_marker_);
      }
    }

    // Publishing detected armors
    armors_pub_->publish(armors_msg_);

    // Publishing marker
    publishMarkers();
  }
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::RgbDepthDetectorNode)
