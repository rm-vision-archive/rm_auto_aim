// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#include <cv_bridge/cv_bridge.h>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/detector_node.hpp"

using std::placeholders::_1;

namespace rm_auto_aim
{
RgbDetectorNode::RgbDetectorNode(const rclcpp::NodeOptions & options)
: BaseDetectorNode("rgb_detector", options)
{
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
      cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
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
    armors_msg_.header = position_marker_.header = text_marker_.header = img_msg->header;
    armors_msg_.armors.clear();
    marker_array_.markers.clear();
    position_marker_.points.clear();
    text_marker_.id = 0;

    auto_aim_interfaces::msg::Armor armor_msg;
    for (const auto & armor : armors) {
      // Fill the armor msg
      geometry_msgs::msg::Point position;
      bool success = pnp_solver_->solvePnP(armor, position);

      if (success) {
        armor_msg.position = position;
        armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor.center);

        armors_msg_.armors.emplace_back(armor_msg);
        position_marker_.points.emplace_back(armor_msg.position);

        text_marker_.id++;
        text_marker_.pose.position = armor_msg.position;
        text_marker_.pose.position.y -= 0.1;
        text_marker_.text = armor.classfication_result;
        marker_array_.markers.emplace_back(text_marker_);
      } else {
        RCLCPP_WARN(this->get_logger(), "PnP failed!");
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
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::RgbDetectorNode)
