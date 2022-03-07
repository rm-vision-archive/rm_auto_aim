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

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::RgbDetectorNode)
