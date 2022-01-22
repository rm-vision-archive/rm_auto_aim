// Copyright 2022 Chen Jun

#include "armor_processor/armor_processor_node.hpp"

#include <Eigen/src/Core/DiagonalMatrix.h>

// STD
#include <memory>
#include <vector>

namespace rm_auto_aim
{
ArmorProcessorNode::ArmorProcessorNode(const rclcpp::NodeOptions & options)
: Node("ArmorProcessorNode", options), dt_(0)
{
  RCLCPP_INFO(this->get_logger(), "Starting ArmorProcessorNode!");

  // Kalman Filter
  // TODO(chenjun): the params need to be changed
  // clang-format off
  A_ << 1,  0,  0, dt_, 0,  0,
        0,  1,  0,  0, dt_, 0,
        0,  0,  1,  0,  0, dt_,
        0,  0,  0,  1,  0,  0,
        0,  0,  0,  0,  1,  0,
        0,  0,  0,  0,  0,  1;

  H_.setIdentity();

  Q_ << 0.01,    0,    0,    0,    0,    0,
           0, 0.01,    0,    0,    0,    0,
           0,    0, 0.01,    0,    0,    0,
           0,    0,    0, 0.01,    0,    0,
           0,    0,    0,    0, 0.01,    0,
           0,    0,    0,    0,    0, 0.01;

  R_ << 0.05, 0,    0,
        0,   50,    0,
        0,    0,  0.1;

  P_.setIdentity();  // clang-format on
  kf_ = std::make_unique<KalmanFilter>(A_, H_, Q_, R_, P_);

  /// Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  armors_sub_.subscribe(this, "/detector/armors", rmw_qos_profile_sensor_data);
  tf2_filter_ = std::make_shared<tf2_filter>(
    armors_sub_, *tf2_buffer_, "shooter_link", 10, this->get_node_logging_interface(),
    this->get_node_clock_interface(), std::chrono::duration<int>(1));
  // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
  tf2_filter_->registerCallback(&ArmorProcessorNode::armorsCallback, this);

  // Publisher

  // Visualization Marker Publisher
  marker_.ns = "armors";
  marker_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker_.action = visualization_msgs::msg::Marker::ADD;
  marker_.scale.x = marker_.scale.y = marker_.scale.z = 0.1;
  marker_.color.a = 1.0;
  marker_.color.r = 1.0;
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/processor/marker", 10);

  // Debug Publishers
  debug_ = this->declare_parameter("debug", true);
  // if (debug_) {
  // }
}

void ArmorProcessorNode::armorsCallback(
  const auto_aim_interfaces::msg::Armors::SharedPtr armors_ptr)
{
  RCLCPP_INFO_STREAM(this->get_logger(), armors_ptr->armors.empty());
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorProcessorNode)
