// Copyright 2022 Chen Jun

#include "armor_processor/armor_processor_node.hpp"

// STD
#include <memory>
#include <string>
#include <vector>

namespace rm_auto_aim
{
ArmorProcessorNode::ArmorProcessorNode(const rclcpp::NodeOptions & options)
: Node("ArmorProcessorNode", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting ArmorProcessorNode!");

  // Subscriptions

  // Publisher

  // Debug Publishers
  debug_ = this->declare_parameter("debug", true);
  // if (debug_) {
  // }
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorProcessorNode)
