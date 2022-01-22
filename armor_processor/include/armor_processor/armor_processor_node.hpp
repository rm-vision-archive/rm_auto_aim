// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__ARMOR_PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__ARMOR_PROCESSOR_NODE_HPP_

// ROS
#include <rclcpp/rclcpp.hpp>

// STD
#include <memory>
#include <vector>

namespace rm_auto_aim
{
/*!
 * Main class for the node to handle the ROS interfacing.
 */
class ArmorProcessorNode : public rclcpp::Node
{
public:
  explicit ArmorProcessorNode(const rclcpp::NodeOptions & options);
  ~ArmorProcessorNode() override;

private:
  bool debug_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__ARMOR_PROCESSOR_NODE_HPP_
