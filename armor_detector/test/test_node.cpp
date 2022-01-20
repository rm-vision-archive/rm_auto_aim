// Copyright 2022 Chen Jun

#include <gtest/gtest.h>

#include <rclcpp/executors.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>

// STD
#include <memory>

#include "armor_detector/armor_detector_node.hpp"

TEST(ArmorDetectorNodeTest, NodeTest)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rm_auto_aim::ArmorDetectorNode>(options);
  rclcpp::shutdown();
}
