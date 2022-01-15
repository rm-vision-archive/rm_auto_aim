// Copyright 2022 Chen Jun

#include <gtest/gtest.h>

// OpenCV
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

// ROS
#include <rclcpp/node.hpp>

// STD
#include <cstdlib>

#include "rm_auto_aim/armor_detector.hpp"

TEST(ArmorDetectorTest, PreprocessTest)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  auto node = rclcpp::Node::make_shared("test_node", options);
  auto detector = rm_auto_aim::ArmorDetector(*node);

  cv::Mat test_mat(1280, 1080, CV_8UC3, cv::Scalar(1, 1, 1));
  detector.preprocessImage(test_mat);
}
