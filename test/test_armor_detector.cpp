// Copyright 2022 Chen Jun

#include <gtest/gtest.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <rclcpp/node.hpp>

#include "rm_auto_aim/armor_detector.hpp"

TEST(ArmorDetectorTest, PreprocessTest)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  auto node = rclcpp::Node::make_shared("test_node", options);
  auto detector = rm_auto_aim::ArmorDetector(*node);

  int loop_count = 0;
  cv::Mat test_mat(1280, 1080, CV_8UC3, cv::Scalar(1, 1, 1));
  while (rclcpp::ok() && loop_count < 50) {
    detector.preprocessImage(test_mat);
    loop_count++;
  }
}
