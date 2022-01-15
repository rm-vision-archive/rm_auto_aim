// Copyright 2022 Chen Jun

#include <gtest/gtest.h>

// OpenCV
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>

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

  auto test_mat = cv::Mat(1080, 1280, CV_8UC3, cv::Scalar(1, 1, 1));
  // auto test_mat = cv::imread("/tmp/test.png");

  detector.detect_color = rm_auto_aim::ArmorDetector::RED;
  auto r_result = detector.preprocessImage(test_mat);
  EXPECT_EQ(cv::imwrite("/tmp/test_r.png", r_result), true);

  detector.detect_color = rm_auto_aim::ArmorDetector::BULE;
  auto b_result = detector.preprocessImage(test_mat);
  EXPECT_EQ(cv::imwrite("/tmp/test_b.png", b_result), true);
}
