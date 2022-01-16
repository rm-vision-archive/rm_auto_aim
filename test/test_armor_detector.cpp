// Copyright 2022 Chen Jun

#include <gtest/gtest.h>

// OpenCV
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

// ROS
#include <rclcpp/node.hpp>

// STD
#include <cstdlib>

#include "rm_auto_aim/armor_detector.hpp"

cv::Mat ORIGIN_MAT;
cv::Mat R_BINARY;
cv::Mat B_BINARY;

TEST(ArmorDetectorTest, PreprocessTest)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  auto node = rclcpp::Node::make_shared("test_node", options);
  auto detector = rm_auto_aim::ArmorDetector(*node);

  ORIGIN_MAT = cv::Mat(1080, 1280, CV_8UC3, cv::Scalar(1, 1, 1));
  // ORIGIN_MAT = cv::imread("/tmp/test.png");

  detector.detect_color = rm_auto_aim::ArmorDetector::RED;
  R_BINARY = detector.preprocessImage(ORIGIN_MAT);
  EXPECT_EQ(cv::imwrite("/tmp/test_r.png", R_BINARY), true);

  detector.detect_color = rm_auto_aim::ArmorDetector::BULE;
  B_BINARY = detector.preprocessImage(ORIGIN_MAT);
  EXPECT_EQ(cv::imwrite("/tmp/test_b.png", B_BINARY), true);
}

TEST(ArmorDetectorTest, FindLightsTest)
{
  rclcpp::NodeOptions options;
  auto node = rclcpp::Node::make_shared("test_node", options);
  auto detector = rm_auto_aim::ArmorDetector(*node);

  auto r_lights = detector.findLights(R_BINARY);
  auto b_lights = detector.findLights(B_BINARY);

  // draw lights
  for (const auto & light : r_lights) {
    cv::ellipse(ORIGIN_MAT, light, cv::Scalar(0, 128, 255), 2);
    cv::circle(ORIGIN_MAT, light.bottom, 2, cv::Scalar(0, 255, 0), -1);
  }
  for (const auto & light : b_lights) {
    cv::ellipse(ORIGIN_MAT, light, cv::Scalar(255, 0, 128), 2);
    cv::circle(ORIGIN_MAT, light.bottom, 2, cv::Scalar(0, 255, 0), -1);
  }
  EXPECT_EQ(cv::imwrite("/tmp/test_lights.png", ORIGIN_MAT), true);
}
