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
#include <memory>
#include <vector>

#include "rm_auto_aim/armor_detector.hpp"

std::unique_ptr<rm_auto_aim::ArmorDetector> DETECTOR;

cv::Mat ORIGIN_MAT;
cv::Mat R_BINARY;
cv::Mat B_BINARY;

std::vector<rm_auto_aim::Light> R_LIGHTS;
std::vector<rm_auto_aim::Light> B_LIGHTS;

std::vector<rm_auto_aim::Armor> R_ARMORS;
std::vector<rm_auto_aim::Armor> B_ARMORS;

TEST(ArmorDetectorTest, PreprocessTest)
{
  ORIGIN_MAT = cv::Mat(1080, 1280, CV_8UC3, cv::Scalar(1, 1, 1));

  // TODO(chenjun): only for testing locally
  // ORIGIN_MAT = cv::imread("/tmp/test.png");

  DETECTOR->detect_color = rm_auto_aim::ArmorDetector::RED;
  R_BINARY = DETECTOR->preprocessImage(ORIGIN_MAT);
  EXPECT_EQ(cv::imwrite("/tmp/test_r.png", R_BINARY), true);

  DETECTOR->detect_color = rm_auto_aim::ArmorDetector::BULE;
  B_BINARY = DETECTOR->preprocessImage(ORIGIN_MAT);
  EXPECT_EQ(cv::imwrite("/tmp/test_b.png", B_BINARY), true);
}

TEST(ArmorDetectorTest, FindLightsTest)
{
  R_LIGHTS = DETECTOR->findLights(R_BINARY);
  B_LIGHTS = DETECTOR->findLights(B_BINARY);

  // Draw lights
  for (const auto & light : R_LIGHTS) {
    cv::ellipse(ORIGIN_MAT, light, cv::Scalar(0, 128, 255), 2);
    cv::circle(ORIGIN_MAT, light.bottom, 2, cv::Scalar(0, 255, 0), -1);
  }
  for (const auto & light : B_LIGHTS) {
    cv::ellipse(ORIGIN_MAT, light, cv::Scalar(255, 0, 128), 2);
    cv::circle(ORIGIN_MAT, light.bottom, 2, cv::Scalar(0, 255, 0), -1);
  }
  EXPECT_EQ(cv::imwrite("/tmp/test_lights.png", ORIGIN_MAT), true);
}

TEST(ArmorDetectorTest, MatchLightsTest)
{
  R_ARMORS = DETECTOR->matchLights(R_LIGHTS);
  B_ARMORS = DETECTOR->matchLights(B_LIGHTS);

  // Draw armors
  for (const auto & armor : R_ARMORS) {
    cv::line(ORIGIN_MAT, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0));
    cv::line(ORIGIN_MAT, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0));
  }
  for (const auto & armor : B_ARMORS) {
    cv::line(ORIGIN_MAT, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0));
    cv::line(ORIGIN_MAT, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0));
  }
  EXPECT_EQ(cv::imwrite("/tmp/test_armors.png", ORIGIN_MAT), true);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = rclcpp::Node::make_shared("test_node", options);
  DETECTOR = std::make_unique<rm_auto_aim::ArmorDetector>(*node);
  return RUN_ALL_TESTS();
}
