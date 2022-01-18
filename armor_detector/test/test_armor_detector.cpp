// Copyright 2022 Chen Jun

#include <gtest/gtest.h>

// OpenCV
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <cstdlib>
#include <memory>
#include <vector>

#include "armor_detector/armor_detector.hpp"

std::unique_ptr<rm_auto_aim::ArmorDetector> DETECTOR;

cv::Mat IMAGE;
cv::Mat R_BINARY;
cv::Mat B_BINARY;

std::vector<rm_auto_aim::Light> R_LIGHTS;
std::vector<rm_auto_aim::Light> B_LIGHTS;

std::vector<rm_auto_aim::Armor> R_ARMORS;
std::vector<rm_auto_aim::Armor> B_ARMORS;

TEST(ArmorDetectorTest, PreprocessTest)
{
  IMAGE = cv::Mat(1080, 1280, CV_8UC3, cv::Scalar(1, 1, 1));

  // XXX(chenjun): this line only for testing locally
  // IMAGE = cv::imread("/tmp/rs_r_b.png");

  DETECTOR->detect_color = rm_auto_aim::ArmorDetector::RED;
  R_BINARY = DETECTOR->preprocessImage(IMAGE);
  EXPECT_EQ(cv::imwrite("/tmp/test_r.png", R_BINARY), true);

  DETECTOR->detect_color = rm_auto_aim::ArmorDetector::BULE;
  B_BINARY = DETECTOR->preprocessImage(IMAGE);
  EXPECT_EQ(cv::imwrite("/tmp/test_b.png", B_BINARY), true);
}

TEST(ArmorDetectorTest, FindLightsTest)
{
  R_LIGHTS = DETECTOR->findLights(R_BINARY);
  B_LIGHTS = DETECTOR->findLights(B_BINARY);

  // Draw lights
  for (const auto & light : R_LIGHTS) {
    cv::ellipse(IMAGE, light, cv::Scalar(0, 128, 255), 2);
    // Draw bottom point of the light
    // cv::circle(ORIGIN_MAT, light.bottom, 2, cv::Scalar(0, 255, 0), -1);
  }
  for (const auto & light : B_LIGHTS) {
    cv::ellipse(IMAGE, light, cv::Scalar(255, 0, 128), 2);
    // cv::circle(ORIGIN_MAT, light.bottom, 2, cv::Scalar(0, 255, 0), -1);
  }
  EXPECT_EQ(cv::imwrite("/tmp/test_lights.png", IMAGE), true);
}

TEST(ArmorDetectorTest, MatchLightsTest)
{
  R_ARMORS = DETECTOR->matchLights(R_LIGHTS);
  B_ARMORS = DETECTOR->matchLights(B_LIGHTS);

  // Draw armors
  for (const auto & armor : R_ARMORS) {
    cv::line(IMAGE, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0));
    cv::line(IMAGE, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0));
  }
  for (const auto & armor : B_ARMORS) {
    cv::line(IMAGE, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0));
    cv::line(IMAGE, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0));
  }
  EXPECT_EQ(cv::imwrite("/tmp/test_armors.png", IMAGE), true);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  bool debug = true;
  DETECTOR = std::make_unique<rm_auto_aim::ArmorDetector>(debug);
  return RUN_ALL_TESTS();
}
