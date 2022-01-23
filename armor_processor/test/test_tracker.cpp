// Copyright 2022 Chen Jun

#include <gtest/gtest.h>

#include <memory>

#include "armor_processor/tracker.hpp"

std::unique_ptr<rm_auto_aim::Tracker> TRACKER;

TEST(TrackerTest, init)
{
  double max_match_distance = 1;
  int tracking_threshold = 5;
  int lost_threshold = 5;
  TRACKER =
    std::make_unique<rm_auto_aim::Tracker>(max_match_distance, tracking_threshold, lost_threshold);
  EXPECT_EQ(TRACKER->state, rm_auto_aim::Tracker::NO_FOUND);

  // Generate test armors
  auto_aim_interfaces::msg::Armors test_armors;
  auto_aim_interfaces::msg::Armor test_armor;
  test_armor.distance_to_image_center = 10;
  test_armors.armors.emplace_back(test_armor);
  test_armor.distance_to_image_center = 8;
  test_armors.armors.emplace_back(test_armor);
  test_armor.distance_to_image_center = 6;
  test_armors.armors.emplace_back(test_armor);

  TRACKER->init(test_armors);

  EXPECT_EQ(TRACKER->tracked_armor.distance_to_image_center, 6);
  EXPECT_EQ(TRACKER->state, rm_auto_aim::Tracker::DETECTING);
}

TEST(TrackerTest, update)
{
  // Generate test armors
  auto_aim_interfaces::msg::Armors test_armors;
  auto_aim_interfaces::msg::Armor test_armor;
  test_armors.armors.emplace_back(test_armor);

  // Detected 6 times
  Eigen::Vector3d predicted_position(0, 0, 0);
  for (int i = 0; i < 6; i++) {
    TRACKER->update(test_armors, predicted_position);
  }
  EXPECT_EQ(TRACKER->state, rm_auto_aim::Tracker::TRACKING);

  // Lost 6 times
  predicted_position << 10, 10, 10;
  for (int i = 0; i < 5; i++) {
    TRACKER->update(test_armors, predicted_position);
    EXPECT_EQ(TRACKER->state, rm_auto_aim::Tracker::LOST);
  }
  TRACKER->update(test_armors, predicted_position);
  EXPECT_EQ(TRACKER->state, rm_auto_aim::Tracker::NO_FOUND);
}
