// Copyright 2022 Chen Jun

#include "armor_processor/tracker.hpp"

#include <memory>

namespace rm_auto_aim
{
Tracker::Tracker(double max_match_distance, int tracking_threshold, int lost_threshold)
: state(DETECTING),
  max_match_distance_(max_match_distance),
  tracking_threshold_(tracking_threshold),
  lost_threshold_(lost_threshold)
{
}

void Tracker::init(auto_aim_interfaces::msg::Armors armors_msg)
{
  // TODO(chenjun): need more judgement
  // Simply choose the armor that is closest to image center
  double min = 1e9;
  auto chosen_armor = armors_msg.armors[0];
  for (const auto & armor : armors_msg.armors) {
    if (armor.distance_to_image_center < min) {
      min = armor.distance_to_image_center;
      chosen_armor = armor;
    }
  }

  tracked_armor = std::make_unique<auto_aim_interfaces::msg::Armor>(chosen_armor);
}

void Tracker::update(
  auto_aim_interfaces::msg::Armors armors_msg, Eigen::Vector3d predicted_position)
{
  Eigen::Vector3d armor_position;
  double position_diff;
  double min_position_diff = 1e9;
  auto matched_armor = armors_msg.armors[0];
  for (const auto & armor : armors_msg.armors) {
    // Difference of the current armor position and tracked armor's predicted position
    armor_position << armor.position.x, armor.position.y, armor.position.z;
    position_diff = (predicted_position - armor_position).norm();

    if (position_diff < min_position_diff) {
      min_position_diff = position_diff;
      matched_armor = armor;
    }
  }

  bool matched = min_position_diff < max_match_distance_;

  // Tracking state machine
  if (state == DETECTING) {
    // DETECTING
    if (matched) {
      detect_count_++;
    } else {
      detect_count_ = 0;
      tracked_armor.reset();
    }
    if (detect_count_ > tracking_threshold_) {
      detect_count_ = 0;
      state = TRACKING;
    }

  } else if (state == TRACKING) {
    // TRACKING
    if (!matched) {
      state = LOST;
      lost_count_++;
    }

  } else if (state == LOST) {
    // LOST
    if (!matched) {
      lost_count_++;
      if (lost_count_ > lost_threshold_) {
        lost_count_ = 0;
        state = DETECTING;
        tracked_armor.reset();
      }
    } else {
      state = TRACKING;
      lost_count_ = 0;
    }
  }
}

}  // namespace rm_auto_aim
