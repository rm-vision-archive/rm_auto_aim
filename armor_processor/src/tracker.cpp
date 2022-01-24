// Copyright 2022 Chen Jun

#include "armor_processor/tracker.hpp"

#include <iostream>
#include <memory>

namespace rm_auto_aim
{
Tracker::Tracker(double max_match_distance, int tracking_threshold, int lost_threshold)
: state(NO_FOUND),
  max_match_distance_(max_match_distance),
  tracking_threshold_(tracking_threshold),
  lost_threshold_(lost_threshold)
{
}

void Tracker::init(const ArmorsMsg & armors_msg)
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

  tracked_armor = chosen_armor;
  tracked_position = getArmorPosition(chosen_armor);
  state = DETECTING;
}

void Tracker::update(const ArmorsMsg & armors_msg, const Eigen::Vector3d & predicted_position)
{
  bool matched = false;

  if (!armors_msg.armors.empty()) {
    double position_diff;
    double min_position_diff = 1e9;
    auto matched_armor = armors_msg.armors[0];
    for (const auto & armor : armors_msg.armors) {
      // Difference of the current armor position and tracked armor's predicted position
      const auto & armor_position = armor.position_stamped.point;
      Eigen::Vector3d position_vec(armor_position.x, armor_position.y, armor_position.z);
      position_diff = (predicted_position - position_vec).norm();
      if (position_diff < min_position_diff) {
        min_position_diff = position_diff;
        matched_armor = armor;
      }
    }
    matched = min_position_diff < max_match_distance_;

    if (matched) {
      tracked_armor = matched_armor;
      tracked_position = getArmorPosition(matched_armor);
    }
  }

  // Tracking state machine
  if (state == DETECTING) {
    // DETECTING
    if (matched) {
      detect_count_++;
    } else {
      detect_count_ = 0;
      state = NO_FOUND;
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
        state = NO_FOUND;
      }
    } else {
      state = TRACKING;
      lost_count_ = 0;
    }
  }
}

}  // namespace rm_auto_aim
