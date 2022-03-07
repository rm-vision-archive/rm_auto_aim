// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

// Eigen
#include <eigen3/Eigen/Eigen>

// STD
#include <memory>

#include "auto_aim_interfaces/msg/armors.hpp"

namespace rm_auto_aim
{
class Tracker
{
public:
  Tracker(double max_match_distance, int tracking_threshold, int lost_threshold);

  using ArmorsMsg = auto_aim_interfaces::msg::Armors;
  using Armor = auto_aim_interfaces::msg::Armor;

  void init(const ArmorsMsg & armors_msg);
  void update(const ArmorsMsg & armors_msg, const Eigen::Vector3d & predicted_position);

  Armor tracked_armor;
  Eigen::Vector3d tracked_position;

  enum State {
    NO_FOUND,
    DETECTING,
    TRACKING,
    LOST,
  } state;

private:
  double max_match_distance_;

  int tracking_threshold_;
  int lost_threshold_;

  int detect_count_;
  int lost_count_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
