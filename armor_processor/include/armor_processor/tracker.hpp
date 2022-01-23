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

  void init(auto_aim_interfaces::msg::Armors armors_msg);
  void update(auto_aim_interfaces::msg::Armors armors_msg, Eigen::Vector3d predicted_position);

  enum State {
    DETECTING,
    TRACKING,
    LOST,
  } state;

  std::unique_ptr<auto_aim_interfaces::msg::Armor> tracked_armor;

private:
  double max_match_distance_;

  int tracking_threshold_;
  int lost_threshold_;

  int detect_count_;
  int lost_count_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
