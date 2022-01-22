// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

#include <eigen3/Eigen/Eigen>

#include "auto_aim_interfaces/msg/armors.hpp"

namespace rm_auto_aim
{
class Tracker
{
public:
  Tracker(int tracking_threshold, int lost_threshold);

  void update(auto_aim_interfaces::msg::Armors armors);
  void update(auto_aim_interfaces::msg::Armors armors, Eigen::Vector3d predicted_position);

  enum State {
    DETECTING,
    TRACKING,
    LOST,
  } state;

private:
  int detect_count_;
  int lost_count_;

  int tracking_threshold_;
  int lost_threshold_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
