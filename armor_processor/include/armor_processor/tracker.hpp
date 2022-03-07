// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// ROS
#include <auto_aim_interfaces/msg/detail/target__struct.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// STD
#include <memory>

#include "armor_processor/kalman_filter.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"

namespace rm_auto_aim
{
class Tracker
{
public:
  Tracker(double max_match_distance, int tracking_threshold, int lost_threshold);

  using Armors = auto_aim_interfaces::msg::Armors;
  using Armor = auto_aim_interfaces::msg::Armor;

  void init(const Armors::SharedPtr & armors_msg, const KalmanFilterMatrices & kf_matrices);

  void update(const Armors::SharedPtr & armors_msg, const Eigen::MatrixXd & kf_a);

  enum State {
    NO_FOUND,
    DETECTING,
    TRACKING,
    LOST,
  } tracker_state;

  Eigen::VectorXd target_state;

private:
  std::unique_ptr<KalmanFilter> kf_;

  double max_match_distance_;

  int tracking_threshold_;
  int lost_threshold_;

  int detect_count_;
  int lost_count_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
