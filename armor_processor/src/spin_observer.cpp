// Copyright 2022 Chen Jun

#include "armor_processor/spin_observer.hpp"

namespace rm_auto_aim
{
SpinObserver::SpinObserver()
: max_jump_distance_(0.1), max_jump_period_(0.5), allow_following_range(0.2), fire_delay(0.1)
{
  target_spinning_ = false;
  jump_period_ = 0.0;
  jump_count_ = 0;
  last_jump_time_ = rclcpp::Time(0);
  last_jump_position_ = Eigen::Vector3d(0, 0, 0);
}

void SpinObserver::update(auto_aim_interfaces::msg::Target & target_msg)
{
  rclcpp::Time current_time = target_msg.header.stamp;
  Eigen::Vector3d current_position(
    target_msg.position.x, target_msg.position.y, target_msg.position.z);

  if ((current_time - last_jump_time_).seconds() > max_jump_period_) {
    target_spinning_ = false;
    jump_count_ = 0;
  }

  double time_after_jumping = (current_time - last_jump_time_).seconds();

  if ((current_position - last_position_).norm() > max_jump_distance_) {
    jump_count_++;
    if (jump_count_ > 1) {
      target_spinning_ = true;
      jump_period_ = time_after_jumping;
    }

    last_jump_time_ = current_time;
    last_jump_position_ = current_position;
  }

  if (jump_count_ > 1) {
    // Anti spinning mode on
    if (time_after_jumping / jump_period_ < allow_following_range) {
      target_msg.suggest_fire = true;
    } else {
      target_msg.position.x = last_jump_position_.x();
      target_msg.position.y = last_jump_position_.y();
      target_msg.position.z = last_jump_position_.z();
      target_msg.velocity.x = 0;
      target_msg.velocity.y = 0;
      target_msg.velocity.z = 0;

      target_msg.suggest_fire = jump_period_ - time_after_jumping < fire_delay;
    }
  }

  // Update last position
  last_position_ << target_msg.position.x, target_msg.position.y, target_msg.position.z;

  // Update spin_info_msg
  spin_info_msg.header = target_msg.header;
  spin_info_msg.target_spinning = target_spinning_;
  spin_info_msg.suggest_fire = target_msg.suggest_fire;
  spin_info_msg.jump_period = jump_period_;
  spin_info_msg.time_after_jumping = time_after_jumping;
}

}  // namespace rm_auto_aim
