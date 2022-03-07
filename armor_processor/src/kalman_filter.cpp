// Copyright 2022 Chen Jun

#include "armor_processor/kalman_filter.hpp"

namespace rm_auto_aim
{
KalmanFilter::KalmanFilter(const KalmanFilterMatrices & matrices)
: A_(matrices.A),
  H_(matrices.H),
  Q_(matrices.Q),
  R_(matrices.R),
  P_(matrices.P),
  n_(matrices.A.rows()),
  I_(Eigen::MatrixXd::Identity(n_, n_)),
  x_pre_(n_),
  x_post_(n_)
{
I_.setIdentity();
}

void KalmanFilter::init(const Eigen::VectorXd & x0) { x_post_ = x0; }

Eigen::MatrixXd KalmanFilter::predict(const Eigen::MatrixXd & A)
{
  this->A_ = A;

  // x'(k) = A*x(k)
  x_pre_ = A_ * x_post_;
  // P'(k) = A*P(k)*At + Q
  P_ = A_ * P_ * A_.transpose() + Q_;

  return x_pre_;
}

Eigen::MatrixXd KalmanFilter::update(const Eigen::VectorXd & measurement)
{
  K_ = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();
  x_pre_ += K_ * (measurement - H_ * x_pre_);
  P_ = (I_ - K_ * H_) * P_;
  x_post_ = x_pre_;

  return x_post_;
}

}  // namespace rm_auto_aim
