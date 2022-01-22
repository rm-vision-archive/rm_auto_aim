// Copyright 2022 Chen Jun

#include "armor_processor/kalman_filter.hpp"

namespace rm_auto_aim
{
KalmanFilter::KalmanFilter(
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & H, const Eigen::MatrixXd & Q,
  const Eigen::MatrixXd & R, const Eigen::MatrixXd & P)
: A_(A), H_(H), Q_(Q), R_(R), P_(P), n_(A.rows()), I_(n_, n_), x_pre_(n_), x_post_(n_)
{
}

void KalmanFilter::init(const Eigen::VectorXd & x0) { x_post_ = x0; }

Eigen::MatrixXd KalmanFilter::predict(const Eigen::MatrixXd A)
{
  this->A_ = A;

  // x'(k) = A*x(k)
  x_pre_ = A_ * x_post_;
  // P'(k) = A*P(k)*At + Q
  P_ = A_ * P_ * A_.transpose() + Q_;

  return x_pre_;
}

Eigen::MatrixXd KalmanFilter::correct(const Eigen::VectorXd & measurement)
{
  K_ = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();
  x_pre_ += K_ * (measurement - H_ * x_pre_);
  P_ = (I_ - K_ * H_) * P_;
  x_post_ = x_pre_;

  return x_post_;
}

void KalmanFilter::setA(int m, int n, double value) { A_(m, n) = value; }

void KalmanFilter::setQ(int m, int n, double value) { Q_(m, n) = value; }

void KalmanFilter::setR(int m, int n, double value) { R_(m, n) = value; }

}  // namespace rm_auto_aim
