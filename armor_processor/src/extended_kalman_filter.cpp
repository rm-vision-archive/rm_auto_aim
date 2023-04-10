// Copyright 2022 Chen Jun

#include "armor_processor/extended_kalman_filter.hpp"

namespace rm_auto_aim
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const NonlinearFunc & f, const NonlinearFunc & h, const JacobianFunc & Jf,
  const JacobianFunc & Jh, const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R,
  const Eigen::MatrixXd & P0)
: f(f),
  h(h),
  Jf(Jf),
  Jh(Jh),
  Q(Q),
  R(R),
  P_post(P0),
  n(Q.rows()),
  I(Eigen::MatrixXd::Identity(n, n)),
  x_pri(n),
  x_post(n)
{
}

void ExtendedKalmanFilter::setState(const Eigen::VectorXd & x0) { x_post = x0; }

Eigen::MatrixXd ExtendedKalmanFilter::predict()
{
  x_pri = f(x_post);
  F = Jf(x_post);
  P_pri = F * P_post * F.transpose() + Q;

  // handle the case when there will be no measurement before the next predict
  x_post = x_pri;
  P_post = P_pri;

  return x_pri;
}

Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z)
{
  H = Jh(x_pri);
  K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
  x_post = x_pri + K * (z - h(x_pri));
  P_post = (I - K * H) * P_pri;

  return x_post;
}

}  // namespace rm_auto_aim
