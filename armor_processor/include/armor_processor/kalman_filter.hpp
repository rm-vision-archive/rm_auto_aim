// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__KALMAN_FILTER_HPP_

#include <Eigen/Dense>

namespace rm_auto_aim
{
struct KalmanFilterMatrices
{
  Eigen::MatrixXd A;  // state transition matrix
  Eigen::MatrixXd H;  // measurement matrix
  Eigen::MatrixXd Q;  // process noise covariance matrix
  Eigen::MatrixXd R;  // measurement noise covariance matrix
  Eigen::MatrixXd P;  // error estimate covariance matrix
};

class KalmanFilter
{
public:
  explicit KalmanFilter(const KalmanFilterMatrices & matrices);

  // Initialize the filter with a guess for initial states.
  void init(const Eigen::VectorXd & x0);

  // Computes a predicted state
  Eigen::MatrixXd predict(const Eigen::MatrixXd & A);

  // Update the estimated state based on measurement
  Eigen::MatrixXd update(const Eigen::VectorXd & measurement);

private:
  // Matrices
  Eigen::MatrixXd A_, H_, Q_, R_, P_;

  // Kalman Gain
  Eigen::MatrixXd K_;

  // System dimensions
  int n_;

  // n-size identity
  Eigen::MatrixXd I_;

  // predicted state
  Eigen::VectorXd x_pre_;
  // corrected state
  Eigen::VectorXd x_post_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
