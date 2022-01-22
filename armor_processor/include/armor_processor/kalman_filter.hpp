// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__KALMAN_FILTER_HPP_

#include <eigen3/Eigen/Dense>

namespace rm_auto_aim
{
class KalmanFilter
{
public:
  /**
  * Create a Kalman filter with the specified matrices.
  *   A - state transition matrix
  *   H - measurement matrix
  *   Q - process noise covariance matrix
  *   R - measurement noise covariance matrix
  *   P - error estimate covariance matrix
  */
  KalmanFilter(
    const Eigen::MatrixXd & A, const Eigen::MatrixXd & H, const Eigen::MatrixXd & Q,
    const Eigen::MatrixXd & R, const Eigen::MatrixXd & P);

  // Initialize the filter with a guess for initial states.
  void init(const Eigen::VectorXd & x0);

  // Computes a predicted state
  Eigen::MatrixXd predict(const Eigen::MatrixXd A);

  // Update the estimated state based on measurement
  Eigen::MatrixXd correct(const Eigen::VectorXd & measurement);

  void setA(int m, int n, double value);
  void setQ(int m, int n, double value);
  void setR(int m, int n, double value);

private:
  // state transition matrix
  Eigen::MatrixXd A_;
  // measurement matrix
  Eigen::MatrixXd H_;
  // process noise covariance matrix
  Eigen::MatrixXd Q_;
  // measurement noise covariance matrix
  Eigen::MatrixXd R_;
  // error estimate covariance matrix
  Eigen::MatrixXd P_;
  // Kalman gain matrix
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
