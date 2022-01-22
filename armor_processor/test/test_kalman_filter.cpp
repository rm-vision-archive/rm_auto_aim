// Copyright 2022 Chen Jun

#include <gtest/gtest.h>

#include <memory>
#include <random>
#include <vector>

#include "armor_processor/kalman_filter.hpp"

int N = 2;  // Number of states
int M = 1;  // Number of measurements

Eigen::MatrixXd A(N, N);  // state transition matrix
Eigen::MatrixXd H(M, N);  // measurement matrix
Eigen::MatrixXd Q(N, N);  // process noise covariance matrix
Eigen::MatrixXd R(M, M);  // measurement noise covariance matrix
Eigen::MatrixXd P(N, N);  // error estimate covariance matrix

std::unique_ptr<rm_auto_aim::KalmanFilter> KF;

TEST(KalmanFilterTest, InitTest)
{
  // Test x = x0 + v*t
  double dt = 1.0;
  A << 1, dt, 0, 1;
  H << 1, 0;

  Q << .05, .05, .0, .05;
  R << 0.1;
  P.setIdentity();

  KF = std::make_unique<rm_auto_aim::KalmanFilter>(A, H, Q, R, P);

  std::cout << "A: \n" << A << std::endl;
  std::cout << "H: \n" << H << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P: \n" << P << std::endl;
}

TEST(KalmanFilterTest, EstimateTest)
{
  std::vector<double> measurements{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

  // Add noise
  std::default_random_engine e;
  std::uniform_real_distribution<double> u(-0.1, 0.1);
  std::cout << "measurement: \n" << std::endl;
  for (auto & measurement : measurements) {
    measurement += u(e);
    std::cout << measurement << "\n" << std::endl;
  }

  // Init
  Eigen::VectorXd x0(N);
  x0 << 0, 0;
  KF->init(x0);

  // Estimate
  std::cout << "result: \n" << std::endl;
  Eigen::VectorXd measurement_vector(M);
  for (const auto & measurement : measurements) {
    KF->predict(A);
    measurement_vector << measurement;
    auto result = KF->correct(measurement_vector);
    std::cout << result.transpose() << std::endl;
  }
}
