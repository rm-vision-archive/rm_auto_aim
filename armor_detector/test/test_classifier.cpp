// Copyright 2022 Chen Jun

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core/mat.hpp>

// STL
#include <algorithm>
#include <chrono>
#include <iostream>
#include <vector>

#include "armor_detector/number_classifier.hpp"

using hrc = std::chrono::high_resolution_clock;

void benchmark(int batch_size)
{
  auto model_path =
    ament_index_cpp::get_package_share_directory("armor_detector") + "/model/model.onnx";
  rm_auto_aim::NumberClassifier nc(0.5, 0.5, model_path);

  auto test_mat = cv::Mat(32, 32, CV_8UC1, cv::Scalar(0));
  auto test_mat_vector = std::vector<cv::Mat>(batch_size, test_mat);

  auto dummy_armors = std::vector<rm_auto_aim::Armor>(batch_size);

  int loop_num = 100;
  int warm_up = 20;

  double time_min = DBL_MAX;
  double time_max = -DBL_MAX;
  double time_avg = 0;

  for (int i = 0; i < warm_up + loop_num; i++) {
    auto start = hrc::now();
    nc.doClassify(test_mat_vector, dummy_armors);
    auto end = hrc::now();
    double time = std::chrono::duration<double, std::milli>(end - start).count();
    if (i >= warm_up) {
      time_min = std::min(time_min, time);
      time_max = std::max(time_max, time);
      time_avg += time;
    }
  }
  time_avg /= loop_num;

  std::cout << "batch_size: " << batch_size << std::endl;
  std::cout << "time_min: " << time_min << "ms" << std::endl;
  std::cout << "time_max: " << time_max << "ms" << std::endl;
  std::cout << "time_avg: " << time_avg << "ms" << std::endl;
}

TEST(benchmark, batch_size_1) { benchmark(1); }

TEST(benchmark, batch_size_5) { benchmark(5); }

TEST(benchmark, batch_size_10) { benchmark(10); }

TEST(benchmark, batch_size_20) { benchmark(20); }
