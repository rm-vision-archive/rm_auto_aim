// Copyright 2022 Chen Jun

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core/mat.hpp>

// STL
#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <vector>

#include "armor_detector/number_classifier.hpp"

using hrc = std::chrono::high_resolution_clock;

void benchmark(int batch_size)
{
  std::map<char, double> similarity_threshold = {
    {'1', 0.8}, {'2', 0.7}, {'3', 0.7}, {'4', 0.7}, {'5', 0.7}, {'B', 0.7}, {'G', 0.7}, {'O', 0.7},
  };
  auto model_path = ament_index_cpp::get_package_share_directory("armor_detector") +
                    "/model/number_classification_14.onnx";
  rm_auto_aim::NumberClassifier nc(2.0, 0.4, 0.56, similarity_threshold, model_path);

  //  随机生成
  // auto dummy_armors = std::vector<rm_auto_aim::Armor>(batch_size);
  // for (auto & armor : dummy_armors) {
  //   auto test_mat = cv::Mat(20, 28, CV_8UC1);
  //   armor.number_img = test_mat;
  // }

  //  固定图片
  auto dummy_armors = std::vector<rm_auto_aim::Armor>(batch_size);
  for (auto & armor : dummy_armors) {
    cv::Mat number_image =
      cv::imread("/root/ros_ws/src/rm_pioneer_vision/rm_auto_aim/armor_detector/template/1.png");
    armor.number_img = number_image;
  }

  int loop_num = 1;
  int warm_up = 1;

  double time_min = DBL_MAX;
  double time_max = -DBL_MAX;
  double time_avg = 0;

  for (int i = 0; i < warm_up + loop_num; i++) {
    auto start = hrc::now();
    nc.fcClassify(dummy_armors);
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

// TEST(benchmark, batch_size_5) { benchmark(5); }

// TEST(benchmark, batch_size_10) { benchmark(10); }

// TEST(benchmark, batch_size_20) { benchmark(20); }
