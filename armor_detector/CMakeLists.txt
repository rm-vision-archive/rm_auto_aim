cmake_minimum_required(VERSION 3.10)
project(armor_detector)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/armor_detector_node.cpp
  src/armor_detector.cpp
  src/depth_processor.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::ArmorDetectorNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)
  ament_add_gtest(test_armor_detector test/test_armor_detector.cpp)
  target_link_libraries(test_armor_detector ${PROJECT_NAME})

  ament_add_gtest(test_node test/test_node.cpp)
  target_link_libraries(test_node ${PROJECT_NAME})
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)