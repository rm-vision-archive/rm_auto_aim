# rm_auto_aim

## Overview

This is a suite of Robomaster auto-aim task using ROS2.

**Keywords:** Robomaster, auto-aim, ROS2

![](armor_detector/docs/result.png)

### License

The source code is released under a [MIT license](rm_auto_aim/LICENSE).

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

**Author: Chen Jun**

**Maintainer: Chen Jun, chen.junn@outlook.com**

The `rm_auto_aim` package has been tested **only** under ROS Galactic on Ubuntu 20.04.

![Build Status](https://github.com/chenjunnn/rm_auto_aim/actions/workflows/ros_ci.yml/badge.svg)

## Building from Source

### Dependencies

- [Robot Operating System 2 (ROS2)](https://docs.ros.org/en/galactic/) (middleware for robotics),

### Building

To build from source, clone the latest version from this repository into your colcon workspace and compile the package using

	cd ros_ws/src
	git clone https://github.com/chenjunnn/rm_auto_aim.git
	cd ..
	rosdep install --from-paths src --ignore-src -r -y
	colcon build --symlink-install --packages-up-to rm_auto_aim

### Testing

Run the tests with

	colcon test --packages-up-to rm_auto_aim

## Packages

- [armor_detector](armor_detector)

	支持RGB及RGBD输入，订阅相机参数并用于构造相应的三维位置解算器，订阅来自相机或视频的图像流进行装甲板的识别，识别完成后发布识别到的装甲板目标

- [armor_processor](armor_processor)

	订阅识别节点发布的装甲板目标及机器人的坐标转换信息，将装甲板目标通过 `tf` 变换到世界坐标系下，然后将目标送入跟踪器中得到跟踪目标在世界坐标系下的位置及速度，再经过小陀螺观测器的处理后，发布最终的目标位置和速度

- [auto_aim_interfaces](auto_aim_interfaces)

	定义了识别节点和处理节点的接口，以及定义了一系列用于 Debug 的信息

- [auto_aim_bringup](auto_aim_bringup)

	包含启动识别节点和处理节点的默认参数文件及 launch 文件

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/chenjunnn/rm_auto_aim/issues).
