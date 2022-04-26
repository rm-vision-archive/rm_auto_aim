# rm_auto_aim

## Overview

This is a package of Robomaster auto-aim task using ROS2.

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
	colcon build --symlink-install --packages-up-to auto_aim_bringup

### Testing

Run the tests with

	colcon test --packages-up-to auto_aim_bringup

## Packages

- [armor_detector](armor_detector/README.md)

- [armor_processor](armor_processor/README.md)

- [auto_aim_interfaces](auto_aim_interfaces/README.md)

- [auto_aim_bringup](auto_aim_bringup/README.md)

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/chenjunnn/rm_auto_aim/issues).
