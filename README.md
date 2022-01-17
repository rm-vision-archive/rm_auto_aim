# RM-Auto-Aim

## Overview

This is a node of Robomaster auto-aim task using OpenCV and ROS2.

**Keywords:** Robomaster, auto-aim, OpenCV, ROS2

### License

To be added.
<!-- The source code is released under a [BSD 3-Clause license](rm_auto_aim/LICENSE). -->

**Author: Chen Jun<br />
Maintainer: Chen Jun, chen.junn@outlook.com**

The `rm_auto_aim` package has been tested **only** under [ROS] Galactic on Ubuntu 20.04.

![Build Status](https://github.com/chenjunnn/rm_auto_aim/actions/workflows/ros_ci.yml/badge.svg)

<!-- ![Example image](doc/example.jpg) -->

## Installation

<!-- ### Installation from Packages

To install all packages from the this repository as Debian packages use

    sudo apt-get install ros-noetic-...
    
Or better, use `rosdep`:

	sudo rosdep install --from-paths src -->

### Building from Source

#### Dependencies

- [Robot Operating System 2 (ROS2)](https://docs.ros.org/en/galactic/) (middleware for robotics),
<!-- - [Eigen] (linear algebra library) -->
- [OpenCV](https://opencv.org/) (open source computer vision library)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd ros2_ws/src
	git clone https://github.com/chenjunnn/rm_auto_aim.git
	cd ..
	rosdep install --from-paths src --ignore-src -r -y
	colcon build --symlink-install --packages-up-to rm_auto_aim

<!-- ### Running in Docker

Docker is a great way to run an application with all dependencies and libraries bundles together. 
Make sure to [install Docker](https://docs.docker.com/get-docker/) first. 

First, spin up a simple container:

	docker run -ti --rm --name ros-container ros:noetic bash
	
This downloads the `ros:noetic` image from the Docker Hub, indicates that it requires an interactive terminal (`-t, -i`), gives it a name (`--name`), removes it after you exit the container (`--rm`) and runs a command (`bash`).

Now, create a catkin workspace, clone the package, build it, done!

	apt-get update && apt-get install -y git
	mkdir -p /ws/src && cd /ws/src
	git clone https://github.com/leggedrobotics/ros_best_practices.git
	cd ..
	rosdep install --from-path src
	catkin_make
	source devel/setup.bash
	roslaunch rm_auto_aim rm_auto_aim.launch -->

### Unit Tests

Run the tests with

	colcon test --event-handlers console_cohesion+ --packages-up-to rm_auto_aim

## Usage

Run the main node with

	ros2 launch rm_auto_aim rm_auto_aim.launch.py

## Config files

Config file config/rm_auto_aim.yaml

* **rm_auto_aim.yaml** <!--Shortly explain the content of this config file-->

<!-- Config file folder/set 2

* **...** -->

## Launch files

* **rm_auto_aim.launch.py:** shortly explain what is launched (e.g standard simulation, simulation with gdb,...)

     Argument set 1

     - **`argument_1`** Short description (e.g. as commented in launch file). Default: `default_value`.

    Argument set 2

    - **`...`**

* **...**

## Nodes

### rm_auto_aim

Reads temperature measurements and computed the average.


#### Subscribed Topics

* **`/camera/color/image_raw`** ([sensor_msgs/msg/Image])


#### Published Topics

* **`/debug/final_image`** ([sensor_msgs/msg/Image])

<!-- #### Services

* **`get_average`** ([std_srvs/Trigger])

	Returns information about the current average. For example, you can trigger the computation from the console with

		rosservice call /rm_auto_aim/get_average -->


#### Parameters

* **`debug`** (bool, default: true)

	Publish debug infos or not.

<!-- * **`cache_size`** (int, default: 200, min: 0, max: 1000)

	The size of the cache. -->


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/chenjunnn/rm_auto_aim/issues).


[ROS]: http://www.ros.org
