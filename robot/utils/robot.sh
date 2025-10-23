#!/bin/bash

source /opt/ros/jazzy/setup.bash
source /home/pi/pupperv3-monorepo/ros2_ws/install/local_setup.bash
ROS_LOCALHOST_ONLY=1 ros2 launch neural_controller launch.py
