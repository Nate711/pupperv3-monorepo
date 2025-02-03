#!/bin/bash

source /opt/ros/jazzy/setup.bash
source /home/pi/pupperv3-monorepo/ros2_ws/install/local_setup.bash
ros2 launch neural_controller launch.py
# ros2 run pupper_feelings face_control
