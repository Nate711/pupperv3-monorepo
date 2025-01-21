#!/bin/bash

source /opt/ros/jazzy/setup.bash
source /home/pi/ros2_ws/install/setup.bash
ros2 launch neural_controller launch.py &
ros2 run pupper_feelings face_control
