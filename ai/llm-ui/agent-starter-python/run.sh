#!/bin/bash

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Set vol to 100%
amixer sset 'Master' 100%

# Source local workspace
source /home/pi/pupperv3-monorepo/ros2_ws/install/local_setup.bash

# Run the agent with unbuffered output
exec /usr/bin/python3 -u src/agent.py console