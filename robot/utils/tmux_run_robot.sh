#!/bin/bash

# Check if the tmux session exists
if tmux has-session -t pupperv3 2>/dev/null; then
    echo "Session pupperv3 already exists. Killing..."
    tmux kill-session -t pupperv3
fi

echo "Creating new tmux session pupperv3."

# Create a new tmux session
tmux new-session -d -s pupperv3

# Split windows
tmux split-window -h
tmux split-window -h
tmux split-window -h

# Send commands to the first pane
tmux send-keys -t pupperv3:0.0 "source /opt/ros/humble/setup.bash; source /home/pi/ros2_ws/install/local_setup.bash" C-m
sleep 1
tmux send-keys -t pupperv3:0.0 "ros2 launch control_board_hardware_interface test_state_publisher.launch.py"

# Send commands to the second pane
tmux send-keys -t pupperv3:0.1 "source /opt/ros/humble/setup.bash; source /home/pi/ros2_ws/install/local_setup.bash" C-m
sleep 1
tmux send-keys -t pupperv3:0.1 "cd ~/StanfordQuadruped" C-m
tmux send-keys -t pupperv3:0.1 "python3 examples/pupper_v3_ds4_control.py"

# Send commands to the third pane
tmux send-keys -t pupperv3:0.2 "source /opt/ros/humble/setup.bash; source /home/pi/ros2_ws/install/local_setup.bash" C-m
sleep 1
# Publish joystick commands at max 50Hz by setting coalesce_interval param
tmux send-keys -t pupperv3:0.2 "ros2 run joy_linux joy_linux_node --ros-args -p coalesce_interval:=0.02"

# Send commands to the fourth pane
tmux send-keys -t pupperv3:0.3 "source /opt/ros/humble/setup.bash; source /home/pi/ros2_ws/install/local_setup.bash" C-m
sleep 1
tmux send-keys -t pupperv3:0.3 "ros2 run pupper_feelings face_control"


# Set layout and attach to the session
tmux select-layout -t pupperv3:0 even-horizontal
tmux attach-session -t pupperv3
