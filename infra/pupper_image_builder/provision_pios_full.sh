#!/bin/bash -e

set -x

# Function to retry a command
retry_command() {
  local cmd="$1"
  local max_attempts=20
  local attempt=0

  # Loop to retry the command
  while [ $attempt -lt $max_attempts ]; do
    echo "Attempting to run: $cmd (Attempt $((attempt + 1))/$max_attempts)"
    
    # Run the command
    if eval "$cmd"; then
      echo "Command succeeded!"
      return 0  # Exit the function as success
    else
      attempt=$((attempt + 1))
      echo "Attempt $attempt/$max_attempts failed. Retrying in 5 seconds..."
      sleep 1
    fi
  done

  # If max attempts reached
  echo "Command failed after $max_attempts attempts."
  return 1  # Indicate failure
}

export DEBIAN_FRONTEND=noninteractive


DEFAULT_USER=pi
mkdir -p /home/$DEFAULT_USER

sudo apt update
sudo apt upgrade -y

sudo rm -f /usr/lib/python3.*/EXTERNALLY-MANAGED
pip install wandb

source /opt/ros/jazzy/setup.bash
cd /home/$DEFAULT_USER/ros2_ws/src
rm -rf control_board_hardware_interface neural_controller pupper_v3_description pupper_feelings
retry_command "git clone https://github.com/G-Levine/control_board_hardware_interface.git --recurse-submodules"
retry_command "git clone https://github.com/G-Levine/neural_controller.git --recurse-submodules"
retry_command "git clone https://github.com/G-Levine/pupper_v3_description.git --recurse-submodules"
retry_command "git clone https://github.com/Nate711/pupper_feelings.git --recurse-submodules"

# Install dependencies
cd /home/$DEFAULT_USER/ros2_ws
# pip install rosdep
# sudo rosdep init && rosdep update
# rosdep install --from-paths src -y --ignore-src

# Build ROS2 workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
echo 'source ~/ros2_ws/install/setup.bash' >> /home/$DEFAULT_USER/.bashrc

# Alias build convenience command
echo 'alias build="cd $HOME/ros2_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && cd -"' >> /home/$DEFAULT_USER/.bashrc

source /home/$DEFAULT_USER/ros2_ws/install/setup.bash

# Install utils
cd /home/$DEFAULT_USER
rm -rf utils
git clone https://github.com/Nate711/utils.git -b launch_neural_controller
bash /home/$DEFAULT_USER/utils/install_battery_monitor.sh
# bash /home/$DEFAULT_USER/utils/install_robot_auto_start_service.sh

# Install hailo
sudo apt full-upgrade -y
sudo apt install -y hailo-all

