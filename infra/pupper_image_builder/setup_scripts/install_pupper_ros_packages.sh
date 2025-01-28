#!/bin/bash -e

set -x

DEFAULT_USER=pi

# Function to retry a command
retry_command() {
  local cmd="$1"
  local max_attempts="$2"
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

source /opt/ros/jazzy/setup.bash

# Create ROS2 workspace
mkdir -p /home/$DEFAULT_USER/ros2_ws/src
cd /home/$DEFAULT_USER/ros2_ws/src
retry_command "git clone https://github.com/G-Levine/control_board_hardware_interface.git" 20
retry_command "git clone https://github.com/G-Levine/neural_controller.git --recurse-submodules" 20
retry_command "git clone https://github.com/G-Levine/pupper_v3_description.git" 20
retry_command "git clone https://github.com/Nate711/pupper_feelings.git" 20
retry_command "git clone https://github.com/christianrauch/camera_ros.git" 20

# Install wandb for neural controller
sudo rm -f /usr/lib/python3.*/EXTERNALLY-MANAGED
pip install --ignore-installed typing_extensions
pip install --upgrade wandb

# Install dependencies
cd /home/$DEFAULT_USER/ros2_ws
sudo apt install -y python3-colcon-common-extensions python3-rosdep
retry_command "sudo rosdep init" 20
retry_command "rosdep update" 20
retry_command "rosdep install --from-paths src -y --ignore-src --skip-keys=libcamera" 20

# Install additional ROS2 packages
sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-teleop-twist-joy ros-jazzy-foxglove-bridge ros-jazzy-xacro
sudo apt install -y ros-jazzy-vision-msgs ros-jazzy-camera-calibration ros-jazzy-image-transport-plugins ros-jazzy-theora-image-transport ros-jazzy-compressed-depth-image-transport ros-jazzy-compressed-image-transport

# Upgrade packages near the end since it takes a long time
sudo apt upgrade -y

# Build ROS2 workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
echo 'source ~/ros2_ws/install/setup.bash' >> /home/$DEFAULT_USER/.bashrc
echo 'alias build="cd $HOME/ros2_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && cd -"' >> /home/$DEFAULT_USER/.bashrc
echo 'export RCUTILS_COLORIZED_OUTPUT=1' >> /home/$DEFAULT_USER/.bashrc
source /home/$DEFAULT_USER/ros2_ws/install/setup.bash

# Install utils
cd /home/$DEFAULT_USER
retry_command "git clone https://github.com/Nate711/utils.git -b launch_neural_controller" 20
