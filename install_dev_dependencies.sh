#!/bin/bash

set -e
set -o pipefail

sudo apt update

# Install ROS2
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo apt update && sudo apt install curl -y
sudo wget -q https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O /usr/share/keyrings/ros-archive-keyring.gpg
sudo bash -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null'
sudo apt update && sudo apt install -y ros-dev-tools
sudo apt install -y ros-jazzy-desktop
echo 'source /opt/ros/jazzy/setup.bash' >> $HOME/.bashrc
source /opt/ros/jazzy/setup.bash

# install pip
sudo apt install -y python3-pip libglfw3-dev

pip install wandb glfw openai[realtime] pyaudio pydub numpy sounddevice --break-system-packages

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Install dependencies
cd $SCRIPT_DIR/ros2_ws
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init || true
rosdep update
rosdep install --from-paths src -y --ignore-src --skip-keys=libcamera

# Install additional ROS2 packages
sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-teleop-twist-joy ros-jazzy-foxglove-bridge ros-jazzy-xacro ros-jazzy-hardware-interface
sudo apt install -y ros-jazzy-vision-msgs ros-jazzy-camera-calibration ros-jazzy-image-transport-plugins ros-jazzy-theora-image-transport ros-jazzy-compressed-depth-image-transport ros-jazzy-compressed-image-transport
sudo apt install -y ros-jazzy-foxglove-bridge

# Upgrade packages near the end since it takes a long time
sudo apt upgrade -y

# Build ROS2 workspace
source /opt/ros/jazzy/setup.bash
bash $SCRIPT_DIR/ros2_ws/build.sh

echo "source $SCRIPT_DIR/ros2_ws/install/setup.bash" >> $HOME/.bashrc
echo 'export RCUTILS_COLORIZED_OUTPUT=1' >> $HOME/.bashrc
source $HOME/.bashrc