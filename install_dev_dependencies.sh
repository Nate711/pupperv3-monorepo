#!/bin/bash

set -e
set -o pipefail

sudo apt update

# Install ROS2
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb
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
sudo apt-get install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-teleop-twist-joy ros-jazzy-foxglove-bridge ros-jazzy-xacro ros-jazzy-hardware-interface
sudo apt-get install -y ros-jazzy-vision-msgs ros-jazzy-camera-calibration ros-jazzy-image-transport-plugins ros-jazzy-theora-image-transport ros-jazzy-compressed-depth-image-transport ros-jazzy-compressed-image-transport
sudo apt-get install -y ros-jazzy-foxglove-bridge
sudo apt-get install -y ros-jazzy-joy-linux
sudo apt-get install ros-jazzy-camera-ros

# Upgrade packages near the end since it takes a long time
sudo apt upgrade -y

# Build ROS2 workspace
source /opt/ros/jazzy/setup.bash
bash $SCRIPT_DIR/ros2_ws/build.sh

echo "source $SCRIPT_DIR/ros2_ws/install/setup.bash" >> $HOME/.bashrc
echo 'export RCUTILS_COLORIZED_OUTPUT=1' >> $HOME/.bashrc
source $HOME/.bashrc