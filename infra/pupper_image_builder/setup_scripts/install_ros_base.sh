#!/bin/bash -e

set -x

DEFAULT_USER=pi
mkdir -p /home/$DEFAULT_USER

# Install ROS2
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo wget -q https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O /usr/share/keyrings/ros-archive-keyring.gpg
sudo bash -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null'
sudo apt update && sudo apt install -y ros-dev-tools
sudo apt install -y ros-jazzy-desktop
echo 'source /opt/ros/jazzy/setup.bash' >> /home/$DEFAULT_USER/.bashrc
source /opt/ros/jazzy/setup.bash
