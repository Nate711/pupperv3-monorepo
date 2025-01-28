#!/bin/bash -e

set -x

# Function to retry a command
retry_command() {
    local cmd="$1"
    local max_attempts=20
    local attempt=0

    until eval "$cmd" || [ $attempt -ge $max_attempts ]; do
        attempt=$((attempt + 1))
        echo "Attempt $attempt/$max_attempts failed. Retrying in 1 seconds..."
        sleep 1
    done

    if [ $attempt -ge $max_attempts ]; then
        echo "Command failed after $max_attempts attempts."
        return 1
    fi

    echo "Command succeeded!"
    return 0
}

export DEBIAN_FRONTEND=noninteractive


DEFAULT_USER=pi
mkdir -p /home/$DEFAULT_USER

sudo apt update
sudo apt upgrade -y

sudo rm -f /usr/lib/python3.*/EXTERNALLY-MANAGED
pip install wandb

# Install hailo
sudo apt full-upgrade -y
sudo apt install -y hailo-all

source /opt/ros/jazzy/setup.bash

apt install git-lfs -y
git lfs install
retry_command "git clone https://github.com/Nate711/pupperv3-monorepo.git --recurse-submodules"

# Install dependencies
cd /home/$DEFAULT_USER/pupperv3-monorepo/ros2_ws
mv /home/$DEFAULT_USER/ros2_ws/src/* /home/$DEFAULT_USER/pupperv3-monorepo/ros2_ws/src/
rm -rf /home/$DEFAULT_USER/ros2_ws

./build.sh


# pip install rosdep
# sudo rosdep init && rosdep update
# rosdep install --from-paths src -y --ignore-src


# # Install utils
# cd /home/$DEFAULT_USER
# rm -rf utils
# git clone https://github.com/Nate711/utils.git -b launch_neural_controller
# bash /home/$DEFAULT_USER/utils/install_battery_monitor.sh
# # bash /home/$DEFAULT_USER/utils/install_robot_auto_start_service.sh



