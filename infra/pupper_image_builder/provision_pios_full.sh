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
chown -R $DEFAULT_USER /home/$DEFAULT_USER

sudo apt update
sudo apt upgrade -y

sudo apt install -y vim

sudo rm -f /usr/lib/python3.*/EXTERNALLY-MANAGED
pip install wandb sounddevice openai[realtime] pydub pyaudio black supervision opencv-python loguru

# Install hailo
yes N | sudo DEBIAN_FRONTEND=noninteractive apt full-upgrade -y

sudo apt install -y hailo-all

# Source ros2
source /opt/ros/jazzy/setup.bash


############################ Prepare monorepo ###############################################

# Prepare monorepo
apt install git-lfs -y
cd /home/$DEFAULT_USER
retry_command "git clone https://github.com/Nate711/pupperv3-monorepo.git --recurse-submodules"
cd /home/$DEFAULT_USER/pupperv3-monorepo/
git lfs install
git lfs pull

############################## Install ros2 deps from source ##################################

# Create directory for ros2 depenedencies we need to build from source
mkdir /home/$DEFAULT_USER/pupperv3-monorepo/ros2_ws/src/common
cd /home/$DEFAULT_USER/pupperv3-monorepo/ros2_ws/src/common

# install libcap-dev
sudo apt install -y libcap-dev

# install dependencies for foxglove-bridge
sudo apt-get install -y libwebsocketpp-dev nlohmann-json3-dev

# install dependencies for camera_ros
sudo apt-get install -y libcamera-dev

pip install typeguard
pip uninstall em
pip install empy==3.3.4

repos=(
    "https://github.com/facontidavide/rosx_introspection.git"
    "https://github.com/foxglove/ros-foxglove-bridge.git"
    "https://github.com/christianrauch/camera_ros.git"
    "https://github.com/ros-perception/vision_msgs.git"
)

for repo in "${repos[@]}"; do
    retry_command "git clone $repo --recurse-submodules"
done

############################### Build everything #############################################

# Build monorepo ros2 code
bash /home/$DEFAULT_USER/pupperv3-monorepo/ros2_ws/build.sh

############### AUTOMATICALLY SOURCE ROS2 WORKSPACE IN BASHRC ################################
echo "source /home/$DEFAULT_USER/pupperv3-monorepo/ros2_ws/install/setup.bash" >> /home/$DEFAULT_USER/.bashrc
echo 'export RCUTILS_COLORIZED_OUTPUT=1' >> /home/$DEFAULT_USER/.bashrc

# Install utils
bash /home/$DEFAULT_USER/pupperv3-monorepo/robot/utils/install_battery_monitor.sh
bash /home/$DEFAULT_USER/pupperv3-monorepo/robot/utils/install_robot_auto_start_service.sh



