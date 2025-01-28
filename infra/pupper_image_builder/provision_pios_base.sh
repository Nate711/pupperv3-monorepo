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

cat /etc/hostname
cat /etc/hosts
hostname

DEFAULT_USER=pi
mkdir -p /home/$DEFAULT_USER

# PiOS debian ros2
sudo apt update
retry_command "wget https://s3.ap-northeast-1.wasabisys.com/download-raw/dpkg/ros2-desktop/debian/bookworm/ros-jazzy-desktop-0.3.2_20240525_arm64.deb"
sudo apt install -y ./ros-jazzy-desktop-0.3.2_20240525_arm64.deb
sudo rm -f /usr/lib/python3.*/EXTERNALLY-MANAGED
sudo pip install vcstool colcon-common-extensions
echo 'source /opt/ros/jazzy/setup.bash' >> /home/$DEFAULT_USER/.bashrc
source /opt/ros/jazzy/setup.bash

sudo apt update

# Setup for Raspberry Pi 5
# Stuff at the top of the config.txt
echo 'dtparam=spi=on' | sudo tee -a /boot/firmware/config.txt
echo 'dtparam=i2c_arm=on,i2c_arm_baudrate=100000' | sudo tee -a /boot/firmware/config.txt
echo 'usb_max_current_enable=1' | sudo tee -a /boot/firmware/config.txt

# At the bottom of config.txt
echo 'dtoverlay=waveshare-4dpic-3b' >> /boot/firmware/config.txt
echo 'dtoverlay=waveshare-4dpic-4b' >> /boot/firmware/config.txt
echo 'dtoverlay=waveshare-4dpic-5b' >> /boot/firmware/config.txt
echo 'hdmi_force_hotplug=1' >> /boot/firmware/config.txt
echo 'config_hdmi_boost=10' >> /boot/firmware/config.txt
echo 'hdmi_group=2' >> /boot/firmware/config.txt
echo 'hdmi_mode=87' >> /boot/firmware/config.txt
echo 'hdmi_timings=720 0 100 20 100 720 0 20 8 20 0 0 0 60 0 48000000 6' >> /boot/firmware/config.txt
echo 'start_x=0' >> /boot/firmware/config.txt
echo 'gpu_mem=128' >> /boot/firmware/config.txt
echo 'dtoverlay=hifiberry-dac' >> /boot/firmware/config.txt
sed -i '1s/^/video=HDMI-A-1:720x720M@60D,rotate=270 /' /boot/firmware/cmdline.txt

# Download and extract the display overlays
retry_command "wget 'https://files.waveshare.com/wiki/4inch%20HDMI%20LCD%20(C)/4HDMIB_DTBO.zip' -O 4HDMIB_DTBO.zip"
sudo apt install -y unzip
unzip 4HDMIB_DTBO.zip
sudo cp 4HDMIB_DTBO/*.dtbo /boot/firmware/overlays/
rm -r 4HDMIB_DTBO 4HDMIB_DTBO.zip

# Set up avahi-daemon
sudo apt install -y avahi-daemon net-tools openssh-server curl

# TODO: figure out if regular kernel is ok for RL control
# Install low-latency kernel
# sudo wget https://github.com/raspberrypi/firmware/raw/master/boot/bcm2712-rpi-5-b.dtb -P /etc/flash-kernel/dtbs/
# sudo apt install -y linux-lowlatency

# # Adafruit GPIO setup
sudo apt install -y python-is-python3 python3-pip i2c-tools libgpiod-dev python3-libgpiod

sudo rm -f /usr/lib/python3.*/EXTERNALLY-MANAGED
pip install Adafruit-Blinka RPi.GPIO

# Bluetooth
sudo apt install -y bluez

# Audio
sudo apt install -y portaudio19-dev python3-pyaudio alsa-utils
pip install --upgrade pyaudio deepgram-sdk

# Install ROS2
sudo apt install -y software-properties-common

# Create ROS2 workspace
mkdir -p /home/$DEFAULT_USER/ros2_ws/src
cd /home/$DEFAULT_USER/ros2_ws/src

# install libcap-dev
sudo apt install -y libcap-dev

pip install typeguard
pip uninstall em
pip install empy==3.3.4
retry_command "git clone https://github.com/pal-robotics/backward_ros.git --recurse-submodules"
retry_command "git clone https://github.com/PickNikRobotics/RSL.git --recurse-submodules"
retry_command "git clone https://github.com/PickNikRobotics/generate_parameter_library.git --recurse-submodules"
retry_command "git clone https://github.com/ros-controls/realtime_tools.git --recurse-submodules"
retry_command "git clone https://github.com/ros-controls/control_msgs.git --recurse-submodules"
retry_command "git clone https://github.com/ros/diagnostics.git --recurse-submodules"
retry_command "git clone https://github.com/ros2/teleop_twist_joy.git --recurse-submodules"
retry_command "git clone -b ros2 https://github.com/ros-drivers/joystick_drivers.git --recurse-submodules"
retry_command "git clone -b ros2 https://github.com/ros/xacro.git --recurse-submodules"
retry_command "git clone https://github.com/ros-controls/ros2_control.git --recurse-submodules"
retry_command "git clone https://github.com/PickNikRobotics/cpp_polyfills.git --recurse-submodules"


# Install dependencies
cd /home/$DEFAULT_USER/ros2_ws
pip install rosdep
# sudo rosdep init && rosdep update
# rosdep install --from-paths src -y --ignore-src

# Install additional ROS2 packages
# sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-teleop-twist-joy ros-jazzy-foxglove-bridge ros-jazzy-xacro

# Build ROS2 workspace
# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
# echo 'source ~/ros2_ws/install/setup.bash' >> /home/$DEFAULT_USER/.bashrc
# source /home/$DEFAULT_USER/ros2_ws/install/setup.bash

# Finally update packages since this step takes a long time
sudo apt upgrade -y