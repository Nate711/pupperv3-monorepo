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
chown -R $DEFAULT_USER /home/$DEFAULT_USER

# PiOS debian ros2
sudo apt update
# retry_command "wget https://s3.ap-northeast-1.wasabisys.com/download-raw/dpkg/ros2-desktop/debian/bookworm/ros-jazzy-desktop-0.3.2_20240525_arm64.deb"
# sudo apt install -y ./ros-jazzy-desktop-0.3.2_20240525_arm64.deb
sudo apt install -y /home/pi/resources/ros.deb

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

# Set up touch screen and HDMI settings
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

# Try to set rotation settings
sed -i '1s/^/video=HDMI-A-1:720x720M@60D,rotate=270 /' /boot/firmware/cmdline.txt

# Enable speaker
echo 'dtoverlay=hifiberry-dac' >> /boot/firmware/config.txt


# Set up firstboot script
# NOT WORKING RIGHT NOW
# sed -i '1s|$| init=/usr/lib/raspberrypi-sys-mods/firstboot systemd.run=/boot/firstrun.sh systemd.run_success_action=reboot systemd.unit=kernel-command-line.target|' /boot/firmware/cmdline.txt

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

# Install some useful tools
sudo apt install -y software-properties-common vim

# Finally update packages since this step takes a long time
export DEBIAN_FRONTEND=noninteractive
export APT_LISTCHANGES_FRONTEND=none
# (Optional) avoid services trying to start in chroot
printf '#!/bin/sh\nexit 101\n' > /usr/sbin/policy-rc.d && chmod +x /usr/sbin/policy-rc.d

sudo apt-get update

apt-get -y \
  -o Dpkg::Options::="--force-confdef" \
  -o Dpkg::Options::="--force-confold" \
  upgrade

rm -f /usr/sbin/policy-rc.d