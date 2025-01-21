#!/bin/bash -e

set -x

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

export DEBIAN_FRONTEND=noninteractive

DEFAULT_USER=pi
mkdir -p /home/$DEFAULT_USER
mkdir -p /home/$DEFAULT_USER/resources

# Make sure bashrc is sourced when SSHing
echo -e 'if [ -f ~/.bashrc ]; then\n    source ~/.bashrc\nfi' >> "/home/$DEFAULT_USER/.bash_profile"

# Update packages
sudo apt update

# Setup for Raspberry Pi 5
echo 'dtparam=i2c_arm=on,i2c_arm_baudrate=100000' >> /boot/firmware/config.txt
echo 'usb_max_current_enable=1' >> /boot/firmware/config.txt
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
echo 'dtdebug=1' >> /boot/firmware/config.txt
sed -i '1s/^/video=HDMI-A-1:720x720M@60D,rotate=270 /' /boot/firmware/cmdline.txt

# Download and extract the display overlays
retry_command "wget 'https://files.waveshare.com/wiki/4inch%20HDMI%20LCD%20(C)/4HDMIB_DTBO.zip' -O 4HDMIB_DTBO.zip" 20
sudo apt install -y unzip
unzip 4HDMIB_DTBO.zip
sudo cp 4HDMIB_DTBO/*.dtbo /boot/firmware/overlays/
rm -r 4HDMIB_DTBO 4HDMIB_DTBO.zip

# Set up avahi-daemon
sudo apt install -y avahi-daemon net-tools openssh-server curl

# Install gpiod for GPIO manipulation
sudo apt install -y python-is-python3 python3-pip i2c-tools libgpiod-dev python3-libgpiod build-essential
sudo rm -f /usr/lib/python3.*/EXTERNALLY-MANAGED
pip install Adafruit-Blinka RPi.GPIO
sudo pip install Adafruit-Blinka RPi.GPIO

# Install gpiozero for alternative GPIO manipulation
sudo apt install -y python3-rpi-lgpio
sudo rm -f /usr/lib/python3.*/EXTERNALLY-MANAGED
pip install gpiozero

# Bluetooth
sudo apt install -y bluez

# Audio
sudo apt install -y portaudio19-dev python3-pyaudio alsa-utils
pip install --upgrade pyaudio deepgram-sdk

# Install web browser
# TODO FIX THIS # sudo apt install chromium-browser -y

# Disable setup wizard
mkdir /home/$DEFAULT_USER/.config
echo "yes" >> /home/$DEFAULT_USER/.config/gnome-initial-setup-done
