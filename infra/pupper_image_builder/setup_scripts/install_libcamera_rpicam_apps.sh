#!/bin/bash -e

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

DEFAULT_USER=pi
cd /home/$DEFAULT_USER

######## Install libcamera ########
sudo apt install -y qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5 libepoxy-dev
sudo apt install -y libavcodec-dev libavdevice-dev libavformat-dev libswresample-dev

sudo apt install -y libboost-dev
sudo apt install -y libgnutls28-dev openssl libtiff5-dev pybind11-dev
sudo apt install -y qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5
sudo apt install -y meson cmake
sudo apt install -y python3-yaml python3-ply
sudo apt install -y libglib2.0-dev libgstreamer-plugins-base1.0-dev

# Check if the folder already exists
if [ ! -d "libcamera" ]; then
    retry_command "git clone --recurse-submodules https://github.com/raspberrypi/libcamera.git" 20
else
    echo "Folder 'libcamera' already exists. Skipping clone."
fi

cd libcamera

meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled -Dwerror=false
# Add GST_PLUGIN_PATH to .bashrc if not already present
if ! grep -q "export GST_PLUGIN_PATH=/home/pi/libcamera/build/src/gstreamer" /home/$DEFAULT_USER/.bashrc; then
    echo "export GST_PLUGIN_PATH=/home/pi/libcamera/build/src/gstreamer" >> /home/$DEFAULT_USER/.bashrc
    echo "Added GST_PLUGIN_PATH to ~/.bashrc"
else
    echo "GST_PLUGIN_PATH is already set in ~/.bashrc"
fi

# Source .bashrc to apply changes immediately
source /home/$DEFAULT_USER/.bashrc

ninja -C build
sudo ninja -C build install


####### Build rpicam-apps #######
# TODO: figure out why this fails due to not finding libpng
# sudo apt install -y cmake libboost-program-options-dev libdrm-dev libexif-dev
# sudo apt install -y meson ninja-build
# cd ..
# if [ ! -d "rpicam-apps" ]; then
#     retry_command "git clone --recurse-submodules https://github.com/raspberrypi/rpicam-apps.git" 20
# else
#     echo "Folder 'rpicam-apps' already exists. Skipping clone."
# fi

# cd rpicam-apps
# # TODO: enable hailo -Denable_hailo=disabled for image builder image
# meson setup --wipe build -Denable_libav=enabled -Denable_drm=enabled -Denable_egl=enabled -Denable_qt=enabled -Denable_opencv=disabled -Denable_tflite=disabled -Dwerror=false -Denable_imx500=false -Ddownload_imx500_models=false
# meson compile -C build
# sudo meson install -C build
# sudo ldconfig
