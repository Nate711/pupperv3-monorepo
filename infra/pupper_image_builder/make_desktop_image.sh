#!/bin/bash -e

set -x

# Check if the image exists
if [ ! -f "pupOS_ubuntu_desktop_ros_base.img" ]; then
  echo "Image not found. Running make_base_image.sh..."
  ./make_base_image.sh
else
  echo "Image found. Skipping base image creation."
fi

# Continue with the original Docker commands
docker pull mkaczanowski/packer-builder-arm:latest
docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest init ubuntu_desktop_full_24.04_arm64.pkr.hcl
docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest build ubuntu_desktop_full_24.04_arm64.pkr.hcl
