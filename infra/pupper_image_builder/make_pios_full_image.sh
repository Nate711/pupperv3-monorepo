#!/bin/bash -e

set -x

# Check if the image exists
if [ ! -f "pupOS_pios_base.img" ]; then
  echo "Image not found. Running make_pios_base_image.sh..."
  ./make_pios_base_image.sh
else
  echo "Image found. Skipping base image creation."
fi

docker pull mkaczanowski/packer-builder-arm:latest
docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest init pios_full_arm64.pkr.hcl
docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest build pios_full_arm64.pkr.hcl