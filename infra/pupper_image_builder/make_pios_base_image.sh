#!/bin/bash -e

set -x

# Get shortened git commit hash
GIT_COMMIT_SHORT=$(git rev-parse --short HEAD 2>/dev/null || echo "unknown")

docker pull mkaczanowski/packer-builder-arm:latest
docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest init pios_base_arm64.pkr.hcl
docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest build pios_base_arm64.pkr.hcl

# Rename the output image to include git commit hash and create symlink
if [ -f "pupOS_pios_base.img" ]; then
  mv -f "pupOS_pios_base.img" "pupOS_pios_base_${GIT_COMMIT_SHORT}.img"
  echo "Image saved as pupOS_pios_base_${GIT_COMMIT_SHORT}.img"
  # Create symlink for next build step
  ln -sf "pupOS_pios_base_${GIT_COMMIT_SHORT}.img" "pupOS_pios_base.img"
  echo "Created symlink pupOS_pios_base.img -> pupOS_pios_base_${GIT_COMMIT_SHORT}.img"
fi