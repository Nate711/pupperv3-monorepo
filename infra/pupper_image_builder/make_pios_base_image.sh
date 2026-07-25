#!/bin/bash -e

set -x

# Get shortened git commit hash
GIT_COMMIT_SHORT=$(git rev-parse --short HEAD 2>/dev/null || echo "unknown")

docker pull mkaczanowski/packer-builder-arm:latest
docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest init pios_base_arm64.pkr.hcl
docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest build pios_base_arm64.pkr.hcl

# Rename the output image to include git commit hash and keep a real-file copy
# under the canonical name. Do NOT symlink: packer's local-file fetch turns a
# symlinked source into a symlinked output image, so the full build's
# provisioning writes through and corrupts the versioned base image.
if [ -f "pupOS_pios_base.img" ]; then
  mv -f "pupOS_pios_base.img" "pupOS_pios_base_${GIT_COMMIT_SHORT}.img"
  echo "Image saved as pupOS_pios_base_${GIT_COMMIT_SHORT}.img"
  cp "pupOS_pios_base_${GIT_COMMIT_SHORT}.img" "pupOS_pios_base.img"
  echo "Copied pupOS_pios_base_${GIT_COMMIT_SHORT}.img to pupOS_pios_base.img"
fi