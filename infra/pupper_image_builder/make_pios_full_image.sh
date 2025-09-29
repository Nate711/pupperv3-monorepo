#!/bin/bash -e

load_env_if_present() {
  local env_file="$1"
  if [ -f "$env_file" ]; then
    set +x
    set -o allexport
    # shellcheck disable=SC1090
    source "$env_file"
    set +o allexport
    set -x
  fi
}

run_packer_container() {
  local args=("$@")
  if [ -n "${GITHUB_TOKEN:-}" ]; then
    set +x
    docker run --rm --privileged -v /dev:/dev -v "${PWD}":/build mkaczanowski/packer-builder-arm:latest "${args[@]}"
    set -x
  else
    docker run --rm --privileged -v /dev:/dev -v "${PWD}":/build mkaczanowski/packer-builder-arm:latest "${args[@]}"
  fi
}

set -x

load_env_if_present ".env.local"

# Get shortened git commit hash
GIT_COMMIT_SHORT=$(git rev-parse --short HEAD 2>/dev/null || echo "unknown")

# Check if the base image exists (either as symlink or actual file)
if [ ! -e "pupOS_pios_base.img" ]; then
  echo "Base image not found. Running make_pios_base_image.sh..."
  ./make_pios_base_image.sh
else
  echo "Base image found. Skipping base image creation."
fi

docker pull mkaczanowski/packer-builder-arm:latest
run_packer_container init pios_full_arm64.pkr.hcl

PACKER_BUILD_ARGS=()
if [ -n "${GITHUB_TOKEN:-}" ]; then
  PACKER_BUILD_ARGS+=(-var "github_token=${GITHUB_TOKEN}")
fi

run_packer_container build "${PACKER_BUILD_ARGS[@]}" pios_full_arm64.pkr.hcl

# !Packer cannot properly handle symlink images as source images!
# Rename the output image to include git commit hash and create symlink
# if [ -f "pupOS_pios_full.img" ]; then
#   mv -f "pupOS_pios_full.img" "pupOS_pios_full_${GIT_COMMIT_SHORT}.img"
#   echo "Image saved as pupOS_pios_full_${GIT_COMMIT_SHORT}.img"
#   # Create symlink for next build step
#   ln -sf "pupOS_pios_full_${GIT_COMMIT_SHORT}.img" "pupOS_pios_full.img"
#   echo "Created symlink pupOS_pios_full.img -> pupOS_pios_full_${GIT_COMMIT_SHORT}.img"
# fi

# Add a metadata file to indicate git commit
if [ -f "pupOS_pios_full.img" ]; then
  DATE_SUFFIX=$(date +"%Y%m%d%H%M%S")
  # Remove old metadata files
  find . -maxdepth 1 -type f -name 'pupOS_pios_full_*' ! -name '*.*' -exec rm -f {} +
  # Add new metadata file
  touch "pupOS_pios_full_${GIT_COMMIT_SHORT}_${DATE_SUFFIX}"
fi