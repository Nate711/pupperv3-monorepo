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

# Check if the image exists
if [ ! -f "pupOS_pios_base.img" ]; then
  echo "Image not found. Running make_pios_base_image.sh..."
  ./make_pios_base_image.sh
else
  echo "Image found. Skipping base image creation."
fi

docker pull mkaczanowski/packer-builder-arm:latest
run_packer_container init pios_full_arm64.pkr.hcl

PACKER_BUILD_ARGS=()
if [ -n "${GITHUB_TOKEN:-}" ]; then
  PACKER_BUILD_ARGS+=(-var "github_token=${GITHUB_TOKEN}")
fi

run_packer_container build "${PACKER_BUILD_ARGS[@]}" pios_full_arm64.pkr.hcl