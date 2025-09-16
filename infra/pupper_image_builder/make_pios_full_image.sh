#!/bin/bash -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

ENV_FILE="${SCRIPT_DIR}/.env.local"
if [ -f "$ENV_FILE" ]; then
  set -a
  # shellcheck disable=SC1090
  source "$ENV_FILE"
  set +a
fi

DOCKER_ENV_ARGS=()
if [ -n "${GITHUB_TOKEN:-}" ]; then
  DOCKER_ENV_ARGS+=(--env GITHUB_TOKEN)
fi

set -x

# Check if the image exists
if [ ! -f "pupOS_pios_base.img" ]; then
  echo "Image not found. Running make_pios_base_image.sh..."
  ./make_pios_base_image.sh
else
  echo "Image found. Skipping base image creation."
fi

docker pull mkaczanowski/packer-builder-arm:latest
docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build "${DOCKER_ENV_ARGS[@]}" mkaczanowski/packer-builder-arm:latest init pios_full_arm64.pkr.hcl
docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build "${DOCKER_ENV_ARGS[@]}" mkaczanowski/packer-builder-arm:latest build pios_full_arm64.pkr.hcl