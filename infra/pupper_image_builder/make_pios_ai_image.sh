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

#### Parse command line arguments
INCLUDE_KEYS=false
for arg in "$@"; do
  case $arg in
    --include-keys)
      INCLUDE_KEYS=true
      shift
      ;;
  esac
done

# Check if the image exists
if [ ! -f "pupOS_pios_full.img" ]; then
  echo "Image not found. Running make_pios_full_image.sh..."
  ./make_pios_full_image.sh
else
  echo "Image found. Skipping full image creation."
fi

#### Select which build to run
PACKER_ONLY="ai.arm.raspbian"
if [ "$INCLUDE_KEYS" = true ]; then
  if [ ! -f ".env.local" ]; then
    echo "Error: --include-keys was passed but .env.local is missing in infra/pupper_image_builder/" >&2
    exit 1
  fi
  PACKER_ONLY="ai.arm.with-keys"
  echo "Including .env.local in image (ai_with_keys)."
fi

docker pull mkaczanowski/packer-builder-arm:latest
docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build "${DOCKER_ENV_ARGS[@]}" mkaczanowski/packer-builder-arm:latest init pios_ai_arm64.pkr.hcl
docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build "${DOCKER_ENV_ARGS[@]}" mkaczanowski/packer-builder-arm:latest build -only=$PACKER_ONLY pios_ai_arm64.pkr.hcl

# If including keys, rename the resulting image with _WITH_SECRETS suffix
if [ "$INCLUDE_KEYS" = true ] && [ -f "pupOS_pios_ai.img" ]; then
  mv -f pupOS_pios_ai.img pupOS_pios_ai_WITH_SECRETS.img
  echo "Renamed image to pupOS_pios_ai_WITH_SECRETS.img because --include-keys was used"
fi
