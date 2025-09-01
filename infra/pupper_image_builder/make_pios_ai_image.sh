#!/bin/bash -e

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
  PACKER_ONLY="ai.arm.with-keys"
  if [ ! -f ".env.local" ]; then
    echo "Warning: --include-keys passed but .env.local not found; proceeding without keys."
    PACKER_ONLY="ai.arm.raspbian"
  else
    echo "Including .env.local in image (ai_with_keys)."
  fi
fi

docker pull mkaczanowski/packer-builder-arm:latest
docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest init pios_ai_arm64.pkr.hcl
docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest build -only=$PACKER_ONLY pios_ai_arm64.pkr.hcl

# If including keys, rename the resulting image with _WITH_SECRETS suffix
if [ "$INCLUDE_KEYS" = true ] && [ -f "pupOS_pios_ai.img" ]; then
  mv -f pupOS_pios_ai.img pupOS_pios_ai_WITH_SECRETS.img
  echo "Renamed image to pupOS_pios_ai_WITH_SECRETS.img because --include-keys was used"
fi
