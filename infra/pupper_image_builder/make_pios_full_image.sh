#!/bin/bash -e

set -x

# Parse command line arguments
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
if [ ! -f "pupOS_pios_base.img" ]; then
  echo "Image not found. Running make_pios_base_image.sh..."
  ./make_pios_base_image.sh
else
  echo "Image found. Skipping base image creation."
fi

# Build Packer variable arguments
PACKER_VARS=""
if [ "$INCLUDE_KEYS" = true ]; then
  if [ -f ".env.local" ]; then
    # Source the .env.local file to load variables
    set -a
    source .env.local
    set +a
    
    if [ -n "$OPENAI_API_KEY" ]; then
      PACKER_VARS="$PACKER_VARS -var OPENAI_API_KEY=$OPENAI_API_KEY"
    fi
    if [ -n "$CARTESIA_API_KEY" ]; then
      PACKER_VARS="$PACKER_VARS -var CARTESIA_API_KEY=$CARTESIA_API_KEY"
    fi
    if [ -n "$GOOGLE_API_KEY" ]; then
      PACKER_VARS="$PACKER_VARS -var GOOGLE_API_KEY=$GOOGLE_API_KEY"
    fi
    if [ -n "$LIVEKIT_URL" ]; then
      PACKER_VARS="$PACKER_VARS -var LIVEKIT_URL=$LIVEKIT_URL"
    fi
    if [ -n "$LIVEKIT_API_KEY" ]; then
      PACKER_VARS="$PACKER_VARS -var LIVEKIT_API_KEY=$LIVEKIT_API_KEY"
    fi
    if [ -n "$LIVEKIT_API_SECRET" ]; then
      PACKER_VARS="$PACKER_VARS -var LIVEKIT_API_SECRET=$LIVEKIT_API_SECRET"
    fi
    if [ -n "$DEEPGRAM_API_KEY" ]; then
      PACKER_VARS="$PACKER_VARS -var DEEPGRAM_API_KEY=$DEEPGRAM_API_KEY"
    fi
    if [ -n "$ELEVEN_API_KEY" ]; then
      PACKER_VARS="$PACKER_VARS -var ELEVEN_API_KEY=$ELEVEN_API_KEY"
    fi
    echo "Including API keys from .env.local file"
  else
    echo "Warning: .env.local file not found. No API keys will be included."
  fi
fi

docker pull mkaczanowski/packer-builder-arm:latest
docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest init pios_full_arm64.pkr.hcl
docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest build $PACKER_VARS pios_full_arm64.pkr.hcl