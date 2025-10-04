#!/usr/bin/env bash
set -euo pipefail

# Resolve this script's directory, even if called via symlink
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
BAGS_DIR="$SCRIPT_DIR/../bags"
mkdir -p "$BAGS_DIR"

# Start recording: all topics except /camera/image_raw, but keep /camera/image_raw/compressed
# The output will be created under ../bags with a timestamped folder name
ros2 bag record -a --exclude-regex '^/camera/image_raw$' -o "$BAGS_DIR/rosbag2_$(date +%Y%m%d_%H%M%S)"
