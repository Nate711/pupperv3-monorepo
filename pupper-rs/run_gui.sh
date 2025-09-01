#!/bin/bash

# Set up environment for GUI
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0

# Ensure we're in the correct directory
cd /home/pi/pupperv3-monorepo/pupper-rs

# Prefer aarch64 cross-compiled binary, fallback to local release
if [ -x "./target/aarch64-unknown-linux-gnu/release/pupper-rs" ]; then
  BINARY="./target/aarch64-unknown-linux-gnu/release/pupper-rs"
elif [ -x "./target/release/pupper-rs" ]; then
  BINARY="./target/release/pupper-rs"
else
  echo "Error: pupper-rs binary not found in ./target/aarch64-unknown-linux-gnu/release or ./target/release" >&2
  exit 1
fi

# Run the chosen binary
exec "$BINARY"
