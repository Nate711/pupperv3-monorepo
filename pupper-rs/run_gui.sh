#!/bin/bash

# Set up environment for GUI
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0

# Ensure we're in the correct directory
cd /home/pi/pupperv3-monorepo/pupper-rs

# Check if binary exists and is newer than source files
BINARY="./target/release/pupper-rs"

# Run the pre-built binary
exec "$BINARY"