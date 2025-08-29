#!/bin/bash

# Set up environment for GUI
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0

# Ensure we're in the correct directory
cd /home/pi/pupperv3-monorepo/pupper-rs

# Check if binary exists and is newer than source files
BINARY="./target/release/pupper-rs"
if [ ! -f "$BINARY" ] || find src -name "*.rs" -newer "$BINARY" | grep -q .; then
    echo "Building pupper-rs..."
    /home/pi/.cargo/bin/cargo build --release
fi

# Run the pre-built binary
exec "$BINARY"