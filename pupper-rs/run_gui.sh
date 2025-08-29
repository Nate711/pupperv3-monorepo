#!/bin/bash

# Set up environment for GUI
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0

# Ensure we're in the correct directory
cd /home/pi/pupperv3-monorepo/pupper-rs

# Build and run the GUI in release mode
exec /home/pi/.cargo/bin/cargo run --release