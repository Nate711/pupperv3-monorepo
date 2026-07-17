#!/bin/bash
#
# One-command deploy of headless WiFi pairing to a pupper already running this
# repo. Does both halves: the system side (comitup + comitup-web + the
# pupper-pairing helper + the NetworkManager loopback fix) and the GUI side
# (builds pupper-rs, which carries the on-screen "WiFi" pairing button, and
# restarts the service).
#
# Run as the 'pi' user; it uses sudo for the privileged steps. Idempotent.
#
# For a FRESH image, this is already wired into the image builder
# (infra/pupper_image_builder). Use this script to update puppers in the field.
#
set -euo pipefail

REPO="$(dirname "$(readlink -f "$0")")"
GUI="$REPO/pupper-rs"

echo "=== [1/3] system WiFi pairing (comitup + patches + helper) ==="
"$REPO/infra/wifi-pairing/install_wifi_pairing.sh"

echo "=== [2/3] building pupper-gui (the on-screen WiFi button) ==="
if ! command -v cargo >/dev/null 2>&1; then
    echo "ERROR: 'cargo' is not on PATH in this shell." >&2
    echo "  Rust is required to build the GUI. Open a normal login shell" >&2
    echo "  (so ~/.cargo/env is sourced) and re-run this script." >&2
    exit 1
fi
# Native target on an aarch64 Pi; this is the path run_gui.sh prefers.
( cd "$GUI" && cargo build --release --target aarch64-unknown-linux-gnu )

echo "=== [3/3] (re)starting pupper-gui service ==="
if ! systemctl cat pupper-gui.service >/dev/null 2>&1; then
    echo "pupper-gui.service not registered — installing it"
    bash "$GUI/install_service.sh"
    sudo systemctl daemon-reload
fi
sudo systemctl restart pupper-gui.service

echo
echo "Done. WiFi pairing deployed:"
printf "  comitup:    %s (enabled -> auto on boot)\n" "$(systemctl is-active comitup 2>&1)"
printf "  pupper-gui: %s\n" "$(systemctl is-active pupper-gui 2>&1)"
echo "The 'Pupper-Setup-<nnn>' hotspot comes up automatically when no known"
echo "WiFi is in range, and the GUI 'WiFi' button toggles pairing mode manually."
