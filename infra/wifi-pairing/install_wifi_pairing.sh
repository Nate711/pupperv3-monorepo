#!/bin/bash
#
# Install headless WiFi pairing (comitup captive portal) + the Pupper
# customizations on a Raspberry Pi. Safe to re-run (idempotent): existing
# system files are backed up as *.orig on first install and then overwritten.
#
# Run as the 'pi' user; it uses sudo for the privileged steps.
#
# What it sets up:
#   - comitup: broadcasts a "Pupper-Setup-<nnn>" hotspot when no known WiFi is
#     reachable, so you can join a new network from a phone (no keyboard).
#   - comitup-web customization: a "Saved networks & priority" page.
#   - pupper-pairing helper + sudoers: lets the GUI toggle pairing mode.
#   - NetworkManager.py loopback patch: fixes the comitup crash loop on NM 1.42+.
#
# The pupper-gui "WiFi" button itself ships with pupper-rs (build + install that
# separately); this script sets up everything it talks to.
#
set -euo pipefail

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
WEB=/usr/share/comitup/web

echo "== 1/7 installing comitup =="
sudo apt-get update -qq
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y comitup

echo "== 2/7 comitup config =="
sudo cp -n /etc/comitup.conf /etc/comitup.conf.orig 2>/dev/null || true
sudo install -m 0644 "$SCRIPT_DIR/comitup.conf" /etc/comitup.conf

echo "== 3/7 comitup-web customization (saved-network priority page) =="
for f in comitupweb.py netpriority.py templates/index.html templates/known.html; do
    dest="$WEB/$f"
    sudo cp -n "$dest" "$dest.orig" 2>/dev/null || true   # back up originals once
    sudo install -m 0644 "$SCRIPT_DIR/comitup-web/$f" "$dest"
done
sudo rm -f "$WEB/__pycache__/comitupweb.cpython-"*.pyc

echo "== 4/7 pupper-pairing helper + sudoers =="
sudo install -m 0755 "$SCRIPT_DIR/pupper-pairing" /usr/local/bin/pupper-pairing
sudo install -m 0440 -o root -g root "$SCRIPT_DIR/sudoers.d/pupper-pairing" /etc/sudoers.d/pupper-pairing
sudo visudo -cf /etc/sudoers.d/pupper-pairing

echo "== 5/7 QR-pairing decode deps (libzbar0 + pyzbar) =="
sudo apt-get install -y libzbar0
# pyzbar is imported by the ROS detection node to decode WiFi QR codes. ROS runs
# on the system Python (not uv-managed), so install it there.
sudo pip install --break-system-packages pyzbar

echo "== 6/7 NetworkManager loopback patch (fixes comitup crash loop on NM 1.42+) =="
sudo python3 "$SCRIPT_DIR/patch_networkmanager_loopback.py"

echo "== 7/7 enable comitup =="
sudo systemctl daemon-reload
sudo systemctl enable comitup.service
sudo systemctl restart comitup.service

echo
echo "Done. comitup is enabled and will manage WiFi on next boot."
echo "Setup hotspot: 'Pupper-Setup-<nnn>'  (password is set in /etc/comitup.conf)."
echo "The GUI 'WiFi' pairing button ships with pupper-rs — build/install that separately."
