#!/bin/bash

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
sudo ln -s ${SCRIPT_DIR}/pupper-gui.service /etc/systemd/system/pupper-gui.service
sudo systemctl enable pupper-gui.service