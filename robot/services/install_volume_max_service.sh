#!/bin/bash

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
sudo ln -s ${SCRIPT_DIR}/volume-max.service /etc/systemd/system/volume-max.service
sudo systemctl daemon-reload
sudo systemctl enable volume-max.service
sudo systemctl start volume-max.service