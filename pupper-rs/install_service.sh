#!/bin/bash

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
sudo systemctl enable ${SCRIPT_DIR}/pupper-gui.service