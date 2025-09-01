#!/bin/bash

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
sudo ln -s ${SCRIPT_DIR}/llm-agent.service /etc/systemd/system/llm-agent.service
sudo systemctl enable llm-agent.service
