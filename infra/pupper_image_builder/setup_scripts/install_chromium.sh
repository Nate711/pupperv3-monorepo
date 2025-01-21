#!/bin/bash -e

set -x

sudo add-apt-repository ppa:xtradeb/apps -y
sudo apt update
sudo apt install -y chromium