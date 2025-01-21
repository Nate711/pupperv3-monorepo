#!/bin/bash -e

set -x

sudo apt install -y build-essential linux-headers-*raspi
pip install /home/pi/resources/hailort-4.19.0-cp312-cp312-linux_aarch64.whl
pip install numpy Pillow supervision tqdm loguru
