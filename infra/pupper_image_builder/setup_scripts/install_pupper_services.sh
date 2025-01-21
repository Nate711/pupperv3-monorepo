#!/bin/bash -e

set -x

DEFAULT_USER=pi

bash /home/$DEFAULT_USER/utils/install_battery_monitor.sh
bash /home/$DEFAULT_USER/utils/install_robot_auto_start_service.sh
