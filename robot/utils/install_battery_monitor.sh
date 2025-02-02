SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
sudo ln -s ${SCRIPT_DIR}/battery_monitor.service /etc/systemd/system/battery_monitor.service
sudo systemctl enable battery_monitor