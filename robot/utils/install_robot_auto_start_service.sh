SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
sudo ln -s ${SCRIPT_DIR}/robot.service /etc/systemd/system/robot.service
sudo ln -s ${SCRIPT_DIR}/change_tty1_permissions.service /etc/systemd/system/change_tty1_permissions.service
sudo systemctl enable change_tty1_permissions
sudo systemctl enable robot