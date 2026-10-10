# Utils
Utils for pupper v3. Make sure you clone to /home/pi/

# Usage
To install battery monitor service
`./install_battery_monitor.sh`

The battery monitor also watches the Pi 5's 5V input and warns (via `wall`) below 4.85V, before the ~4.63V firmware undervoltage flag. It logs the minimum 5V reading every minute: `journalctl -u battery_monitor -f`

To install auto start service
`./install_robot_auto_start_service.sh`
