#!/bin/bash
#
# Start Pupper's main services. If a service isn't registered with systemd yet
# (fresh Pi, or the unit was never installed), register it first via that
# service's own install script, then start it. Prints a status summary so a
# failed start is visible rather than silent.
#
set -uo pipefail

MONOREPO="$(dirname "$(readlink -f "$0")")"

# Start order matters (robot brings up ros2_control before the GUI/agent).
SERVICES=(robot llm-agent pupper-gui)

# Each unit -> the install script that symlinks it into systemd and enables it.
installer_for() {
    case "$1" in
        robot)      echo "$MONOREPO/robot/utils/install_robot_auto_start_service.sh" ;;
        llm-agent)  echo "$MONOREPO/ai/llm-ui/agent-starter-python/install_service.sh" ;;
        pupper-gui) echo "$MONOREPO/pupper-rs/install_service.sh" ;;
        *)          echo "" ;;
    esac
}

# Register any unit systemd doesn't know about yet.
for svc in "${SERVICES[@]}"; do
    if ! systemctl cat "${svc}.service" >/dev/null 2>&1; then
        installer="$(installer_for "$svc")"
        if [ -f "$installer" ]; then
            echo "[$svc] not registered — running $(basename "$installer")"
            # Run via bash so a missing executable bit doesn't block install.
            bash "$installer"
        else
            echo "[$svc] ERROR: not registered and no installer at $installer" >&2
        fi
    fi
done

sudo systemctl daemon-reload

for svc in "${SERVICES[@]}"; do
    echo "[$svc] starting"
    sudo systemctl start "${svc}.service"
done

echo
echo "=== status ==="
for svc in "${SERVICES[@]}"; do
    printf "%-11s %s\n" "$svc" "$(systemctl is-active "${svc}.service" 2>&1)"
done
