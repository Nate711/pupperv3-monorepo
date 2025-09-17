#!/bin/bash -e

set -x

# Handle optional GitHub token for authenticated clones
GIT_ASKPASS_SCRIPT=""
GITHUB_TOKEN_CONFIGURED=false

cleanup_github_credentials() {
    if [ -n "${GIT_ASKPASS_SCRIPT:-}" ] && [ -f "${GIT_ASKPASS_SCRIPT}" ]; then
        rm -f "${GIT_ASKPASS_SCRIPT}"
    fi
    unset GIT_ASKPASS_SCRIPT
    unset GIT_ASKPASS
    unset GIT_TERMINAL_PROMPT
    unset GITHUB_TOKEN
    GITHUB_TOKEN_CONFIGURED=false
}

if [ -n "${GITHUB_TOKEN:-}" ]; then
    GITHUB_TOKEN_CONFIGURED=true
    GIT_ASKPASS_SCRIPT=$(mktemp /tmp/git-askpass-XXXXXX.sh)
    cat <<'EOF' > "${GIT_ASKPASS_SCRIPT}"
#!/bin/sh
case "$1" in
  Username*) echo "x-access-token" ;;
  Password*) echo "${GITHUB_TOKEN}" ;;
  *) echo "${GITHUB_TOKEN}" ;;
esac
EOF
    chmod 700 "${GIT_ASKPASS_SCRIPT}"
    export GIT_ASKPASS="${GIT_ASKPASS_SCRIPT}"
    export GIT_TERMINAL_PROMPT=0
    trap cleanup_github_credentials EXIT
fi

# Function to retry a command
retry_command() {
    local cmd="$1"
    local max_attempts=20
    local attempt=0

    until eval "$cmd" || [ $attempt -ge $max_attempts ]; do
        attempt=$((attempt + 1))
        echo "Attempt $attempt/$max_attempts failed. Retrying in 1 seconds..."
        sleep 1
    done

    if [ $attempt -ge $max_attempts ]; then
        echo "Command failed after $max_attempts attempts."
        return 1
    fi

    echo "Command succeeded!"
    return 0
}

############################## Basic setup ###############################################

export DEBIAN_FRONTEND=noninteractive


DEFAULT_USER=pi
mkdir -p /home/$DEFAULT_USER
chown -R $DEFAULT_USER:$DEFAULT_USER /home/$DEFAULT_USER


############################ Prepare monorepo ###############################################

# Prepare monorepo
apt-get update
apt-get install git-lfs -y
cd /home/$DEFAULT_USER
git clone https://github.com/Nate711/pupperv3-monorepo.git --recurse-submodules
cd /home/$DEFAULT_USER/pupperv3-monorepo/
git config --global --add safe.directory /home/$DEFAULT_USER/pupperv3-monorepo
git lfs install
git lfs pull
chown -R $DEFAULT_USER:$DEFAULT_USER /home/$DEFAULT_USER/pupperv3-monorepo

############################### Install dev dependencies ###############################################

export APT_LISTCHANGES_FRONTEND=none
# (Optional) avoid services trying to start in chroot
printf '#!/bin/sh\nexit 101\n' > /usr/sbin/policy-rc.d && chmod +x /usr/sbin/policy-rc.d


apt-get update

apt-get -y \
  -o Dpkg::Options::="--force-confdef" \
  -o Dpkg::Options::="--force-confold" \
  upgrade

rm -f /usr/sbin/policy-rc.d

apt-get install -y vim

rm -f /usr/lib/python3.*/EXTERNALLY-MANAGED
pip install wandb sounddevice openai[realtime] pydub pyaudio black supervision opencv-python loguru

# Install hailo
yes N | DEBIAN_FRONTEND=noninteractive apt full-upgrade -y

apt-get install -y hailo-all

# Source ros2
source /opt/ros/jazzy/setup.bash


############################### Install LLM deps ###########################
sudo rm -f /usr/lib/python3.*/EXTERNALLY-MANAGED
pip install "livekit-agents[cartesia,google,openai]~=1.2"
pip install "python-dotenv"



############################## Install ros2 deps from source ##################################

# Create directory for ros2 depenedencies we need to build from source
mkdir /home/$DEFAULT_USER/pupperv3-monorepo/ros2_ws/src/common
cd /home/$DEFAULT_USER/pupperv3-monorepo/ros2_ws/src/common

# install libcap-dev
apt-get install -y libcap-dev

# install dependencies for foxglove-bridge
apt-get install -y libwebsocketpp-dev nlohmann-json3-dev

# install dependencies for camera_ros
apt-get install -y libcamera-dev

pip install typeguard
pip uninstall em
pip install empy==3.3.4

repos=(
    "https://github.com/facontidavide/rosx_introspection.git"
    "https://github.com/foxglove/ros-foxglove-bridge.git"
    "https://github.com/christianrauch/camera_ros.git"
    "https://github.com/ros-perception/vision_msgs.git"
)

for repo in "${repos[@]}"; do
    retry_command "git clone $repo --recurse-submodules"
done

if [ "$GITHUB_TOKEN_CONFIGURED" = true ]; then
    cleanup_github_credentials
    trap - EXIT
fi

############################### Build everything #############################################

# Build monorepo ros2 code
bash /home/$DEFAULT_USER/pupperv3-monorepo/ros2_ws/build.sh

############### AUTOMATICALLY SOURCE ROS2 WORKSPACE IN BASHRC ################################
echo "source /home/$DEFAULT_USER/pupperv3-monorepo/ros2_ws/install/setup.bash" >> /home/$DEFAULT_USER/.bashrc
echo 'export RCUTILS_COLORIZED_OUTPUT=1' >> /home/$DEFAULT_USER/.bashrc

# Install utils
bash /home/$DEFAULT_USER/pupperv3-monorepo/robot/utils/install_battery_monitor.sh
bash /home/$DEFAULT_USER/pupperv3-monorepo/robot/utils/install_robot_auto_start_service.sh
sudo systemctl enable systemd-time-wait-sync.service

# Install volume-max service to set volume to 100% on boot
cd /home/$DEFAULT_USER/pupperv3-monorepo/robot/services/
bash install_volume_max_service.sh

# Try chowning again 
chown -R $DEFAULT_USER:$DEFAULT_USER /home/$DEFAULT_USER
