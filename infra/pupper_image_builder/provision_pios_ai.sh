#!/bin/bash -e

set -x

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

export DEBIAN_FRONTEND=noninteractive


DEFAULT_USER=pi
mkdir -p /home/$DEFAULT_USER
chown -R $DEFAULT_USER:$DEFAULT_USER /home/$DEFAULT_USER

export APT_LISTCHANGES_FRONTEND=none
# (Optional) avoid services trying to start in chroot
printf '#!/bin/sh\nexit 101\n' > /usr/sbin/policy-rc.d && chmod +x /usr/sbin/policy-rc.d


sudo apt-get update

apt-get -y \
  -o Dpkg::Options::="--force-confdef" \
  -o Dpkg::Options::="--force-confold" \
  upgrade

rm -f /usr/sbin/policy-rc.d


########################### Install rust ########################
export CARGO_HOME=/home/$DEFAULT_USER/.cargo
export RUSTUP_HOME=/home/$DEFAULT_USER/.rustup
# put these in bashrc as well
echo 'export CARGO_HOME=/home/pi/.cargo' >> /home/$DEFAULT_USER/.bashrc
echo 'export RUSTUP_HOME=/home/pi/.rustup' >> /home/$DEFAULT_USER/.bashrc
echo 'source $HOME/.cargo/env' >> /home/$DEFAULT_USER/.bashrc

curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source $HOME/.cargo/env

rustup target add aarch64-unknown-linux-gnu

############################### Install LLM deps ###########################
sudo rm -f /usr/lib/python3.*/EXTERNALLY-MANAGED
pip install "livekit-agents[cartesia,google,openai,deepgram,silero,turn-detector]~=1.2"
pip install "python-dotenv"
pip install pandas

############################ Prepare monorepo ###############################################

# Prepare monorepo
cd /home/$DEFAULT_USER/pupperv3-monorepo/
chown -R pi:pi /home/$DEFAULT_USER/pupperv3-monorepo/
git config --global --add safe.directory /home/$DEFAULT_USER/pupperv3-monorepo
git pull

############################## Download turn detector model #############################################
cd /home/$DEFAULT_USER/pupperv3-monorepo/ai/llm-ui/agent-starter-python/
python3 src/agent.py download-files

############################### Build everything #############################################

# Build monorepo ros2 code
bash /home/$DEFAULT_USER/pupperv3-monorepo/ros2_ws/build.sh

# Build Rust GUI
cd /home/$DEFAULT_USER/pupperv3-monorepo/pupper-rs
cargo build --release --target aarch64-unknown-linux-gnu

# Install systemctl services
bash /home/$DEFAULT_USER/pupperv3-monorepo/pupper-rs/install_service.sh
bash /home/$DEFAULT_USER/pupperv3-monorepo/ai/llm-ui/agent-starter-python/install_service.sh
sudo systemctl enable systemd-time-wait-sync.service

# Try chowning again 
chown -R $DEFAULT_USER:$DEFAULT_USER /home/$DEFAULT_USER