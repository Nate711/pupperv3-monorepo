#!/bin/bash -e

set -x

# Function to retry a command
retry_command() {
  local cmd="$1"
  local max_attempts="$2"
  local attempt=0

  # Loop to retry the command
  while [ $attempt -lt $max_attempts ]; do
    echo "Attempting to run: $cmd (Attempt $((attempt + 1))/$max_attempts)"
    
    # Run the command
    if eval "$cmd"; then
      echo "Command succeeded!"
      return 0  # Exit the function as success
    else
      attempt=$((attempt + 1))
      echo "Attempt $attempt/$max_attempts failed. Retrying in 5 seconds..."
      sleep 1
    fi
  done

  # If max attempts reached
  echo "Command failed after $max_attempts attempts."
  return 1  # Indicate failure
}

# Get the latest vscode version
# sudo apt-get install wget gpg
# wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
# sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
# echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" |sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null
# rm -f packages.microsoft.gpg

# sudo apt install apt-transport-https
# sudo apt update
# sudo apt install code # or code-insiders

# Get VSCode version 1.93.0
retry_command "wget -q https://update.code.visualstudio.com/1.93.0/linux-deb-arm64/stable -O vscode_1.93.0_arm64.deb" 20
apt install -y ./vscode_1.93.0_arm64.deb

## Disable git extension to stop VSCode from freezing when opening git repos ##

# Define the settings file path (user-level settings)
USERNAME="pi"
SETTINGS_FILE="/home/${USERNAME}/.config/Code/User/settings.json"

# Create the settings directory if it doesn't exist
mkdir -p "$(dirname "$SETTINGS_FILE")"

# Disable Git in settings.json
if [ -f "$SETTINGS_FILE" ]; then
  # If the settings file exists, modify it
  if grep -q '"git.enabled":' "$SETTINGS_FILE"; then
    # If "git.enabled" exists, replace its value with false
    sed -i 's/"git.enabled":.*/"git.enabled": false,/' "$SETTINGS_FILE"
  else
    # If "git.enabled" does not exist, append it before the closing bracket
    sed -i 's/}$/,"git.enabled": false}/' "$SETTINGS_FILE"
  fi
else
  # If the settings file does not exist, create it with the git setting
  cat <<EOL > "$SETTINGS_FILE"
{
    "git.enabled": false
}
EOL
fi

echo "VSCode Git integration has been disabled."
