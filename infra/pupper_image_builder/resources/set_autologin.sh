#!/bin/bash -e

# Define the username for autologin
USERNAME="pi"

# Check if a username is provided
if [ -z "$USERNAME" ]; then
  echo "Usage: $0 <username>"
  exit 1
fi

# Write the autologin configuration to /etc/gdm3/custom.conf
sudo bash -c "cat > /etc/gdm3/custom.conf" <<EOL
[daemon]
# Enabling automatic login
AutomaticLoginEnable=true
AutomaticLogin=$USERNAME

# Other settings can be added here if needed
EOL

# Ensure the correct file permissions
sudo chmod 644 /etc/gdm3/custom.conf

# Restart the GDM service
sudo systemctl restart gdm3

echo "Autologin has been enabled for user: $USERNAME"
