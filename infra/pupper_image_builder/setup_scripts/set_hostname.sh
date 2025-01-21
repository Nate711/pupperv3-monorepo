#!/bin/bash -e

set -x

# Define the content for /etc/hosts
HOSTS_CONTENT=$(cat <<EOL
127.0.1.1 pupper
127.0.0.1 localhost

# The following lines are desirable for IPv6 capable hosts
::1 localhost ip6-localhost ip6-loopback
ff02::1 ip6-allnodes
ff02::2 ip6-allrouters
EOL
)

# Write the content to /etc/hosts
echo "$HOSTS_CONTENT" | sudo tee /etc/hosts > /dev/null

# Notify the user that /etc/hosts has been updated
echo "/etc/hosts has been updated successfully!"
