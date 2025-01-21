#!/bin/bash -e

# Backup the original resolved.conf file
sudo cp /etc/systemd/resolved.conf /etc/systemd/resolved.conf.backup

# Update the DNS settings in resolved.conf
sudo sed -i 's/^#DNS=.*/DNS=8.8.8.8 1.1.1.1/' /etc/systemd/resolved.conf

# Restart systemd-resolved service to apply changes
sudo systemctl restart systemd-resolved

echo "DNS settings updated and systemd-resolved restarted."
