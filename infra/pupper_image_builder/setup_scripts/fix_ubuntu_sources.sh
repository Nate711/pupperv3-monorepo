#!/bin/bash -e

# Define the path to the sources file
SOURCE_FILE="/etc/apt/sources.list.d/ubuntu.sources"

# Backup the original file
cp "$SOURCE_FILE" "${SOURCE_FILE}.bak"

# Overwrite the ubuntu.sources file with the desired content
cat <<EOL > "$SOURCE_FILE"
## Ubuntu distribution and updates repository for noble
Types: deb
URIs: http://ports.ubuntu.com/ubuntu-ports/
Suites: noble noble-updates noble-backports noble-security
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
EOL

echo "Overwritten $SOURCE_FILE with noble, noble-updates, noble-backports, and noble-security on a single line."

# Verify the content
cat "$SOURCE_FILE"
