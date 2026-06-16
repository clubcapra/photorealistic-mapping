#!/usr/bin/env bash

if [[ "$EUID" -eq 0 ]]; then
    echo "Cannot run this as root, run this script as a user"
    exit 1
fi

# 1. Setup Logging Directories
echo "1/6 Setup log dirs"
sudo mkdir -p /mnt/ssd/sftp/log/ros2
sudo chmod 775 /mnt/ssd/sftp/log/ros2
sudo chown capra:capra /mnt/ssd/sftp/log/ros2

# 2. Define Paths
echo "2/6 Define paths"
USER_SYSTEMD_DIR="$HOME/.config/systemd/user"
mkdir -p "$USER_SYSTEMD_DIR"
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

# 3. Enable lingering so user services start on boot and survive logout
echo "3/6 Enable user lingering"
sudo loginctl enable-linger capra

# 4. Install and Enable User-level services and set executable
echo "4/6 Install and enable services"
echo "Installing services to $USER_SYSTEMD_DIR..."

# cp "$SCRIPT_DIR/rove_mapping_launch.service" "$USER_SYSTEMD_DIR/"
cp "$SCRIPT_DIR/rove_mapping_api.service" "$USER_SYSTEMD_DIR/"

# 5. Reload the user daemon
echo "5/6 Reload daemon"
systemctl --user daemon-reload

# 6. Enable and Start the services
echo "6/6 Enable and Start the services"
# systemctl --user enable rove_mapping_launch.service
# systemctl --user restart rove_mapping_launch.service
systemctl --user enable rove_mapping_api.service
systemctl --user restart rove_mapping_api.service

echo 'Done! Check status with: systemctl --user status rove_mapping_launch.service and systemctl --user status rove_mapping_api.service'