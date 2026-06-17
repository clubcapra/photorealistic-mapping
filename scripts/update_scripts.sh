#!/usr/bin/env bash

if [[ "$EUID" -eq 0 ]]; then
    echo "Cannot run this as root, run this script as a user"
    exit 1
fi

# 1. Setup Logging Directories
echo "1/7 Setup log dirs"
sudo mkdir -p /mnt/ssd/sftp/log/ros2
sudo chmod 775 /mnt/ssd/sftp/log/ros2
sudo chown capra:capra /mnt/ssd/sftp/log/ros2

# 2. Define Paths
echo "2/7 Define paths"
USER_SYSTEMD_DIR="$HOME/.config/systemd/user"
mkdir -p "$USER_SYSTEMD_DIR"
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
FORWARDER_DIR="$SCRIPT_DIR/livox_forwarder"
FORWARDER_BIN="$FORWARDER_DIR/livox_forwarder"
FORWARDER_SRC="$FORWARDER_DIR/livox_forwarder.cpp"

# 3. Enable lingering so user services start on boot and survive logout
echo "3/7 Enable user lingering"
sudo loginctl enable-linger capra

# 4. Build the livox forwarder
echo "4/7 Build livox_forwarder"
if [[ ! -f "$FORWARDER_SRC" ]]; then
    echo "ERROR: livox_forwarder.cpp not found at $FORWARDER_SRC"
    exit 1
fi
g++ -O2 -o "$FORWARDER_BIN" "$FORWARDER_SRC"
if [[ $? -ne 0 ]]; then
    echo "ERROR: failed to build livox_forwarder"
    exit 1
fi
echo "Built $FORWARDER_BIN"

# 5. Install User-level services
echo "5/7 Install and enable services"
echo "Installing services to $USER_SYSTEMD_DIR..."

# cp "$SCRIPT_DIR/rove_mapping_launch.service" "$USER_SYSTEMD_DIR/"
cp "$SCRIPT_DIR/rove_mapping_api.service"  "$USER_SYSTEMD_DIR/"
cp "$SCRIPT_DIR/livox_forwarder.service"   "$USER_SYSTEMD_DIR/"

# 6. Reload the user daemon
echo "6/7 Reload daemon"
systemctl --user daemon-reload

# 7. Enable and Start the services
echo "7/7 Enable and Start the services"
# systemctl --user enable rove_mapping_launch.service
# systemctl --user restart rove_mapping_launch.service
systemctl --user enable  rove_mapping_api.service
systemctl --user restart rove_mapping_api.service
systemctl --user enable  livox_forwarder.service
systemctl --user restart livox_forwarder.service

echo 'Done! Check status with:'
echo '  systemctl --user status rove_mapping_api.service'
echo '  systemctl --user status rove_mapping_launch.service'
echo '  systemctl --user status livox_forwarder.service'
echo ''
echo 'Watch forwarding stats with:'
echo '  journalctl --user -u livox_forwarder.service -f'