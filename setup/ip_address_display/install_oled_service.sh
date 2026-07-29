#!/bin/bash

# OLED IP Display Service Installer
# Usage: bash setup/ip_address_display/install_oled_service.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_DIR=$(echo "$SCRIPT_DIR" | tr -d '\r')
CURRENT_USER="${SUDO_USER:-$USER}"
CURRENT_USER=$(echo "$CURRENT_USER" | tr -d '\r')

echo "Installing OLED IP Display service..."
echo "  User: $CURRENT_USER"
echo "  Script dir: $SCRIPT_DIR"

# Fix CRLF line endings in shell scripts (if cloned on Windows)
sed -i 's/\r$//' "$SCRIPT_DIR/oled_ip_display.sh"

# Generate service file directly (no template, CRLF-safe)
printf "[Unit]\nDescription=OLED IP Address Display\nAfter=network-online.target\nWants=network-online.target\n\n[Service]\nType=oneshot\nUser=%s\nExecStart=%s/oled_ip_display.sh\nRemainAfterExit=yes\nWorkingDirectory=%s\n\n[Install]\nWantedBy=multi-user.target\n" "$CURRENT_USER" "$SCRIPT_DIR" "$SCRIPT_DIR" | sudo tee /etc/systemd/system/oled-ip-display.service > /dev/null

# Set execute permissions
chmod +x "$SCRIPT_DIR/oled_ip_display.sh"
chmod +x "$SCRIPT_DIR/oled_ip_display.py"

# Enable and start service
sudo systemctl daemon-reload
sudo systemctl enable oled-ip-display.service
sudo systemctl start oled-ip-display.service

echo "Done. Checking status..."
sudo systemctl status oled-ip-display.service --no-pager
