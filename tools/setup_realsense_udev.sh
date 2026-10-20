#!/usr/bin/env bash
# Run on the Linux host, where udev manages camera device permissions.
set -euo pipefail

rules_file=$(mktemp)
trap 'rm -f "$rules_file"' EXIT

# Use a released SDK's rules rather than a moving master branch.
curl --fail --silent --show-error --location \
    https://raw.githubusercontent.com/realsenseai/librealsense/v2.58.4/config/99-realsense-libusb.rules \
    --output "$rules_file"

sudo install -m 0644 "$rules_file" /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules
echo 'RealSense udev rules installed. Unplug and reconnect the camera.'
