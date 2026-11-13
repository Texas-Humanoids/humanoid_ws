#!/usr/bin/env bash
# Install the bench camera live feed on the Jetson (run on the Jetson).
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"

sudo apt-get install -y ustreamer v4l-utils
sudo install -m 644 "$here/bench-camera.service" /etc/systemd/system/bench-camera.service
# Keep local edits to the settings file.
[ -f /etc/default/bench-camera ] || sudo install -m 644 "$here/bench-camera.env" /etc/default/bench-camera
sudo systemctl daemon-reload
sudo systemctl enable --now bench-camera
sudo systemctl restart bench-camera
echo "Live feed: http://$(hostname):$(. /etc/default/bench-camera; echo "$CAMERA_PORT")/stream"
