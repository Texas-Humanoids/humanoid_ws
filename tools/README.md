# tools

Standalone helper scripts, e.g. setting motor CAN IDs, calibrating joint
offsets, bringing up CAN interfaces.

`bash tools/setup_realsense_udev.sh` installs the upstream RealSense device
permission rules on the Linux host and reloads udev. Run it once, then reconnect
the camera. See [the camera setup guide](../src/th_sensors/README.md).
