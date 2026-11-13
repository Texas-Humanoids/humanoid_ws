# tools

Standalone helper scripts, e.g. setting motor CAN IDs, calibrating joint
offsets, bringing up CAN interfaces.

- `jetson/10-can-usb.link`: names the USB-CAN adapter `can_usb` on the
  Jetson so it does not swap with the built-in CAN port at boot. Install steps
  are in [th_bringup](../src/th_bringup/README.md).
