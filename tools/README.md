# tools

Standalone helper scripts, e.g. setting motor CAN IDs, calibrating joint
offsets, bringing up CAN interfaces.

- `bench_camera/` — live webcam feed of the tabletop motor bench (runs on the Jetson).
- `jetson/10-can-usb.link` — names the USB-CAN adapter `can_usb` on the
  Jetson so it does not swap with the built-in CAN port at boot. Install steps
  are in [th_bringup](../src/th_bringup/README.md).
- `walker/` — WalkerV2 bring-up: `calibrate_offsets.py` measures every motor offset
  in the default pose, and `wiggle_test.py` steps each joint to check directions
  and log step responses. See [th_bringup](../src/th_bringup/README.md).
