# tools

Standalone helper scripts, e.g. setting motor CAN IDs, calibrating joint
offsets, bringing up CAN interfaces.

- `bench_camera/` — live webcam feed of the tabletop motor bench (runs on the Jetson).
- `jetson/05-can-left.link`, `jetson/05-can-right.link`: name the two USB-CAN adapters
  `can_left` (motors 11–15) and `can_right` (21–25) by their serial numbers, so the names
  don't swap between boots. `jetson/10-can-usb.link` names any other CANable `can_usb` (the one-motor bench).
  `jetson/can_status.sh` lists each adapter (name, serial, USB port, bus state) and the motor IDs
  that answer on it. Install steps are in [th_bringup](../src/th_bringup/README.md).
- `walker/` — WalkerV2 bring-up: `calibrate_offsets.py` measures every motor offset
  in the default pose, and `wiggle_test.py` steps each joint to check directions
  and log step responses. See [th_bringup](../src/th_bringup/README.md).
  `sim_check.py` drives the MuJoCo sim (`sim:=true`) through STAND and WALK
  and checks rates and that the robot stays up; `sim_viewer.py` shows a sim
  streamed out of Docker on the Mac.
- `walker/sysid/`: actuator system ID. `chirp.py` runs chirp, friction, step and top-speed
  tests on one motor (Jetson), and `fit.py` fits inertia, friction, delay and top speed for
  training. See its README. Also there: `spin.py` (one motor at a constant speed), `mit_all.py`
  (every motor at once in MIT mode: hold, sine, spin, home) and `monitor.py` (a live table of
  every drive's position, velocity, torque, temperature, voltage, current, mode, faults and limits).
- `walker/bench/`: scripts and config from the Jetson two-motor bench (MIT checks, hold
  test, pipeline and policy monitors). See its README.
