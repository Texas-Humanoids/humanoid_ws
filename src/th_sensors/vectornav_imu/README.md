# vectornav_imu

A small serial driver for the VectorNav VN-100 (the team's unit is a VN-100T-CR,
firmware 3.1.0.0, at 115200 baud). It publishes `sensor_msgs/Imu` on
`/imu/data` at 200 Hz for [policy_bridge](../../th_policy_bridge/README.md).

We wrote our own because no VectorNav package is released for ROS 2 Jazzy, and
the bridge only needs orientation and angular velocity. The node has no
dependencies beyond `rclcpp` and `sensor_msgs`.

## What it does

At startup it turns off the sensor's default ASCII output (`$VNYMR` at 40 Hz)
and enables binary output 1: quaternion, angular rate and acceleration at
800 Hz / divisor. These register writes are **not** saved to flash; on exit
the node restores the ASCII output, and a power cycle restores everything.

**Frames.** The sensor reports its body frame as x forward, y right, z down,
with orientation relative to North-East-Down. The node converts to ROS
conventions (REP 103): body x forward, y left, z up, orientation relative to
East-North-Up. This was checked on the real sensor: gravity computed from the
orientation matches the accelerometer to 0.1° (that reading is also a unit
test). The node repeats this check on every start and logs an error if they
disagree by more than 0.1 rad while the sensor is still.

## Run on its own

The serial port needs root or the `dialout` group; the `robot` compose
profile already runs as root with `/dev` mounted.

```bash
colcon build --packages-select vectornav_imu && source install/setup.bash
ros2 run vectornav_imu vectornav_imu_node --ros-args \
  -p port:=/dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LSON8-if00-port0
```

Expect `Frame check passed` within a few seconds and `200 Hz, 0 bad packets`
every 5 s. The robot launch starts it automatically when the robot yaml has
`imu.driver: vectornav` (see [th_bringup](../../th_bringup/README.md)).

| Parameter | Default | Meaning |
|---|---|---|
| `port` | `/dev/ttyUSB0` | serial device; prefer the `/dev/serial/by-id/...` path |
| `baud` | 115200 | the sensor's current baud (the node does not change it) |
| `rate` | 200 | Hz; 800 divided by a whole number that fits the baud (200 Hz uses 80 % of 115200) |
| `imu_topic` | `/imu/data` | output topic |
| `frame_id` | `imu_link` | header frame |
| `restore_on_exit` | true | turn the default ASCII output back on when stopping |
| `max_gravity_error` | 0.1 | rad; threshold for the startup frame check |

## Test

```bash
colcon test --packages-select vectornav_imu && colcon test-result --verbose
```

The gtests cover ASCII checksums against real replies from the sensor, binary
packet encode/decode and CRC, resyncing through ASCII text and stray sync
bytes, and the frame conversion using a real tilted reading.
