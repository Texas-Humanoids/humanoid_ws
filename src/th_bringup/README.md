# th_bringup

Launch files that start the robot, plus YAML config (CAN bus assignments and
joint calibration).

## robot.launch.py

Starts the full gamepad → policy → motor pipeline: `game_controller_node` and
`gamepad_node`, `policy_node`, `policy_bridge`, and one `robstride_motor_node`
per entry in the motors config. See
[th_policy_bridge](../th_policy_bridge/README.md) for how the pieces connect.

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/fetch_berkeley_policy.py
colcon build --packages-up-to th_bringup
source install/setup.bash
ros2 launch th_bringup robot.launch.py   # run from the repo root (model path is relative)
```

| Argument | Default | Meaning |
|---|---|---|
| `input_mode` | `controller` | `controller` or `keyboard`, passed to gamepad_node |
| `motors_config` | `config/motors.yaml` (empty) | Joint-to-motor mapping and calibration |
| `policy_config` | th_policy_node's `policy_humanoid.yaml` | Joint order, default pose, action scale |
| `model_path` | `policies/policy_humanoid.onnx` | ONNX checkpoint |
| `imu_topic` | `/imu/data` | `sensor_msgs/Imu` source |
| `require_imu` | `true` | `false` assumes upright and still (bench tests without an IMU) |

Hold **LB** and use the sticks to command velocity. Press **START** to send
policy targets to the motors and **BACK** to stop sending them (the motors
hold position). Ctrl-C stops every motor.

## Motor config

`config/motors.yaml` ships empty, so the default launch commands no hardware.
Each entry maps one policy joint to one motor:

```yaml
motors:
  - joint: leg_left_knee_pitch_joint   # must be a policy joint name
    motor_id: 2                        # from find_motor_id.sh
    motor_model: RS06                  # RS03 or RS06
    can_interface: can_usb
    direction: 1.0                     # motor = direction * joint + offset
    offset: 0.0
    lower: 0.0                         # physical joint limits, radians
    upper: 2.443
```

Each motor runs in namespace `/<joint name without _joint>`, for example
`/leg_left_knee_pitch/motor/target_position`. Joints without an entry read as
the policy's default pose. Unknown joints, missing fields and duplicate
joints or motor IDs stop the launch.

## Bench test: Jetson + 3 motors + Xbox controller

[config/bench_motors.yaml](config/bench_motors.yaml) maps three free-spinning
motors to the left leg's hip, knee and ankle pitch joints.

1. **Controller.** On the Jetson host, pair the Xbox controller with
   `bluetoothctl` (`scan on`, `pair <MAC>`, `trust <MAC>`, `connect <MAC>`).
   If it keeps disconnecting, update the controller's firmware from a
   Windows/Xbox console first.
2. **CAN.** The motors are on the USB-CAN adapter. Linux names it `can0` or
   `can1` depending on which CAN driver loads first at boot (the Jetson's
   built-in CAN takes the other name), so give it a fixed name once:
   ```bash
   sudo cp tools/jetson/10-can-usb.link /etc/systemd/network/
   sudo udevadm control --reload
   sudo udevadm trigger --action=add /sys/class/net/can0 /sys/class/net/can1
   ip -br link show type can   # the adapter is now can_usb
   ```
   Wire CAN_H, CAN_L **and GND** to every motor, power the motors, then
   `sudo ip link set can_usb up type can bitrate 1000000`. See
   [robstride_motor](../robstride_motor/README.md) for details.
3. **Container.** From the repo root on the Jetson:
   `docker compose --profile robot up -d --build`, then
   `docker compose exec robot bash`. This profile shares the host network
   (for CAN), `/dev` and udev (for the controller).
4. **Motor IDs and offsets.** Run `src/robstride_motor/scripts/find_motor_id.sh can_usb`
   and put the real IDs and models (RS03/RS06) in `bench_motors.yaml`. Then
   measure each `offset` as described under
   [One-motor bench](#one-motor-bench-tested-on-the-jetson). With
   `offset: 0.0`, START sends each motor to the policy pose measured from the
   motor's zero, which can be several radians from where it sits.
5. **Check the controller.** `ros2 run joy game_controller_node` and
   `ros2 topic echo /joy` in a second shell. LB should be `buttons[9]` and
   the left stick `axes[0]`/`axes[1]`. Stop it before launching.
6. **Run.**
   ```bash
   source /opt/ros/jazzy/setup.bash
   python3 tools/fetch_berkeley_policy.py
   colcon build --packages-up-to th_bringup && source install/setup.bash
   ros2 launch th_bringup robot.launch.py require_imu:=false \
     motors_config:=src/th_bringup/config/bench_motors.yaml
   ```
   Wait for `Bridging 3 motors` and for the `Waiting: motor feedback is stale`
   warnings to stop, then press START (`ARMED` is logged).
7. **Drive.** Hold LB and push the left stick forward, back, or the right
   stick sideways. Press BACK to disarm, Ctrl-C to stop.

What to expect: after START the motors move to the policy's standing pose
(knee about 0.45 rad) at the 1 rad/s speed limit and keep making small
corrections. Each stick direction shifts the joints by roughly 0.1 rad
(about 6°), and centering the stick returns them. Tape a flag on each shaft
to see it. In a simulated bench run (fake motors at 1 rad/s), the mean
targets were:

| Command | hip | knee | ankle |
|---|---|---|---|
| stop | -0.47 | 0.44 | -0.28 |
| forward 0.5 m/s | -0.48 | 0.45 | -0.38 |
| back 0.5 m/s | -0.36 | 0.58 | -0.29 |
| turn left 1 rad/s | -0.56 | 0.50 | -0.31 |

Keep the 1 rad/s speed limit for this test. With faster motors the policy,
which only sees three real joints and no IMU, chases its own motion and
swings over the full joint range regardless of the stick.

## One-motor bench (tested on the Jetson)

[config/bench_one_motor.yaml](config/bench_one_motor.yaml) is the config the
pipeline ran with on the Jetson on 2026-11-08: one RS06 (ID 1) on the USB CAN
adapter (`can_usb`) standing in for the left hip pitch joint, with an Xbox
Wireless controller over Bluetooth. Follow steps 1-6 above, then launch with:

```bash
ros2 launch th_bringup robot.launch.py require_imu:=false \
  motors_config:=src/th_bringup/config/bench_one_motor.yaml
```

After START the motor moved about 0.29 rad (17°) at 1 rad/s to the policy's
standing hip pose (joint angle about -0.49 rad, close to the -0.47 in the
table above) and held it, and it responded to the stick. When the motor was
unplugged it disabled itself and the node logged a possible CAN timeout.

**Measuring the offset.** The offset maps the policy's default joint angle to
the motor's current position, so it changes whenever the motor is remounted.
Launch with the config and leave the bridge disarmed (motors hold where they
are until START), then read the motor position:

```bash
ros2 topic echo --once /leg_left_hip_pitch/motor/actual_position
```

`offset = actual_position - direction * default_angle`, where `default_angle`
is the joint's entry in `default_joint_positions` in
`th_policy_node/config/policy_humanoid.yaml` (-0.2 for the hip pitch, 0.4 for
the knee, -0.3 for the ankle). Ctrl-C, put the offset in the config, and
relaunch.

## Live topics

`ros2 topic echo /leg_left_knee_pitch/motor/actual_velocity` and
`/policy/joint_targets` show the same numbers live.
`/leg_left_knee_pitch/motor/status` shows each motor's mode, faults,
temperature and torque. If a motor faults or its CAN timeout trips (for
example after the container is killed), it stops and ignores targets until
`ros2 service call /leg_left_knee_pitch/motor/clear_faults std_srvs/srv/Trigger`.
