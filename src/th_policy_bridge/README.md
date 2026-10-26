# th_policy_bridge

`policy_bridge` connects the policy node to the rest of the robot:

```
gamepad_node ──/cmd_vel───────────►┐
gamepad_node ──/gamepad/button_event►  policy_bridge ──/policy/observation──► policy_node
IMU ──/imu/data───────────────────►┘        ▲    │                                 │
                                             │    └──────/policy/joint_targets◄─────┘
robstride_motor_node ×N ─<ns>/motor/actual_position, actual_velocity
                        ◄<ns>/motor/target_position── (only while armed)
```

Start everything with `ros2 launch th_bringup robot.launch.py`
(see [th_bringup](../th_bringup/README.md)).

## Observations

At `rate` (25 Hz) the bridge publishes one 75-value Berkeley observation on
`/policy/observation`, in the order documented in
[th_policy_node](../th_policy_node/README.md):

- `/cmd_vel` (x, y, yaw). A command older than `cmd_vel_timeout` (0.5 s)
  reads as zero, which matches the gamepad sending one stop and then going quiet.
- IMU angular velocity and projected gravity (from the IMU orientation).
- Joint positions relative to the default pose, and joint velocities, from
  each motor's `actual_position` and `actual_velocity` after calibration.
  **Joints without a motor read as the default pose with zero velocity**, so a
  bench with a few motors runs the full policy.
- Previous actions, recovered from `/policy/joint_targets` as
  `(target - default) / action_scale`.

It publishes nothing while any motor's feedback is older than
`feedback_timeout` (0.1 s) or the IMU is older than `imu_timeout` (0.1 s).
`require_imu:=false` assumes upright and not rotating instead, for bench tests
without an IMU. The launch file copies the joint order, default pose and
action scale from the policy config, so they are defined once.

## Motor commands

For each configured motor, the bridge sends
`direction * clamp(target, lower, upper) + offset` to
`<ns>/motor/target_position`, and converts feedback back with the inverse.
Targets reach the motors only while the bridge is **armed**:

- **START** (`gamepad/button_event` = `START pressed`) arms, but only when
  every motor (and the IMU, if required) is fresh and the policy is publishing.
- **BACK** disarms. So does a policy target gap longer than `target_timeout`
  (0.2 s), or stale motor or IMU data.
- While disarmed, the motors hold their last target. The bridge never
  disables motors; stopping the motor nodes (Ctrl-C) sends their stop frame.

The motor node's speed limit (1 rad/s by default) is what keeps the first
armed target from jumping the joint.

## Test

```bash
colcon build --packages-select th_policy_bridge
colcon test --packages-select th_policy_bridge
colcon test-result --verbose
```

Google Tests cover projected gravity, observation layout, action recovery,
and joint calibration and limits. The pytest suite runs the real executable
over DDS with fake motors, IMU and policy. It checks observation contents,
joints without motors, freshness gating, arming, clamping, calibration, both
disarm timeouts and config validation.

## Known limitations

- There is no IMU driver in this repo yet. Any node publishing
  `sensor_msgs/Imu` with orientation and body-frame angular velocity on
  `/imu/data` works.
- The 25 Hz rate and observation scaling should be checked against the
  checkpoint's training config before trusting it on a walking robot.
