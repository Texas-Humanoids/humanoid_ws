# th_policy_bridge

Two nodes that connect the policy node to the rest of the robot:

```
gamepad_node ──/cmd_vel──► observation_builder ──/policy/observation──► policy_node
IMU ──/imu/data──────────►        ▲    ▲                                    │
                                  │    └──────/policy/joint_targets─────────┤
                          /joint_states                                     ▼
robstride_motor_node ×N ◄─<joint>/motor/target_position── joint_motor_bridge
                        ──<joint>/motor/actual_position──►  (armed from the gamepad)
```

Start everything with `ros2 launch th_bringup robot.launch.py`
(see [th_bringup](../th_bringup/README.md)).

## observation_builder

Publishes one 75-value Berkeley observation at `rate` (25 Hz) on
`/policy/observation`, in the order documented in
[th_policy_node](../th_policy_node/README.md): `/cmd_vel` (x, y, yaw),
IMU angular velocity, projected gravity (from the IMU orientation), joint
positions relative to the default pose, joint velocities, and previous
actions. It recovers the previous actions from `/policy/joint_targets` as
`(target - default) / action_scale`.

It enforces input freshness. It publishes nothing if the IMU is older than
`imu_timeout` (0.1 s) or any joint's feedback is older than
`joint_state_timeout` (0.1 s). A `/cmd_vel` older than `cmd_vel_timeout`
(0.5 s) reads as zero, which matches the gamepad sending one stop and then
going quiet. The launch file copies the joint order, default pose and action
scale from the policy config, so they are defined once.

For bench tests without the full robot, `require_imu:=false` assumes upright
and not rotating, and `require_all_joints:=false` reads joints without
feedback as the default pose. Both log a warning. With missing feedback, the
policy is open loop and its targets drift far outside the joint range, so
only the bridge's limits keep motors safe.

## joint_motor_bridge

For each configured joint, it subscribes to the policy targets and publishes
`direction * clamp(target, lower, upper) + offset` to
`<namespace>/motor/target_position`. It converts each
`<namespace>/motor/actual_position` back to joint space, estimates velocity
by finite difference, and publishes `/joint_states` at 50 Hz with only the
joints whose feedback is fresher than `feedback_timeout` (0.3 s). Targets for
unconfigured joints are dropped.

Targets reach the motors only while the bridge is **armed**:

- **START** (`gamepad/button_event` = `START pressed`) arms, but only when
  every configured motor has fresh feedback and the policy is publishing.
- **BACK** disarms. So does any policy target gap longer than
  `target_timeout` (0.2 s) or stale motor feedback.
- While disarmed, the motors hold their last target. The bridge never
  disables motors; stopping the motor nodes (Ctrl-C) sends their stop frame.

The motor node's 1 rad/s speed limit is what keeps the first armed target
from jumping the joint. Arm with the robot supported.

## Test

```bash
colcon build --packages-select th_policy_bridge
colcon test --packages-select th_policy_bridge
colcon test-result --verbose
```

Google Tests cover projected gravity, observation layout, action recovery,
and joint calibration and limits. The pytest suite starts both real
executables over DDS with fake motors, IMU and policy. It checks observation
contents, freshness gating, bench mode, arming, clamping, calibration and
both disarm timeouts.

## Known limitations

- The RS06 node reports position only, at 10 Hz. Joint velocity is a finite
  difference of those samples, which is noisy and lags compared with the
  policy's training data. Faster feedback with motor-reported velocity is
  needed before walking.
- There is no IMU driver in this repo yet. Any node publishing
  `sensor_msgs/Imu` with orientation and body-frame angular velocity on
  `/imu/data` works.
- The 25 Hz rate and observation scaling should be checked against the
  checkpoint's training config before trusting it on hardware.
