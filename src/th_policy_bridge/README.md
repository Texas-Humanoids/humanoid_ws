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

At `rate` (50 Hz) the bridge publishes one observation on
`/policy/observation`. Its layout is a list of terms, `observation_terms`,
each multiplied by its entry in `observation_scales`:

| term | size | source |
|---|---|---|
| `velocity_command` | 3 | `/cmd_vel` (x, y, yaw) |
| `base_ang_vel` | 3 | IMU angular velocity, rotated into the base frame |
| `projected_gravity` | 3 | IMU orientation, rotated into the base frame |
| `joint_pos_rel` | N | joint positions minus the default pose |
| `joint_vel` | N | joint velocities |
| `last_action` | N | previous policy action (zero while standing, so WALK starts from zero) |
| `gait_phase` | 2 | sin, cos of 2π·t/`gait_period`; t starts at WALK, 0 before |

With no `observation_terms` the layout is the Berkeley one (command, angular
velocity, gravity, joint positions, joint velocities, previous actions,
unscaled; 75 values for 22 joints). The launch file fills these from the
training export when given `policy_spec:=` (see
[th_bringup](../th_bringup/README.md)). Details:

- `/cmd_vel` (x, y, yaw). A command older than `cmd_vel_timeout` (0.5 s)
  reads as zero, which matches the gamepad sending one stop and then going quiet.
- IMU angular velocity and projected gravity (from the IMU orientation).
- Joint positions relative to the default pose, and joint velocities, from
  each motor's `actual_position` and `actual_velocity` after calibration.
  **Joints without a motor read as the default pose with zero velocity**, so a
  bench with a few motors runs the full policy.
- Previous actions, recovered from `/policy/joint_targets` as
  `(target - default) / action_scale`.

**IMU frame.** `imu_rotation` ([w, x, y, z], identity by default) rotates
IMU-frame vectors into the base frame (x forward, y left, z up).
`imu_world_frame: ned` handles IMUs whose orientation is relative to a z-down
world (VectorNav's default). `~/projected_gravity` publishes the result, so
the mounting can be checked by tilting the robot by hand.

While disarmed it publishes nothing when any motor's feedback is older than
`feedback_timeout` (0.1 s) or the IMU is older than `imu_timeout` (0.1 s).
Once armed, gaps up to `armed_dropout_timeout` (0.3 s) only log a warning and
the observation uses the last readings. Disarming mid-walk drops the robot, so
only a longer outage disarms.
`require_imu:=false` assumes upright and not rotating instead, for bench tests
without an IMU. The launch file copies the joint order, default pose and
action scale from the policy config, so they are defined once.

## Motor commands

For each configured motor, the bridge sends
`direction * clamp(target, lower, upper) + offset` to
`<ns>/motor/target_position`, and converts feedback back with the inverse.
Targets reach the motors only while the bridge is **armed**:

- **START** (`gamepad/button_event` = `START pressed`) arms, but only when
  every motor (and the IMU, if required) is fresh and the base is upright.
- With `walk_event` set (e.g. `A pressed`), arming is two-step. START enters
  **STAND**: targets ramp linearly from the measured pose to the default pose
  over `stand_ramp_time` (2 s) and hold there. `walk_event` then enters
  **WALK**, which forwards the policy's targets. Without `walk_event`, START
  goes straight to WALK (the bench behavior).
- **BACK** disarms. So do a policy target gap longer than `target_timeout`
  (0.2 s) while walking, a sensor outage (above), and a base tilt beyond
  `max_tilt` (1.0 rad; IMU only).
- `~/mode` (transient local) publishes `DISARMED`, `STAND` or `WALK`.
- Once targets stop, CSP motors hold their last target and MIT motors drop
  to damping (see [robstride_motor](../robstride_motor/README.md)). The bridge
  never disables motors; stopping the motor nodes (Ctrl-C) sends their stop
  frame.

## Test

```bash
colcon build --packages-select th_policy_bridge
colcon test --packages-select th_policy_bridge
colcon test-result --verbose
```

Google Tests cover projected gravity (ENU and NED), the IMU rotation,
observation layouts and scales, the gait phase, the stand ramp, action
recovery, and joint calibration and limits. The pytest suite runs the real executable
over DDS with fake motors, IMU and policy. It checks observation contents,
joints without motors, freshness gating, arming, clamping, calibration, both
disarm timeouts, STAND→WALK arming, the tilt cutoff, the armed dropout
tolerance and config validation.

## Known limitations

- There is no IMU driver in this repo yet. Any node publishing
  `sensor_msgs/Imu` with orientation and body-frame angular velocity on
  `/imu/data` works.
- The observation must match the training config exactly. Use the training
  export (`policy_spec.json`) rather than hand-copying values.
