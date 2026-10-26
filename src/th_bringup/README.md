# th_bringup

Launch files that start the robot, plus YAML config (CAN bus assignments and
joint calibration).

## robot.launch.py

Starts the full gamepad → policy → motor pipeline: `joy_node` and
`gamepad_node`, `observation_builder`, `policy_node`, `joint_motor_bridge`,
and one `robstride_motor_node` per entry in `config/motors.yaml`. See
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
| `motors_config` | `config/motors.yaml` | Joint-to-motor mapping and calibration |
| `policy_config` | th_policy_node's `policy_humanoid.yaml` | Joint order, default pose, action scale |
| `model_path` | `policies/policy_humanoid.onnx` | ONNX checkpoint |
| `imu_topic` | `/imu/data` | `sensor_msgs/Imu` source |
| `require_imu` | `true` | `false` assumes upright and still (bench only) |
| `require_all_joints` | `true` | `false` reads missing joints as the default pose (bench only) |

Hold **LB** and use the sticks to command velocity. Press **START** to send
policy targets to the motors and **BACK** to stop sending them (the motors
hold position).

## Motor config

`config/motors.yaml` ships empty, so the default launch commands no hardware.
Add one entry per joint with a motor:

```yaml
motors:
  - joint: leg_left_knee_pitch_joint   # must be a policy joint name
    motor_id: 1                        # from find_motor_id.sh
    can_interface: can0
    direction: 1.0                     # motor = direction * joint + offset
    offset: 0.0
    lower: 0.0                         # physical joint limits, radians
    upper: 1.2
```

Each motor runs in namespace `/<joint name without _joint>`, for example
`/leg_left_knee_pitch/motor/target_position`. The launch file rejects
unknown joints, missing fields, and duplicate joints or CAN IDs.

### Single-motor bench test

With one motor and no IMU, the policy cannot close the loop. Its targets for
that joint are meaningless, so set tight `lower`/`upper` limits first:

```bash
ros2 launch th_bringup robot.launch.py \
  motors_config:=/path/to/bench_motors.yaml \
  require_imu:=false require_all_joints:=false
```

Then press START with the joint free to move within its limits.
