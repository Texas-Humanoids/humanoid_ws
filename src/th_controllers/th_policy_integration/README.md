# Policy observation builder (issue #18)

`observation_builder` connects the gamepad's `/cmd_vel` to the existing Berkeley
policy input, using live IMU and joint feedback. Run it alongside
`th_policy_node` after sourcing the workspace:

```bash
ros2 run th_policy_integration observation_builder
ros2 run th_policy_node policy_node --ros-args \
  --params-file src/th_policy_node/config/policy_humanoid.yaml \
  -p model_path:=policies/policy_humanoid.onnx
```

It publishes `/policy/observation` (`Float32MultiArray`) at 25 Hz only when
`/imu/data` (`Imu`) and `/joint_states` (`JointState`) contain fresh, complete,
finite data. The joint message must have positions **and velocities** for all
22 Berkeley joint names; its names may arrive in any order. An expired
`/cmd_vel` (`Twist`) becomes zero. After each policy output on
`/policy/joint_targets`, the node reconstructs the previous clipped actions
for the next observation. Initial previous actions are zero.

The IMU orientation must describe the body frame relative to the world frame,
and angular velocity must be expressed in that same body frame. The current
gamepad PR publishes forward and yaw commands; lateral command remains zero.
Topic names, joint profile, policy period, and freshness timeouts are ROS
parameters. The default feedback timeout is 0.2 s and command timeout is
0.3 s.

This package **does not publish motor commands**. The current motor PR accepts
one `Float64` target, while the policy publishes 22 named targets. Before
physical control, the team needs complete measured joint and IMU feedback,
joint-to-CAN mapping, directions, zero offsets, gearing, joint limits, and an
explicit enable/stop mechanism. The policy's MuJoCo diagnostic currently finds
some targets beyond model joint ranges. No motor bridge should pass those
targets to hardware as-is.

For a ROS transport check using synthetic feedback and the real ONNX model:

```bash
python tools/verify_issue18_ros.py --model policies/policy_humanoid.onnx
```

Run from the repository root after building and sourcing both ROS packages.
The script uses a separate ROS domain and remapped topics, then checks the
observation order, policy output, previous-action feedback, and stale-state
behavior. It does not open CAN or command any motor.
