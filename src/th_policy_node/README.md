# th_policy_node

Runs one ONNX walking policy from a ROS 2 node and publishes named joint
position targets. It is an inference bridge for checking the model inside ROS;
it does not connect to motors or a `ros2_control` command interface.

## Contract

- Subscribe to `std_msgs/msg/Float32MultiArray` on `/policy/observation`.
- The message must already contain the complete, model-ready observation vector
  in the model's exact feature order. The default full Berkeley Humanoid Lite
  profile expects 75 `float32` values.
- Run the ONNX model on CPU and scale its action vector by `action_scale`, then
  add `default_joint_positions`.
- Publish the named target positions in `sensor_msgs/msg/JointState` on
  `/policy/joint_targets`.

The default joint order, standing pose, action scale, and observation size are
based on Berkeley Humanoid Lite's 22-joint policy configuration. They are a
starting profile, not a replacement for the Controls/Middleware/Embedded
interface review tracked in TEX-44. Update them to match the agreed policy and
robot contract before connecting any downstream controller. The node does not
construct observations from IMU or joint-state topics.

For the matching Berkeley profile, the 75 input values are velocity command
(3), base angular velocity (3), projected gravity (3), relative joint position
(22), joint velocity (22), and previous policy action (22). This ordering is
inferred from the current Berkeley inference code and must be confirmed against
the policy version being run.

## Run

Install the ROS workspace dependencies in the CPU or GPU development container,
then build from the repository root:

```bash
python3 -m pip install --break-system-packages -r src/th_policy_node/requirements.txt
colcon build --symlink-install
source install/setup.bash
```

Place a deployed policy at `policies/policy_humanoid.onnx` or change `model_path`
in the parameter file to another ONNX file. Start the node from the repository
root:

```bash
ros2 run th_policy_node policy_node --ros-args --params-file src/th_policy_node/config/policy_humanoid.yaml
```

Publish one model-ready observation on `/policy/observation`. Inspect named
joint targets with:

```bash
ros2 topic echo /policy/joint_targets
```

An incorrect observation length, non-finite input, inference error, or action
count that does not match the configured joints is rejected without publishing.
