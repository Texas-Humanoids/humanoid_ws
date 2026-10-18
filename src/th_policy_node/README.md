# th_policy_node

Runs one ONNX walking policy from a ROS 2 node and publishes named joint
position targets. It is an inference bridge for checking the model inside ROS;
it does not connect to motors or a `ros2_control` command interface.

Implements the initial policy-loading step in
[issue #17](https://github.com/Texas-Humanoids/humanoid_ws/issues/17).

See [VALIDATION.md](VALIDATION.md) for the completed ROS/real-model check and
its limitations. See [INTEGRATION.md](INTEGRATION.md) for the current joystick,
policy, and single-motor message contracts.

## Repeatable Berkeley demo (Ubuntu / WSL, ROS 2 Jazzy)

Run from the repository root with ROS 2 Jazzy installed. A virtual environment
keeps ONNX Runtime separate from the system Python while retaining ROS modules.
Use the virtual environment's Python for colcon so the installed node uses it too.

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r src/th_policy_node/requirements.txt
python -m colcon build --packages-select th_policy_node --symlink-install
source install/setup.bash
ros2 run th_policy_node fetch_berkeley_policy
python tools/verify_policy_ros.py
```

The fetch command downloads the team's existing Berkeley model at a fixed
revision, checks SHA-256, and refuses to overwrite a different model. The model
is cached locally in `policies/policy_humanoid.onnx` and ignored by Git.

The demo starts the installed node, sends three synthetic observations over ROS,
and checks that all 22 named joint targets match independent ONNX Runtime
inference with the configured scaling/default pose. It prints PASS for stationary,
forward-command, and turning-command fixtures; saves observations and outputs in
`log/policy_demo.json`; and shuts down the child node. It uses domain 117 and
`/policy_demo/*` topics. Another domain can be selected with `--domain-id`.
No CAN adapter is needed. This checks inference and ROS messaging; it does not
simulate physics, demonstrate walking, or connect to the motor topics.

## MuJoCo feedback-loop diagnostic

An optional diagnostic runs the installed ROS node at Berkeley's 25 Hz policy
period. It builds each observation from the Berkeley robot's current MuJoCo
joint and IMU state, applies the resulting targets through Berkeley's PD gains
and torque limits, and advances the physics before the next observation.
It uses isolated `/policy_mujoco/*` topics and does not publish motor commands.

Place the Berkeley Humanoid Lite repository at `../Berkeley-Humanoid-Lite` and
check out revision `984741a3623c93b0583ccfdc479f1f8b1c4d900e` so its
`configs/policy_humanoid.yaml` matches the pinned checkpoint. Then run:

```bash
python -m pip install -r src/th_policy_node/requirements-sim.txt
python tools/verify_policy_mujoco.py
```

The script uses the Berkeley MJCF already in `th_description`. It writes
`log/policy_mujoco.json` with state change and the number of targets beyond
joint ranges at every step. Completing the loop shows the ROS outputs affect
simulated robot state. It is not a walking-quality or hardware-safety test.

To resume in a fresh terminal:

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
python tools/verify_policy_ros.py
```

## Model provenance

- Team repository revision: `984741a3623c93b0583ccfdc479f1f8b1c4d900e`.
- [Checkpoint](https://github.com/Texas-Humanoids/berkeley-humanoid-lite/blob/984741a3623c93b0583ccfdc479f1f8b1c4d900e/checkpoints/policy_humanoid.onnx)
  and [matching configuration](https://github.com/Texas-Humanoids/berkeley-humanoid-lite/blob/984741a3623c93b0583ccfdc479f1f8b1c4d900e/configs/policy_humanoid.yaml).
- SHA-256: `d3f096eda1b2444afce193fed82b9e1fa6d0421cc9ec1d450c549c30163c3a36`.
- Observation assembly and output scaling match the repository's pinned
  [lowlevel runner](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Lowlevel/blob/652777cc7c49884e7cd7ddfada758dc1979bf627/berkeley_humanoid_lite_lowlevel/policy/rl_controller.py).

| Input indices (zero-based) | Meaning |
| --- | --- |
| 0–2 | Requested forward, lateral, and yaw velocity |
| 3–5 | Body angular velocity |
| 6–8 | Projected gravity; upright fixture is `[0, 0, -1]` |
| 9–30 | Joint positions minus the default pose, in configured joint order |
| 31–52 | Joint velocities, in the same order |
| 53–74 | Previous clipped policy actions, before scaling or adding the pose |

The upstream policy period is 0.04 seconds (25 Hz). This ROS node executes once
per received observation; it does not own a control-loop timer. The demo holds
the fictional robot at its default pose and carries previous actions across
three steps. Fake zero relative joint positions mean the default pose, not
zero absolute joint angles.

## Contract

- Subscribe to `std_msgs/msg/Float32MultiArray` on `/policy/observation`.
- The message must already contain the complete, model-ready observation vector
  in the model's exact feature order. The default full Berkeley Humanoid Lite
  profile expects 75 `float32` values.
- Run the ONNX model on CPU and scale its action vector by `action_scale`, then
  add `default_joint_positions`.
- The model must have one float32 observation input and one float32 action
  output. A dynamic batch dimension is resolved to one; feature dimensions
  must match `expected_observation_size`.
- Publish the named target positions in `sensor_msgs/msg/JointState` on
  `/policy/joint_targets`.

The default joint order, standing pose, action scale, and observation size match
the pinned Berkeley configuration above. Other checkpoints may use a different
contract. The Controls/Middleware/Embedded interface review is still required
before connecting a downstream controller. The node does not construct
observations from IMU or joint-state topics.

For the matching Berkeley profile, the 75 input values are velocity command
(3), base angular velocity (3), projected gravity (3), relative joint position
(22), joint velocity (22), and previous policy action (22). This ordering is
confirmed against the pinned Berkeley inference code linked above.

## Run

Install the ROS workspace dependencies in the CPU or GPU development container,
then build from the repository root:

```bash
python3 -m pip install --break-system-packages -r src/th_policy_node/requirements.txt
colcon build --symlink-install
source install/setup.bash
```

Run `ros2 run th_policy_node fetch_berkeley_policy` or change `model_path`
in the parameter file to another ONNX file. Start the node from the repository
root:

```bash
ros2 run th_policy_node policy_node --ros-args --params-file src/th_policy_node/config/policy_humanoid.yaml
```

In a second sourced terminal, inspect named joint targets:

```bash
ros2 topic echo /policy/joint_targets
```

In a third sourced terminal, send one synthetic observation for the default
75-value profile (zero commands, velocities, relative positions, and previous
actions; projected gravity `[0, 0, -1]`):

```bash
observation="$(python3 -c 'import json; print(json.dumps({"data": [0.0] * 8 + [-1.0] + [0.0] * 66}))')"
ros2 topic pub --once /policy/observation std_msgs/msg/Float32MultiArray "$observation"
```

This exercises inference and message output with a compatible model. It does
not establish walking performance or suitability for commanding hardware.
No model is bundled; provide a policy whose feature and action conventions
match the configuration. The action limits clip model actions, not physical
joint limits.

Connecting the gamepad and motors is the separate issue #18. This package's
output is diagnostic joint targets, not a motor-enable or motion-command topic.

An incorrect observation length, non-finite input, inference error, or action
count that does not match the configured joints is rejected without publishing.
Non-finite targets after scaling are also rejected.

## Focused checks

With NumPy installed, run from the repository root:

```bash
python3 -m unittest discover -s src/th_policy_node/test -v
python -m colcon test --packages-select th_policy_node
python -m colcon test-result --verbose
```

These boundary tests cover shape/type rejection, invalid inputs and outputs,
inference failures, scaling/clipping, and target overflow. They use real NumPy
with ROS and ONNX Runtime stand-ins. The separate demo above checks the installed
ROS executable and the actual Berkeley model; neither check proves hardware
control or walking performance.
