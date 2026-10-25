# Policy node

This C++ ROS 2 node runs the Berkeley Humanoid Lite ONNX policy. It subscribes
to a complete, model-ready observation and publishes named joint targets. It
does not assemble observations, enable motors, or publish actuator commands.
Issue #18 owns the gamepad/feedback integration and motor interface.

## Build and run

Use Ubuntu 24.04 with ROS 2 Jazzy and the ONNX Runtime **C++ SDK 1.22.0**.
The CPU and GPU Dockerfiles install that SDK at `/opt/onnxruntime`. For a local
install, point `CMAKE_PREFIX_PATH` at the SDK directory and make its `lib`
directory visible to the dynamic linker.

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/fetch_berkeley_policy.py
colcon build --packages-select th_policy_node
colcon test --packages-select th_policy_node
colcon test-result --verbose
source install/setup.bash
ros2 run th_policy_node policy_node --ros-args \
  --params-file src/th_policy_node/config/policy_humanoid.yaml
```

The fetch script pins the team's Berkeley checkpoint to revision
`984741a3623c93b0583ccfdc479f1f8b1c4d900e` and verifies SHA-256
`d3f096eda1b2444afce193fed82b9e1fa6d0421cc9ec1d450c549c30163c3a36`.
The model is ignored by Git. Alternatively, set `model_path` to a compatible
ONNX file. The process fails at startup if the path, model tensor types/shapes,
or configured joint profile is invalid.

## Interface

| Topic | Type | Meaning |
| --- | --- | --- |
| `/policy/observation` | `std_msgs/msg/Float32MultiArray` | One complete 75-value Berkeley observation. |
| `/policy/joint_targets` | `sensor_msgs/msg/JointState` | 22 named target positions in radians, stamped when computed. |

The observation order is: requested forward/lateral/yaw velocity (3), body
angular velocity (3), projected gravity (3), joint positions relative to the
default pose (22), joint velocities (22), and previous clipped actions (22).
The joint order and default pose are defined **once** in
`config/policy_humanoid.yaml`. Incoming observations must already use that
order. The node rejects wrong-length or non-finite observations and invalid
model outputs without publishing targets.

The model output is clipped to the configured action range, multiplied by
`action_scale`, and added to the configured default pose. These action limits
are **not physical joint limits**. The node receives an un-stamped array, so
it can timestamp output but cannot determine the age of the source state;
the observation builder must enforce freshness.

## Verify

Install the Python validation dependencies with
`python3 -m pip install -r src/th_policy_node/requirements.txt` in a ROS-aware
virtual environment. Then run:

```bash
python3 tools/verify_policy_ros.py
```

The five Google Test cases cover model tensor shapes, action clipping and
scaling, invalid values, and target overflow. The ROS script publishes three
synthetic observations through the installed C++ node and compares all 22
outputs with independent inference from the same
checkpoint. It also checks that invalid observations produce no targets.
Its isolated ROS domain and topics keep the check off the motor interface.

Earlier validation of the Python prototype passed 11 boundary tests and a
MuJoCo diagnostic. The diagnostic drove changing simulated joint state over
25 policy steps, but at least one target exceeded model joint limits on every
step. Those prototype results do not validate physical walking. The C++
version's Google Tests and repeatable ROS/model check above are the current
acceptance checks.

## Integration boundary

The gamepad publishes `/cmd_vel` (`geometry_msgs/msg/Twist`), not a model-ready
observation. An observation builder must combine it with fresh IMU and all 22
joint positions and velocities. The current RS06 motor node accepts one
`std_msgs/msg/Float64` target, not this 22-joint `JointState`. A downstream
controller must map joint names to motors, apply calibrated directions/offsets
and physical limits, and own enable/stop behavior before hardware can move.
