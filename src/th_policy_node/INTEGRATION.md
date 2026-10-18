# Integration handoff

The policy node implements the inference part of issue #17. Its ROS interfaces
are deliberately separate from actuator commands. Issue #18 connects the nodes.

| Producer | Topic / type | Consumer requirement |
| --- | --- | --- |
| Gamepad node | `/cmd_vel`, `geometry_msgs/msg/Twist` | Use `linear.x`, `linear.y`, and `angular.z` as the first three policy observation values. The current gamepad node supplies forward and turn; lateral defaults to zero. |
| Observation builder (needed) | `/policy/observation`, `std_msgs/msg/Float32MultiArray` | Publish 75 finite values in the Berkeley order below after receiving fresh, correctly mapped robot state. |
| Policy node | `/policy/joint_targets`, `sensor_msgs/msg/JointState` | Receives 22 named position targets in radians. These are diagnostic policy outputs, not actuator commands. |
| Single RS06 motor node | `/motor/target_position`, `std_msgs/msg/Float64` | Accepts one actuator position. It cannot consume the 22-joint message directly. |

The observation builder needs the commanded velocity (3), body angular velocity
(3), projected gravity in the body frame (3), positions relative to the default
pose (22), joint velocities (22), and previous clipped policy actions (22).
Joint measurements must be reordered by name to match
`config/policy_humanoid.yaml`. The existing motor node provides only one motor;
it cannot supply complete state for the 22-joint policy. An IMU interface and
the remaining joint feedback are also needed for physical deployment.

The downstream controller must establish the mapping from Berkeley joint names
and radians to each real motor's CAN ID, direction, zero offset, gearing, and
physical travel. It must reject stale or out-of-range targets and define an
explicit enable/stop sequence. The policy node does not arm motors. In the
current motor PR, `/motor/simulated_position` remains simulated feedback even
though its latest commit can send real CAN commands.

The MuJoCo feedback-loop check in `tools/verify_policy_mujoco.py` exercises this
policy with evolving simulated state. It also reports policy targets outside
the model's joint ranges. That check helps review the observation and control
contract; it does not validate robot stability or authorize a hardware bridge.
