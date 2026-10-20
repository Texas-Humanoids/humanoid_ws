# Issue #17 validation

Validated locally on 2026-10-05 using Ubuntu 24.04 in WSL, ROS 2 Jazzy,
Python 3.12.3, NumPy 1.26.4, and ONNX Runtime 1.22.1 (CPU).

## Result

The installed `th_policy_node` executable loads the real Berkeley humanoid
checkpoint and publishes named joint angles from synthetic ROS observations.
This demonstrates the initial policy-loading/output goal of issue #17.

- `python -m colcon build --packages-select th_policy_node --symlink-install`: passed.
- `python -m colcon test --packages-select th_policy_node`: 11 tests passed.
- `python -m colcon test-result --verbose`: 0 errors, 0 failures, 0 skipped.
- `python tools/verify_policy_ros.py`: all three ROS/model cases passed.
- `git diff --check`: passed.

An additional MuJoCo diagnostic was run on 2026-10-18 with the same pinned
Berkeley policy and configuration, the `th_description` Berkeley scene, and
MuJoCo 3.3.7. Over 25 policy steps (one simulated second), ROS policy targets
drove changing simulated joint state; the maximum joint change was 0.251 rad.
All 25 steps had at least one target beyond the MJCF joint ranges. This is a
concrete integration warning, not a walking success. The diagnostic records
each step in `log/policy_mujoco.json`; its scene, initial state, and PD control
follow the Berkeley configuration, but it does not establish that the policy
is suitable for physical actuation.

The actual model input is `obs`, float32 shape `[1, 75]`; its output is
float32 shape `[1, 22]`. See the README for the pinned source and SHA-256.
Each case publishes a 75-value message and receives a 22-position JointState
with the expected names/order. All positions are finite and match direct
execution of the same model, followed by Berkeley's clipping, scaling, and
default-pose addition (maximum absolute error: 0.0 rad in each case).

| Synthetic case | Requested velocity (forward, lateral, yaw) |
| --- | --- |
| Stationary | `[0.0, 0.0, 0.0]` |
| Forward | `[0.2, 0.0, 0.0]` |
| Turn | `[0.0, 0.0, 0.2]` |

The fixtures keep an upright, stationary robot at the default pose and carry
the previous clipped action into the next observation. They do not evolve
joint state with physics. The generated `log/policy_demo.json` contains every
input and output; `log/policy_demo_node.log` records node startup/shutdown.
These generated logs and the downloaded checkpoint are local, ignored files.

## Scope remaining with the team

This validates inference and ROS message transport, not walking quality,
physical joint limits, timing at 25 Hz, or performance on the team's robot.
The published targets are diagnostic outputs; even a stationary synthetic
input can produce large joint targets and must not be treated as a safe pose.

Issue #18 owns gamepad/feedback observation assembly and downstream motor
integration. Before that connection, the team must agree on joint mapping,
calibration, physical limits, enable/stop behavior, and command timing. The
Berkeley checkpoint provides a concrete baseline for this task, not evidence
that it is suitable for a different robot configuration.
