# WalkerV2 policy v6

`v6_sym`, trained in Isaac Lab on 2026-11-13 (800 PPO iterations, left-right
mirror symmetry). The training team's recommended hardware candidate: even
pacing in every direction, about 0.2 m of sideways drift per 5 m, and no
falls in the start/stop/turn scenario or in 30 random-command episodes with
pushes. Training code, run log and videos are in the training workspace
(`training/walker_isaaclab/`, `RUN_LOG.md`).

| File | |
|---|---|
| `policy.onnx` + `policy.onnx.data` | the network (41 → 10). The weights live in `.onnx.data`, which must stay next to `policy.onnx` |
| `policy_spec.json` | the deploy interface, read by `robot.launch.py policy_spec:=...` |
| `env.yaml`, `agent.yaml` | the exact Isaac Lab env and PPO configs it was trained with |

```bash
ros2 launch th_bringup robot.launch.py \
  motors_config:=src/th_bringup/config/walker_v2_robot.yaml \
  policy_spec:=policies/walker_v2/v6/policy_spec.json \
  model_path:=policies/walker_v2/v6/policy.onnx walk_event:='A pressed'
```

## Interface (checked against `env.yaml`)

- 50 Hz (sim dt 0.005 × decimation 4). Actions: `target = default + 0.25 · action`,
  joint order left then right, each leg hip pitch, hip roll, hip yaw, knee, ankle.
  Training had no clip; the spec clips |action| ≤ 4 on hardware.
- Observation (41): angular velocity × 0.25, projected gravity, command
  (vx, vy, wz), joint positions − default, joint velocities × 0.05, last
  action, gait phase (sin, cos) with a 0.7 s period starting when the policy starts.
- PD gains (MIT mode), which the hardware must use unchanged: hip pitch, hip
  roll and knee kp 60 / kd 3; hip yaw 25 / 1; **ankle 30 / 1.5**.
- Commands trained: vx −0.3…0.5 m/s, vy ±0.1, wz ±0.5 rad/s; 20 % standing.

## Why ankle kp is 30, and what it means on the robot

The gains are part of what the policy learned, so the robot runs the
training gains and kp 30 is settled for v6. Changing it would need
retraining. The consequence is that the robot **cannot stand on PD alone**.
At kp 30 the ankles give way (it falls in about 2 s in both simulators;
standing passively needs about 80). It stays up only while the policy is
actively balancing. So STAND (the ramp to the default pose) and anything
after a disarm must happen on the gantry. See the procedure in the
[th_bringup README](../../../src/th_bringup/README.md).

## Results

| Test | Result |
|---|---|
| Training team's MuJoCo sim2sim (`sim2sim.py`), 0.3 m/s, 15 s | no fall, 3.83 m, 0.263 m/s |
| **Our deploy stack** (`sim:=true`: real `policy_node`, `policy_bridge` and launch against MuJoCo), 0.3 m/s, 15 s | no fall, **3.93 m**, base ≥ 0.643 m |
| Our deploy stack, zero command, 15 s | stands, moves under 1 cm |
| Jetson Orin Nano, 2-motor bench (left hip pitch + knee, other joints read as default, IMU flat) | loads and runs at 50 Hz. Motors follow the 0.70 s gait rhythm (12 mrad hip tracking at zero command). The stick changes the motion: forward 0.5 m/s widens the knee swing from 0.36…0.45 to 0.08…1.37 rad; back −0.5 shifts the hip swing forward |

The bench run is open-loop (no body responds), so it shows the interface and
command response, not balance. The knee lagged by about 0.4 rad at 0.5 m/s
because the bench caps torque at 5 Nm.

Known (from the run log): 6-8° nose-down lean when standing at 0-0.05 m/s;
hip roll above its 20 Nm rated torque about 23 % of the time while walking
(peak is 60 Nm, so watch hip-roll temperatures).
