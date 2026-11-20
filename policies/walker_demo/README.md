# Demo walker policies (2026-11-17)

Two velocity-tracking policies for the demo walker (`pkg_002`, 16.87 kg, 10 DOF, no ankle roll).
Both were trained in **mjlab** (MuJoCo Warp, 4096 envs, 3000 PPO iterations, 40 min each on the 5060 Ti)
from `humanoid_ws/training` (`TH-Velocity-Flat-WalkerDemo-Cad` / `-Wide`). They differ **only in the
joint limits** they were trained with:

| | hip roll | knee | use when |
|---|---|---|---|
| `cad_v1` | abduction only (CAD: 0 inward) | 0-60 deg (CAD) | default. Safe on any robot whose real stops are at least the CAD limits |
| `wide_v1` | 20 deg inward (WalkerV2) | 0-120 deg | only if jog.py measures at least that range on the real robot |

Deploy (folder = `cad_v1` or `wide_v1`):
```bash
ros2 launch th_bringup robot.launch.py \
  motors_config:=src/th_bringup/config/walker_demo_robot.yaml \
  policy_spec:=policies/walker_demo/cad_v1/policy_spec.json \
  model_path:=policies/walker_demo/cad_v1/policy.onnx walk_event:='A pressed'
```
Bring-up procedure first: `tools/walker/bringup/README.md`.

## Interface (`policy_spec.json`, read by robot.launch.py)

- 50 Hz. `target = default + 0.25 * clip(action, +-4)`. Joint order: left then right; hip pitch, hip roll, hip yaw, knee, ankle.
- Observation (41): base_ang_vel, projected_gravity, joint_pos_rel, joint_vel, last_action,
  velocity_command (vx, vy, wz), gait_phase (sin, cos; period 0.72 s from WALK start). All scales 1.
- MIT gains, which the drives must use unchanged: hip pitch 80/4, hip roll 100/5, hip yaw 30/1, knee
  120/4, ankle 100/2.5. They **stand on PD alone** (sim: down to 0.7x kp, 15 ms delay, 0.3 m/s push), so
  STAND needs no stand_kp override.
- Default pose: hip pitch -0.2, knee 0.4, ankle -0.2; hip roll **+-0.05 (cad_v1)** / 0 (wide_v1).
- Trained commands: vx -0.3..0.6 m/s, vy +-0.2, wz +-0.8 rad/s, 20% standing.
- Actuators: measured armature, friction and 36 V top speed (sysid 11-15); 0-15 ms command delay;
  kp/kd x0.8-1.2 / x0.7-1.3; base mass x0.9-1.25; encoder bias +-0.03 rad; pushes every 3-6 s.

## Validation

Same 46 s script on every run: stand, forward 0.2 / 0.4, stop, back 0.2, sideways +-0.15, turn +-0.6,
forward+turn, stick drift (0.04, 0.03, 0), stand; with 4 velocity pushes (0.25-0.3 m/s). The observation
is built from `policy_spec.json` the way `th_policy_bridge` builds it. Cells are achieved / commanded,
mean over each segment's last 60%.

### cad_v1: MuJoCo (`sim2sim_demo sweep`, independent of mjlab's code)
See `cad_v1/sim2sim_sweep.txt`. **10/10 survived** (incl. 25 ms delay, mu 0.4, 30 V, combined).
Nominal: fwd 0.18/0.20, 0.37/0.40; back -0.17/-0.20; turn +0.52/+0.60, -0.57/-0.60; sideways 0.

### cad_v1: Isaac Sim 5.1 / PhysX (`isaac_check_demo`, a different contact solver)
See `cad_v1/isaac_sweep.log` / `.json`. **9/9 survived**, plus nominal and combined at foot friction 0.4 (`isaac_mu04.log`): **11/11**.
Nominal: fwd 0.19/0.20, 0.36/0.40; back -0.15/-0.20; turn +0.45/+0.60, -0.47/-0.60; sideways 0.
PhysX turns ~15% slower than MuJoCo; everything else agrees within a few cm/s.

### wide_v1: MuJoCo
See `wide_v1/sim2sim_sweep.txt`. **10/10 survived.** Nominal: fwd 0.20/0.20, 0.39/0.40; back
-0.17/-0.20; turn +-0.53/0.60; sideways ~0.

### wide_v1: Isaac / PhysX
See `wide_v1/isaac_sweep.log`. **9/9 survived, plus 2/2 at friction 0.4.** Forward and back match
cad_v1, but **turning drops to about 60% of the command** (+-0.37/0.60 nominal; -0.21/-0.60 combined),
where cad_v1 keeps 75-80%. So wide_v1 relies more on MuJoCo's contact behaviour when it turns.

### Which one
**cad_v1** unless the robot's stops are WalkerV2-like **and** you need what the extra range buys. On
current evidence the extra range buys nothing (neither sidesteps), and cad_v1 transfers better to PhysX.

## Known limits

- **Neither policy steps sideways** (vy tracks ~0). Not purely the CAD hip-roll limit, since wide_v1 doesn't
  either: the policies learned to ignore the small (+-0.2) vy commands. Fix in a v2: a larger lateral
  command share and a tighter tracking std, or vy-only episodes. For the demo, drive with vx + wz.
- Effort, nominal run: ankle peaks at 80-83% of the RS06 36 Nm peak (short transients; RMS 0.41-0.45 of
  rated); hip roll RMS 0.47 (wide) / 0.67 (cad) of rated. cad_v1 works hip roll harder; watch its
  temperature in long demos.
- Peak joint speed is under half of the 36 V top speed (knee 0.36-0.47), so the battery sagging to
  30 V changes nothing.
- Trained on flat ground with friction 0.4-1.2. Not trained on slopes, steps or carpet edges.
