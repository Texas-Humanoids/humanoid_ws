# Demo walker bring-up: first power-on to walking on the gamepad

The order matters: each phase proves something the next one depends on. **Don't skip ahead
because the previous phase "probably worked."** Every phase has a pass check. If it fails, stop and fix it.

| Phase | What it proves | Tool | Robot | Time |
|---|---|---|---|---|
| 0 | Files on the Jetson, safety kit ready | scp, pytest | off | 15 min |
| 1 | Power and CAN: all 10 drives answer, no faults | `can_status.sh`, `jog.py` | on stand, legs hanging | 10 min |
| 2 | Each actuator moves both ways, smoothly, nothing binds | `jog.py` (e, arrows, r) | hanging | 20 min |
| 3 | Directions: motor + = joint + on every joint | `jog.py` (p, then y/n) | hanging | 15 min |
| 4 | Offsets, hard stops, and which policy is allowed | `jog.py` (z z, seen, w) | hanging, held in the zero pose | 20 min |
| 5 | All 10 hold together; the default pose looks right | `jog.py` (E, g, s) | hanging | 10 min |
| 6 | ROS stack wiring: IMU frame, gyro sign, observation, gamepad | `robot.launch.py`, disarmed | hanging | 15 min |
| 7 | STAND on PD only | START | gantry slack, feet on the floor | 10 min |
| 8 | Policy walking | A, sticks | gantry catching | rest |

Joint convention (same for both legs; matches the URDF, the policy and `wiggle_test.py`):

| Joint | + means |
|---|---|
| hip pitch | foot swings **backward** |
| hip roll | foot moves **left (+y)**: the left leg goes out, the right leg goes in |
| hip yaw | toes turn **left** (counter-clockwise seen from above) |
| knee | **flexes** |
| ankle | toes go **down** |

`motor = direction * joint + offset`. Phase 3 finds `direction` (+1 or -1), and phase 4 finds `offset`.

---

## Phase 0: before you leave (laptop)

1. Copy to the Jetson. Nothing is committed yet, so use scp:
   ```bash
   J=humanoids@100.118.244.11:~/humanoid_ws
   cd /c/Users/karti/OneDrive/Documents/humanoids_sw/humanoid_ws_sysid
   scp -r tools/walker/bringup $J/tools/walker/
   scp src/th_bringup/config/walker_demo_robot.yaml $J/src/th_bringup/config/
   scp -r policies/walker_demo $J/policies/
   ```
2. On the Jetson, check the tool against simulated motors (no hardware needed):
   `python3 -m pytest tools/walker/bringup/test_jog.py -q` should pass all 11. Then try the
   dashboard: `python3 tools/walker/bringup/jog.py --sim`. Widen the terminal to at least 135 columns.
3. Gamepad: `python3 tools/walker/bringup/gamepad.py`. Press LB, A, B, X, Y and move the left stick and
   d-pad. If the numbers aren't the Xbox ones in `gamepad.py`, write a map json and pass `--map`.
4. Safety kit (non-negotiable):
   - A **hard power cut** on the motor bus that someone other than the laptop operator holds. `jog.py`'s
     q and the gamepad's B are software. The kill switch is the real e-stop.
   - A **gantry or hoist** that can hold 17 kg, with the strap at the pelvis.
   - Bench PSU at 36 V with the **current limit at ~3 A** for phases 1-5. Raise it to ~15-20 A for phase 7+,
     or switch to the battery.

## Phase 1: power and CAN (legs hanging free)

```bash
ssh -t humanoids@100.118.244.11
cd ~/humanoid_ws
tools/jetson/can_status.sh              # both adapters up, which motor IDs on which adapter
sudo python3 tools/walker/bringup/jog.py --robot src/th_bringup/config/walker_demo_robot.yaml \
    --spec policies/walker_demo/cad_v1/policy_spec.json
```
`jog.py` configures every drive (MIT mode, a soft torque limit of 15/5/10 Nm, a 0.2 s CAN timeout,
each one read back) and leaves them **limp**. It refuses to start if anything else is sending motor
commands, so stop `robot.launch.py` first.

**Pass:** 10 rows; VBUS in the top bar 35.5-39 V; no row highlighted (fault); temperatures near ambient;
moving a joint by hand changes its `q_motor`.
**Note the `iface` per ID.** On the bench, `can_left` carried the RIGHT leg. Fix `can_interface` in the
yaml (or the cables) now, or `robot.launch.py` will talk to the wrong leg later.

## Phase 2: each actuator alone (hanging)

Order: ankles, knees, hip yaw, hip roll, hip pitch (lightest load first). For each joint:

1. **Limp check.** Move it slowly by hand from stop to stop. Feel for binding and cable pull, and watch
   the hip-yaw wiring. The `seen` column records the range.
2. **e** enables it: it holds where it is.
3. **Up arrow** once or twice (0.1-0.2 rad/s). It moves smoothly. **space** stops it, **r** reverses it.
   Down arrow past zero also reverses. The bar shows speed and direction.
4. Watch `tau` while it moves: about 0-2 Nm for ankle and knee unloaded. A big constant torque means
   it's rubbing.
5. **e** again turns it limp. Move to the next joint with **right arrow**.

Gamepad: hold **LB** and push the left stick (analog speed). Let go of LB and it stops (dead-man).

Built-in protection: a +-0.6 rad window around where you enabled it ("AT WINDOW"). Stall detection: if
the tracking error goes over 0.15 rad, it hit something, so the target freezes and the row shows "STALL".
Any fault, mode drop, temperature over 70 C or 100 ms without feedback on ANY enabled motor makes
everything damp, then go limp.

**Pass:** all 10 move both ways without grinding, stalls or faults.

## Phase 3: directions (hanging)

For each joint: select it, press **p**. It moves +0.15 rad in motor-positive and comes back. The
bottom line asks *"did it go '<+ means>'?"* Press **y** or **n**, and `dir` gets a `*` (checked).

- Do the left leg, then the right. Mirrored mounting means the right side is usually the opposite
  sign on pitch joints. Don't assume it: answer what you see.
- Hip roll is the confusing one: **+ is foot to the robot's LEFT for both legs.** On the right leg that's
  inward.
- Unsure? Press p again. Wrong answer? **f** flips it.

**Pass:** all 10 rows show `*`.

## Phase 4: offsets, hard stops, which policy (hanging)

1. Hold the robot in the **zero pose**: legs straight down, feet flat and parallel to the floor, toes
   forward. Use a square on the shank and a level or phone inclinometer on the sole. The knee is at
   full extension, which is usually its hard stop and an exact reference.
2. **z**, then **z** within 3 s. `q_joint` now reads about 0 everywhere.
3. **Check it by eye:** bend one knee by hand to roughly 45 degrees, and `q_joint` should say about +0.8.
   Swing a foot back, and hip pitch goes +.
4. **Hard stops.** Press **c** on a joint to clear its `seen` range, then move it limp **by hand** gently to
   each mechanical stop. `seen` shows J[min, max] in joint radians. (Or jog slowly with **u** to unlock the
   yaml limits. Stall detection catches the stop.)
5. **w** writes direction and offset into the yaml and the ranges into `bringup_session.json`.
6. **Power-cycle test.** Cut motor power, restore it, restart `jog.py` and hold the zero pose again.
   `q_joint` must read within +-0.02. If it's off by a multiple of 2 pi / gear ratio (about 0.7 rad on a
   9:1 RS03), the drives don't keep absolute position across power cycles. Then re-run steps 1-2 at
   **every power-up** (2 minutes in the gantry), and tell the team.

**Pick the policy from the measured stops:**

| | hip roll inner stop | knee max | policy |
|---|---|---|---|
| CAD as exported | 0 (abduction only) | 60 deg (1.05 rad) | `policies/walker_demo/cad_v1` |
| WalkerV2-like | at least 0.35 rad (20 deg) of adduction | at least 1.6 rad | `policies/walker_demo/wide_v1` (cad_v1 is also fine) |
| in between | | | cad_v1 |

`cad_v1` is safe on any robot whose real stops are at least the CAD limits. Then set `lower`/`upper` in
the yaml to the chosen policy's `joint_limits` (in its `policy_spec.json`), but never past a measured
stop minus 0.05 rad. The bridge clamps targets to them.

## Phase 5: everything holding (hanging)

1. **E** enables all 10. Push each leg: stiff, no buzzing or oscillation.
2. **g** ramps to the policy default pose over 3 s (needs `--spec`). Both knees bend **forward** the same
   amount, feet stay flat, and the legs are mirror images. A leg going the wrong way means its direction
   is wrong: go back to phase 3.
3. **s** does a slow squat (3 x 4 s), in the air.
4. Restart with the policy's gains:
   `sudo python3 tools/walker/bringup/jog.py --robot <yaml> --spec <spec> --policy-gains`,
   **E**, **g**. Listen for buzzing at kp 120 (knee) and 100 (ankle, hip roll). Buzz means stop; note which
   joint and report back. (kd may need to drop.)
5. Optional demo check, feet on the floor, gantry slack: with `--policy-gains`, **E**, **g**, then **s**.
   In sim the squat stands on PD alone (knee +0.25 rad, 0.8x kp, no fall).

**Pass:** the default pose looks like the sim pose (knees ~23 deg, feet flat, symmetric). Quit `jog.py`
(**q**) before phase 6.

## Phase 6: the ROS stack, disarmed (hanging)

```bash
ros2 launch th_bringup robot.launch.py \
  motors_config:=src/th_bringup/config/walker_demo_robot.yaml \
  policy_spec:=policies/walker_demo/cad_v1/policy_spec.json \
  model_path:=policies/walker_demo/cad_v1/policy.onnx walk_event:='A pressed'
```
The robot starts **disarmed**: motors damp. In a second terminal:

| Check | Command | Expect |
|---|---|---|
| IMU mounting | `ros2 topic echo /policy_bridge/projected_gravity` | upright 0 0 -1, nose down +x, left side down +y. If not, fix `imu.rotation` |
| gyro sign | `ros2 topic echo /imu/data` while you yaw the robot left (CCW from above) | angular_velocity.z > 0 |
| joint mapping | `ros2 topic echo /policy/observation` in the zero pose | the 10 joint_pos_rel entries = minus the default pose. cad_v1: left (0.2, -0.05, 0, -0.4, 0.2), right (0.2, +0.05, 0, -0.4, 0.2) |
| drive torque limit | `sudo python3 tools/walker/sysid/monitor.py --once` (with launch stopped) | the motor node writes model max (60/17/36) at start, replacing jog's 15/5/10 |
| gamepad | `ros2 topic echo /cmd_vel` | stick centered reads ~0 (drift: see note) |

Stick drift: on 10-01 the controller read vx 0.04, vy 0.05 with the stick "centered". The demo policies
were tested with that drift (sim2sim's last segment) and stand still, but a dead zone (~0.1) in
`gamepad_node` is still the right fix.

## Phase 7: STAND (gantry slack, feet on the floor)

1. Lower the gantry until the feet are 1-2 cm off the floor. **START**: a 2 s ramp to the default pose.
2. Lower until the feet carry the weight, with the strap slack but ready to catch.
3. The demo policies' gains **stand on PD alone** (in sim down to 0.7x kp with 15 ms delay and a
   0.3 m/s push), so there's no stand_kp override. Gentle pushes front, back and sides: it should recover
   or sit into the gantry, never fight itself.

## Phase 8: walking (gantry catching)

1. **A** starts the policy. At zero command it should stand quietly (no stepping).
2. Forward a little (~0.1-0.2 m/s), then stop, then back, then turn. Short bursts.
3. **BACK** disarms: motors damp and the robot **drops**. The gantry must be catching.
4. Abort (BACK, then the kill switch if needed) on: a joint hitting a stop, buzzing or oscillation, feet
   dragging or scuffing, lean past ~15 degrees, or any `feedback late` in the launch log.
5. Known open issue: the about 820 ms `robstride_motor_node` stall from 11-15 (once per about 110 s on
   the bench). On the robot it's a joint going limp mid-step. Keep the gantry on until it's root-caused.

## Files

| File | What |
|---|---|
| `jog.py` | the dashboard (this README's phases 1-5) |
| `gamepad.py` | joystick reader; `python3 gamepad.py` prints raw events |
| `test_jog.py` | 11 tests against simulated drives (`chirp.SimMotor`) |
| `../../../src/th_bringup/config/walker_demo_robot.yaml` | motor map; `w` in jog.py fills direction/offset |
| `../../../policies/walker_demo/` | the two policies + specs + sim2sim results |
