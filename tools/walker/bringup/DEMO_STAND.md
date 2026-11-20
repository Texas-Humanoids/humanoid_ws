# Stand-only demo (no policy)

The robot holds the stand pose on stiff joint PD and keeps its balance against gentle nudges.
No policy controls the motors. This is the bridge's STAND mode, with every safety check still on:
motor faults, stale CAN feedback, the joint speed trip and B to disarm.

## THE DEMO ROUTINE (proven 2026-11-19: 94 s hands-off through gentle nudges)

On the Jetson, from `~/humanoid_ws`:

1. `tools/walker/bringup/ros.sh boot`: both CAN adapters UP, clocks locked. Run it after every power-up.
2. `tools/walker/bringup/ros.sh launch stand-auto foxglove:=false`
   * It must end with **`all joints in range`**. If a drive booted one turn off (the right hip
     pitch does this), the launch fixes the offset itself and restarts once. If it says
     **DO NOT press START**, stop and get help.
   * `tools/walker/bringup/ros.sh health` should show 10 drives, 0 dropped out, and the controller connected.
     Wait about 20 s after launch.
3. Rope taking some weight, feet flat → **Menu (START)**. It ramps over 2 s into the balanced pose
   (saved lean −0.050, `stand_balance: lean` in the robot yaml).
4. Let the rope out slowly until it's slack. The auto-lean finds the balance point by itself,
   holding +1.5 N m of ankle torque and correcting at most ±0.03 rad. The D-pad still works
   (up = forward, down = back) if it ever needs help.
5. Show it standing; **gentle fingertip nudges only**. It cannot step, and the real usable sole is
   small (~0.13–0.18 m/s pushes in sim).
6. **B** → the motors go soft, so catch it with the rope. **START** again to restand; no relaunch is needed.
7. Done: `tools/walker/bringup/ros.sh stop`.

**Rules:** rope attached throughout, one person on the controller, one spotting. **If the motor
power is switched off and on, relaunch (step 2) before START.** The drives come back disabled and
the hip pitch may be a turn off. Fallback mode: `ros.sh launch stand` (fixed lean, no auto).

## Why this works, and its limits (first principles)

* Without stepping, the robot survives a push only while its capture point (COM + v/omega,
  omega = sqrt(g/h) = 4.24 1/s) stays over the feet. Sole: heel -6.5 cm, toe +8.7 cm, outer edges
  +-22 cm. So the hardest push any non-stepping controller can survive is about
  **0.34 m/s forward, 0.30 m/s back, 0.9 m/s sideways**. That includes a policy that doesn't step.
* Stiff PD on the stand pose already reaches that bound in sim (0.34 m/s forward and back).
* An off-center COM is what costs margin. The real COM is probably about 3 cm behind the CAD,
  and the ankle calibration is set by eye, so the margin drops to about 0.23 m/s on one side.
* **stand-balance** fixes that. It leans the robot slowly on its ankles until the mean ankle
  torque is ~0, which puts the COM over mid-foot. It needs no IMU: the drive's torque is
  kp(target - q) - kd v. In sim the weakest direction goes from 0.23-0.25 to 0.33 m/s with COM or
  calibration errors, and it is never worse than PD alone.
  (`th_training/stand_balance.py`; C++ in `th_policy_bridge/stand_balance.hpp`, identical output.)
* A first, faster version of the trim was **worse** than PD (0.19 m/s, and a rocking oscillation
  with ankle offsets). The shipped one is slow, filtered and frozen during pushes. Don't raise
  `stand_balance.gain` above 0.0015 without rerunning `python -m th_training.stand_balance sweep`.

What to show: it stands, holds its full weight, and resists a **gentle** hand nudge at the
chest or hip. It will NOT recover from a real shove: there is no stepping.

## Before the demo (once, with the Jetson free)

1. Deploy the bridge (branch `walker/bridge-motor-safety`: stand balance, held gait clock,
   safety), `robot.launch.py`, `walker_demo_robot.yaml`, `ros.sh`. Build:
   `tools/walker/bringup/ros.sh build`.
2. Unit tests in the container: `colcon test --packages-select th_policy_bridge` and check the
   gtests (CenteringTrim, HeldGaitClock) and the pipeline tests pass. Known: 1-2 pipeline tests
   are flaky under load, before these changes too. Re-run any that fail.
3. Optional end-to-end with the simulated robot (no motors):
   `ros2 launch th_bringup robot.launch.py sim:=true ... walk_event:='(walking disabled: stand-only demo)' stand_balance_gain:=0.0015`
   then START. The log shows `STAND balance: lean ...` settling within +-0.03 rad.
4. Hardware: the repaired ankle passes `bringup.py pose default` and `check-cal` (worst < 0.05 rad).
   Both USB-CAN adapters plug straight into Jetson ports, not the hub, with strain relief and no
   cable pulled when the legs move. Battery charged.

## Run it

| Step | Command / action | Look for |
|---|---|---|
| 1 | `tools/walker/bringup/ros.sh boot` (after any power-up) | both CAN adapters UP, clocks locked |
| 2 | `tools/walker/bringup/ros.sh launch stand` (PD only, first time) | prints `limits: torque caps [...] speed trip [...]`; `ros.sh health`: 10 drives, controller connected |
| 3 | Robot **hanging, feet off the floor**: press **Menu (START)** | 2 s ramp to the stand pose. Listen: no buzzing or chatter with the stiffer stand gains |
| 4 | Lower it slowly onto its feet; let the tether go slack, but keep it attached | it holds its height, knees bent, upright |
| 5 | Hold 1 min, then gentle nudges (fingertip, chest height) | small sway, comes back |
| 6 | **B** | motors damp, it sinks into the tether |
| 7 | If 2-6 looked good: `ros.sh stop`, `ros.sh launch stand-balance`, repeat 3-6 | log: `STAND balance: lean +x.xxx rad ...` every 2 s, settles within +-0.03 rad within ~30 s |

Use stand-balance for the demo if step 7 looked as good as or better than PD only. Otherwise use
`launch stand`. **A does nothing in both modes.** The policy node runs, but no button gives it
the motors.

## When something goes wrong

| Symptom | What it is | Do |
|---|---|---|
| START does nothing | the bridge refused: `ros.sh health` shows why (stale feedback = a CAN adapter dropped) | `ros.sh restart stand` (or `stand-balance`) |
| Buzz/chatter in STAND | stand gains too stiff for the real drives | edit `stand_kp`/`stand_kd` in the robot yaml (drop 20%), relaunch |
| Slowly leans to one side and stays there | COM/calibration offset; PD only | use `stand-balance`; or re-check `check-cal` |
| stand-balance lean pinned at +-0.06 | a large offset (> ~3 deg), or a sign error | B; check `check-cal`; fall back to `launch stand` |
| Heel-toe rocking that grows | (seen in sim only with the old fast trim) | B; fall back to `launch stand` |
| Collapses mid-demo | the bridge disarmed: CAN dropout, a motor fault or the speed trip; the reason is in `ros.sh health` / log | catch it (spotter); `ros.sh restart ...` |

**Always:** tether attached with a little slack, one person on the controller (B), one person
spotting. A USB-CAN dropout makes the robot go limp. That happened 4 times on 2026-11-19, so
the cabling fix matters more than anything above.

## Limits (so nothing swings out of control)

`ros.sh launch` sets these by default. The caps only tighten the yaml values; they never loosen
them. Override with `TORQUE_CAPS=...` / `SPEED_CAPS=...` (`type=value,...`), or `none`.

| Joint | Stand: torque cap (N m) / speed trip (rad/s) | Stand uses (sim, through pushes) | Policy: torque cap / speed trip | Policy uses (sim, cad_v1 walk + pushes) |
|---|---|---|---|---|
| hip pitch | 15 / 2 | 1.7 / 0.12 | 25 / 4.5 | 16 / 2.8 |
| hip roll | 15 / 2 | 2.9 / 0.54 | 30 / 2.5 | 27 / 0.7 |
| hip yaw | 6 / 2 | 0.6 / 0.74 | 10 / 5 | 8 / 3.0 |
| knee | 20 / 2 | 7.6 / 0.47 | 35 / 8 | 37 / 5.7 |
| ankle | 10 / 2 | 5.8 / 0.55 | 20 / 10 | 29 / 7.3 |

* **Torque cap:** written into each RobStride drive as its torque limit (param 0x700B) when the
  motor enables. The drive enforces it itself, even if the software above it misbehaves.
* **Speed trip:** the bridge disarms all motors (they damp) if any joint goes faster than its
  limit for 50 ms. This is what stops a policy thrashing in the air.
* In sim the stand caps change nothing: push tolerance is identical. cad_v1 with the policy caps
  still finishes the scripted walk, pushes included, without a fall.
* Lower the robot onto its feet slowly. Dropping it can exceed the 2 rad/s stand speed trip,
  which disarms it. That is safe but annoying.

## Finding the balance lean with the D-pad (2026-11-19)

The real robot's weight sits behind its feet in the CAD stand pose, so it tips back onto the rope.
In sim, a COM 8 cm behind the CAD falls back unless leaned; leaning forward 0.05-0.10 rad makes
it stand, and 0.15 rad tips it forward. So the window is a few presses wide.

1. `ros.sh launch stand` (stiffness only, no auto-centring), rope carrying some weight, START.
2. **D-pad up** = lean forward 0.01 rad (moves the COM ~0.7 cm forward); **D-pad down** = back.
   One press at a time: the lean follows at 0.05 rad/s, so each press takes ~0.2 s.
3. Let the rope out a little after each press. The log line `STAND lean ... ankle torque ...`
   (every 2 s) shows the balance: **mean ankle torque ~0** = weight over mid-foot; strongly
   negative = still behind (press up); positive and growing = too far forward (press down).
4. When it stands with the rope slack, note the lean (each press logs it) and put it in the yaml:
   `stand_balance: lean: <value>`. Every STAND then starts there.

## stand-auto-y: auto-lean that waits for Y (2026-11-19, after the floor tests)

**Why:** with `stand-auto` the trim runs from the end of the START ramp. While the robot is still
being lowered on ropes, the ankle torque doesn't show the final balance. So the trim wound up to its
±0.03 limit, then swung to the other limit when the full weight landed. That 0.06 rad swing is
wider than the robot's whole balance window (about −0.02 to −0.065 rad), so it fell while being put down.

`ros.sh launch stand-auto-y foxglove:=false`:

1. Rope taking some weight, feet flat, side by side → **START**. It ramps to the saved lean (−0.050).
   The trim is **off**, and the log says `STAND balance waiting for 'Y pressed'`.
2. Lower it **decisively** (2–3 s) to its full weight. **No D-pad during lowering.**
3. Once it stands on its own weight, press **Y**. The log says `STAND balance ARMED`. From then on
   the trim holds +1.5 N m of ankle torque, correcting at most **±0.015 rad** (half the old limit).
4. The D-pad still works at any time. **B** disarms, and every new STAND waits for Y again.

Overrides: `AUTO_MAX=0.02 AUTO_TARGET=1.0 tools/walker/bringup/ros.sh launch stand-auto-y ...`.
If it misbehaves, fall back to `launch stand` (fixed lean, no trim).

## Squat (X): one scripted balanced squat (2026-11-19, sim-tested, NOT yet tried on the robot)

No policy. `ros.sh launch stand-auto-y` also arms a scripted squat on **X**: the pelvis drops
**2 cm** (min-jerk, 3 s down, 1 s hold, 3 s up), then returns exactly to the stand pose.

* How it stays balanced: per leg, hip/knee/ankle pitch move together along an IK table
  (`th_training/squat_study.py`, MuJoCo, real-foot model, lean -0.055) that keeps the feet flat,
  the torso pitch unchanged and the whole-body COM over the same point relative to the ankles.
  The bridge (`stand_balance.hpp` SquatMotion) carries the same table and timing (C++ vs Python:
  identical to 5e-13 rad).
* Sim (real-foot model: sole ~1.5 cm behind / ~6 cm ahead of the ankle): 2 cm passes every tested
  condition (15 ms delay + kp x0.9, ~1 deg encoder offsets, COM +-1 cm, all combined); with the
  auto-lean trim on (after Y), up to 4 cm. Without the trim, the combined worst case fails at 3 cm.
  With a CAD-size foot (e.g. a heel plate), 6 cm passes everywhere.
  At 2 cm: peak joint speed 0.19 rad/s (trip 2), torque <= 44% of the caps; the mean ankle
  torque stays roughly +0.4 .. +2.3 N m (the foot edge is ~5.7).

Procedure (rope rig, people on the ropes):
1. START, lower decisively onto both feet, let the ropes go slack.
2. **Y** once it carries itself (auto-lean on; the squat is tested with it on).
3. Wait ~5 s until it is still, then **X**. Log: `SQUAT start depth 2.0 cm ...`, then
   `SQUAT done` ~7 s later. X during a squat is ignored; X before the ramp ends is refused.
4. **B** aborts at any time (motors go soft: catch it).

Deeper: `SQUAT_DEPTH=0.03 tools/walker/bringup/ros.sh launch stand-auto-y ...`
(bridge refuses > 0.05 m). Go up 1 cm at a time, and only after a clean run.
