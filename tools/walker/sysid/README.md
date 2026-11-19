# Actuator system ID (chirp tests)

Measures what training currently guesses from datasheets for each RobStride motor: rotor
inertia (`armature`), friction, command delay, top speed at our 36 V bus, and whether the drive
applies the kp/kd we send. One motor at a time, with **nothing attached to the output**.

| File | Runs on | |
|---|---|---|
| `chirp.py` | Jetson | Drives one motor over raw SocketCAN at 500 Hz and logs every command and reply. Standard library only |
| `fit.py` | desktop | Fits the motor model training uses to the logs; writes YAML and plots. numpy, scipy, matplotlib |
| `rsproto.py` | both | Python port of `robstride_motor/can_protocol.hpp` |
| `spin.py` | Jetson | One motor at a constant velocity: `spin.py 11 2 +` |
| `mit_all.py` | Jetson | Every motor at once in MIT mode: hold, sine, spin, home |
| `monitor.py` | Jetson | Live table of everything the drives report; never moves anything |
| `test_sysid.py` | both | Protocol vectors (same as the C++ tests), every test against a simulated motor (with a simulated scale for `stall`), the abort paths, and fit recovery of known parameters |

## Bench setup

- **Clamp the motor housing** to the table. This is direct drive: at a few Nm, an unclamped
  housing spins instead of the output.
- Tape flag on the output face so you can see it move. Nothing else on the output.
- Someone at the power switch.
- **Stop `robot.launch.py` and every `robstride_motor_node`.** They share the CAN bus, and a running
  node would fight the test.
- Before the first run on an RS03 or RS02, check the MIT gain scaling with the bench tools
  (`bench/scale_check.py`, then `bench/hold_test.py` at kp 5). The RS03 and RS02 ranges have never run
  on hardware, and a wrong range scales kp and kd silently.

## Run (Jetson, robot container, repo root)

```bash
python3 tools/walker/sysid/chirp.py --id 14 all          # ~2.5 min: torque-chirp, pos-chirp, pos-chirp@50, step, friction
python3 tools/walker/sysid/chirp.py --id 14 top-speed    # separately: spins near top speed
```

The model comes from the ID (11-15 / 21-25: RS03, RS03, RS02, RS03, RS06). For other IDs, pass
`--model`. The CAN interface is found automatically: whichever adapter (`can_left`, `can_right`,
`can_usb`) the motor answers on. Pass `--iface` to choose one. It prints the plan and asks before moving anything. Logs go to
`sysid_logs/<date>_m<id>_<test>/` (`meta.json`, `cmd.csv`, `fb.csv`). Add `--note "..."` to tag a run,
for example with the room temperature.

| Test | Command to the motor | Gives |
|---|---|---|
| `torque-chirp` | kp 0.5 (centering), feed-forward torque sweep 2 → 40 Hz, sized for about ±0.2 rad | **inertia, delay** |
| `pos-chirp` | v6 gains (RS03 60/3, RS02 25/1, RS06 30/1.5), ±0.2 rad sweep 0.2 → 20 Hz | how the motor tracks policy targets; sim-vs-real Bode |
| `pos-chirp --hold-rate 50` | same, targets held at 50 Hz like the policy | the same, with the policy's sample-and-hold |
| `step` | ±0.1 and ±0.3 rad at v6 gains | time-domain check of the fit |
| `friction` | kp 0, kd 1, velocity holds at ±0.05 … 8 rad/s. **Spins.** | Coulomb and viscous friction; kd scaling check |
| `top-speed` | kp 0, ramps to 1.25 × the datasheet speed scaled to 36 V. **Spins fast.** | real top speed at the bus voltage |

**Safety built in:**
- The drive's own torque limit is set for every run: RS03 12, RS02 4, RS06 8 Nm (`--torque-limit`).
- The CAN timeout is set to 0.2 s, so the motor stops itself if the script dies.
- A run aborts on any fault, temperature above 70 °C, 50 ms without a reply, or the output moving
  further than the test should allow.
- Every run ends with 0.5 s of damping and then a stop.
- Spinning tests turn the output back to 0 rad afterwards. `--rewind-only` does just that.
- Exit code 2 means the run aborted; its logs up to the abort are still written.

**If timing is poor:**
- **What to watch:** the run prints the command period (p50 / p99) and the reply ratio.
- **If p99 goes above 5 ms or replies drop below 90%:**
  - Run it as `sudo chrt -f 50 python3 ...`.
  - Or lower `--rate`.
  - Or check the bus with `canbusload can_usb@1000000`.

## Locked-output torque (`stall`, `stall-hold`)

The chirp tests can't tell whether the drive delivers the torque it's told to: with a free output,
a torque error ends up inside the fitted inertia. On the robot, gravity loads are real torques, so it
matters. These two tests measure true output torque with a rod pressing on a scale. **They need the
opposite setup from `all`**, so they are never part of it.

**Rig**
- Housing clamped as usual. A **stiff** rod or bar bolted to the output.
- The rod rests on a scale. Measure L from the **motor axis** to the **contact point** (a bolt head
  or knife edge makes the point well defined). The rod should be level at contact.
- Choose L so the top step fits the scale: reading = torque / (9.81 × L) kg. RS03 at 60 Nm with
  L = 0.15 m is 41 kg, a bathroom scale. RS02 at 12 Nm with L = 0.3 m is 4 kg, a kitchen scale. The
  script prints the expected maximum before it asks to run.
- **Tare the scale with the rod resting on it and the motor off**, so the rod's weight drops out.
- `--dir` is the direction that presses the rod **down**. If it's wrong, the rod lifts and the first
  (small) step aborts.
- Nobody in the rod's swing. Start with the default `--stall-max` (the test cap) before going to peak.

```bash
python3 tools/walker/sysid/chirp.py --id 21 stall --arm 0.15 --dir +                  # 8 steps to 12 Nm and back
python3 tools/walker/sysid/chirp.py --id 21 stall --arm 0.15 --dir + --stall-max 58   # up to peak
python3 tools/walker/sysid/chirp.py --id 21 stall-hold --arm 0.15 --dir + --stall-max 20 --duration 180
```

| Test | Command to the motor | Gives |
|---|---|---|
| `stall` | kp 0, feed-forward torque in `--levels` steps up to `--stall-max` and back down. Each step is held until you **type the scale reading + Enter** (Enter alone skips, `q` stops, `--max-hold` 15 s then moves on) | torque gain k, saturation, friction band, torque constant Kt |
| `stall-hold` | one torque held for `--duration` s; type a reading about every `--prompt-every` s | heating rate, torque drop when warm |

- **What prints at each reading:** commanded torque, scale → true torque and the ratio, the drive's
  own estimate, q-axis current (read at 20 Hz) and temperature. Readings go into `meta.json`
  (`readings`), with the current in `iq.csv`.
- **Units:** `--scale-unit kg|g|lb|N`. If L was mis-measured, fix `profile.arm` in `meta.json`. fit.py
  recomputes from the raw readings.
- **Torque limit:** the drive's limit is set to 1.1 × `--stall-max` (capped at peak), so it never
  clips a step.
- **If the rod slips off:** the output turning faster than 1.5 rad/s, or more than `--stall-travel`
  (0.05 rad) from the start, aborts. The drive then brakes with the v6 kd (RS03 3, RS02 1, RS06 1.5),
  not the gentle idle damping. In simulation, a slip at full RS03 peak stops within 0.08 rad.
- **What to do with the result:** k is the centre of `strength_range` in `walker_v2_cfg.py`, and the
  armature becomes `J_true_torque` (J × k). The peak where the top step saturates is
  `saturation_effort`.

## Spin one motor by hand (`spin.py`)

`spin.py` turns one motor at a constant velocity: you give the ID, the speed (rad/s), the direction, and optionally how many seconds to hold the speed (default 5).

```bash
python3 tools/walker/sysid/spin.py 11 2 +          # motor 11, +2 rad/s, 5 s
python3 tools/walker/sysid/spin.py 14 1.5 - 10     # motor 14, -1.5 rad/s, 10 s
python3 tools/walker/sysid/spin.py                 # interactive: type "11 2 +", "15 4 - 8", ... then q
```

- **Safety:** it's `chirp.py spin` underneath, so it gets the same torque limit, CAN timeout, safety-setting checks and aborts as the other tests.
- **Motion:** it ramps up over 1 s, holds the speed, ramps down, damps, then stops.
- **Control:** velocity only (kp 0, kd 2 / 1 / 2 for RS03 / RS02 / RS06). The output turns freely, so nothing may be attached.
- **After each spin it prints** the speed actually reached and the torque that took, which is the friction at that speed.
- **Speed cap:** just above the no-load speed at 36 V (RS03 16.8, RS02 34.9, RS06 41.5 rad/s).
- **Direction:** `+` is the motor's own increasing angle. A joint's sign on the robot also depends on its `direction` in the robot yaml.
- **Rewind:** the output is left where the spin ended, not turned back to 0. Before a position test, run `chirp.py --id <id> step --rewind-only`.
- **Before running:** stop `robot.launch.py` and every motor node.
- **Extra options** go after the arguments, for example `--ramp 2`, `--kd 1`, `--model RS02`, or `--sim` to preview.

## Drive every motor at once (`mit_all.py`)

One process sends full MIT frames (q_ref, v_ref, kp, kd, τ_ff) to every motor it finds on every adapter, at 200 Hz each. It's the same frame the ROS motor node sends, but with no ROS.

```bash
python3 tools/walker/sysid/mit_all.py home                   # first, after any spins: every output back to 0 rad
python3 tools/walker/sysid/mit_all.py sine                   # +/-0.2 rad at 0.5 Hz, 10 s, v6 gains
python3 tools/walker/sysid/mit_all.py sine --amp 0.3 --freq 1 --seconds 20 --stagger 30
python3 tools/walker/sysid/mit_all.py hold --seconds 30      # stiff at the v6 gains: push on it
python3 tools/walker/sysid/mit_all.py spin --speed 2 --dir - # velocity only, like spin.py
python3 tools/walker/sysid/mit_all.py sine --ids 11-15       # one leg
```

| Motion | kp / kd | What it does |
|---|---|---|
| `hold` | v6 gains (RS03 60/3, RS02 25/1, RS06 30/1.5) | holds the angle each motor starts at |
| `sine` | v6 gains | `--amp` rad at `--freq` Hz around the start angle; 1 s fade in and out. `--stagger` deg makes a wave along the motors |
| `spin` | 0 / 2 (RS02 1) | `--speed` rad/s for `--seconds`; prints each motor's speed and friction |
| `home` | 0 / 1 (RS02 0.5) | turns every output to mechPos 0 at up to 2 rad/s, all at once |

- **Defaults:** every one of 11–15 and 21–25 that answers (the rest are listed as skipped). Pass `--ids` and every listed motor must answer.
- **Before `hold` or `sine`, run `home` after any spins.** The MIT angle wraps at ±4π (±12.6 rad), and spins leave outputs several turns away. The script refuses to start a position motion if any motor's MIT angle doesn't match its mechPos, or is too close to the wrap. That refusal is what tells you to run `home`.
- **Safety:** each motor gets chirp.py's setup (torque limit, CAN timeout, both checked). All 10 are configured before any is enabled. If any one motor has a fault, leaves run mode, goes over 70 °C, misses replies for 50 ms, or moves further than the motion allows, **every** motor is damped and stopped. Ctrl-C does the same.
- **Output:** per motor, the reply ratio plus the tracking error (`hold` / `sine`) or the speed and friction (`spin`). Logs go to `sysid_logs/<date>_mit_all_<motion>/`, where `cmd.csv` and `fb.csv` have an `id` column.
- **Measured 2026-11-15, all 10 at 39 V:** `sine` ±0.2 rad at 0.5 Hz tracks within 7.5–9.1 mrad rms with 100% replies. `home` from 2–3 turns away takes about 10 s.

## See everything the drives report (`monitor.py`)

```bash
python3 tools/walker/sysid/monitor.py             # live table, every motor on every adapter (q quits, p pauses polling)
python3 tools/walker/sysid/monitor.py --once      # one snapshot, then exit (scripts, logs)
python3 tools/walker/sysid/monitor.py --csv m.csv # also log every motor 10x a second
python3 tools/walker/sysid/monitor.py --passive   # send nothing, only listen
```

- **It never moves anything,** so it can run while the robot, `chirp.py` or `mit_all.py` is running. Run it in a second SSH window.
- **Top table:**
  - mode (RUN / reset), faults
  - position, velocity and torque estimate, temperature
  - bus voltage, q-axis current
  - status frames/s, and commands/s from whatever program is driving the motor
- **Bottom table:**
  - run mode, mechPos / mechVel (the true multi-turn angle)
  - torque, current and speed limits; the drive's own position and speed loop gains
  - the last command another program sent (q_ref, kp, kd, τ)
  - the MCU UID
- **Where the values come from:**
  - **Registers** come from parameter reads, about 1000 small frames/s per bus. Running with `mit_all.py` on both buses cost it no replies.
  - **Status values** come from the replies to whatever is commanding the motor.
  - **A motor nobody has commanded for 2 s** gets a stop frame once a second. A stopped drive answers with a status frame, which is how temperature and position appear while idle.
- **New motors** on any ID 1–127 appear within 5 s, so you can plug things in while it runs.
- **Colours:** green = running, red = fault or over 70 °C, yellow = over 55 °C, dim = no reply for 3 s.

**Preview anywhere, with no CAN:** `python3 tools/walker/sysid/chirp.py --id 14 all --sim`. `mit_all.py` and `monitor.py` take `--sim` too.

## Fit (desktop)

```bash
scp -r jetson:~/humanoid_ws/sysid_logs .
python3 tools/walker/sysid/fit.py sysid_logs/*_m14_* --plots plots/ --out m14.yaml
python3 tools/walker/sysid/fit.py sysid_logs/* --out actuators.yaml   # every motor + per-model summary
```

**The fitted model**, the same per-joint actuator model training simulates, with the delay `d` on the commands:

```
tau = clip(kp (q_ref - q) + kd (v_ref - v) + tau_ff, +/- drive limit)
J dv/dt = tau - b v - Fc tanh(v / 0.01)
```

The inertia comes out in units of *commanded* torque. If the drive's torque constant is off,
`J` absorbs it, which is what sim needs, because sim applies the commanded torque.

**What it reports:**
- **friction:** `Fc` and `b`, per direction. Also the ratio of the drive's torque estimate to
  kd·(v_ref − v): a ratio of 1.0 means the kd scaling and the torque estimate agree.
- **torque-chirp:** `J` and `delay`, the delay found by grid search. `Fc` and `b` are taken from the
  friction run when it's included.
- **pos-chirp / step:** a prediction with those values and no tuning (rms error), and a Bode plot of
  sim vs real. **Pass = within 1 dB and 10° up to 10 Hz.**
- **pos-chirp / step refit:** kp and kd scale factors. 1.0 means the drive applies the gains we send.
- **top-speed:** the plateau speed at the measured bus voltage.
- **stall:** the torque gain k (true / commanded torque), whether the top step saturates, the
  friction band, the torque constant Kt, and the drive estimate's error. With a torque-chirp run of
  the same motor it also prints `J_true_torque` = J × k.
- **stall-hold:** heating rate (°C/min over the first minute), minutes to the temperature limit at
  that rate, and how much the true torque dropped as the motor warmed.
- **Per-model summary:** mean and min..max across units, plus a ready `MOTORS` line for the mjlab
  config, and a suggested `strength_range` once stall runs are in. Use the spread for domain
  randomization ranges.

**Limits:**
- **Delay:** what's measured is the CAN round trip only, the delay the host sees. The ROS pipeline
  adds its own on the robot.
- **Torque under motion:** `stall` measures torque at zero speed. The torque-speed line (how torque
  falls with speed at 36 V) would need a second motor as a dynamometer.
- **Isaac Lab friction:** PhysX joint friction is a coefficient, not a torque. Check how the
  actuator config uses it before plugging in `Fc`.

On the simulated motor (J 0.020, b 0.010, Fc 0.15, delay 3 ms), the fit recovers J 0.0200, b 0.0099,
Fc 0.150 and a 3.00 ms delay. The position-chirp prediction is within 0.02 dB and 1° of the measurement.
