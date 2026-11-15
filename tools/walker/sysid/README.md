# Actuator system ID (chirp tests)

Measures what training currently guesses from datasheets for each RobStride motor: rotor
inertia (`armature`), friction, command delay, top speed at our 36 V bus, and whether the drive
applies the kp/kd we send. One motor at a time, with **nothing attached to the output**.

| File | Runs on | |
|---|---|---|
| `chirp.py` | Jetson | Drives one motor over raw SocketCAN at 500 Hz and logs every command and reply. Standard library only |
| `fit.py` | desktop | Fits the motor model training uses to the logs; writes YAML and plots. numpy, scipy, matplotlib |
| `rsproto.py` | both | Python port of `robstride_motor/can_protocol.hpp` |
| `test_sysid.py` | both | Protocol vectors (same as the C++ tests), every test against a simulated motor, the abort paths, and fit recovery of known parameters |

## Bench setup

- **Clamp the motor housing** to the table. This is direct drive: at a few Nm, an unclamped
  housing spins instead of the output.
- Tape flag on the output face so you can see it move. Nothing else on the output.
- Someone at the power switch.
- **Stop `robot.launch.py` and every `robstride_motor_node`.** They share `can_usb`, and a running
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
`--model`. It prints the plan and asks before moving anything. Logs go to
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

**Preview anywhere, with no CAN:** `python3 tools/walker/sysid/chirp.py --id 14 all --sim`.

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
- **Per-model summary:** mean and min..max across units, plus a ready `MOTORS` line for the mjlab
  config. Use the spread for domain randomization ranges.

**Limits:**
- **Delay:** what's measured is the CAN round trip only, the delay the host sees. The ROS pipeline
  adds its own on the robot.
- **Torque:** true output torque, the torque constant and thermal limits need a locked output with
  a load cell.
- **Isaac Lab friction:** PhysX joint friction is a coefficient, not a torque. Check how the
  actuator config uses it before plugging in `Fc`.

On the simulated motor (J 0.020, b 0.010, Fc 0.15, delay 3 ms), the fit recovers J 0.0200, b 0.0099,
Fc 0.150 and a 3.00 ms delay. The position-chirp prediction is within 0.02 dB and 1° of the measurement.
