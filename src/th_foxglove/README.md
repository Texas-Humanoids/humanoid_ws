# th_foxglove

A live dashboard for the walker in [Foxglove](https://foxglove.dev): a 3D model
that moves with the real joints, the camera, battery and estimated current, robot state,
motor health, and the log. It works the same on the robot and in the MuJoCo sim.

[`layouts/walker_v2.json`](layouts/walker_v2.json) is the ready-made layout:
import it once and it is saved to your Foxglove account.

## How it fits together

```
robot / sim                      th_foxglove (on the Jetson)              your laptop
-----------                      ---------------------------              -----------
motor nodes ──┐                  robot_state_publisher  URDF + joint TFs
policy_bridge ├─ ROS 2 topics ─> robot_monitor          battery, power,   foxglove_bridge ── ws://humanoids:8765 ──> Foxglove app
IMU / th_sim ─┘                                         tilt, /diagnostics
bench camera (uStreamer) ──────> mjpeg_camera           /camera/image/compressed
```

`robot.launch.py` starts all of this by default (`foxglove:=true`), so there is
nothing extra to run on the robot.

## Set up the dashboard (once)

1. **Install Foxglove** on your laptop: download the desktop app from
   <https://foxglove.dev/download> and sign in (a free account is enough). Use the
   desktop app rather than the web app: browsers block the web app from opening a
   plain `ws://` connection to another computer.
2. **Start the robot** (or the sim, see below). On the Jetson, rebuild the image
   once so it has `foxglove_bridge`:
   ```bash
   docker compose --profile robot up -d --build
   docker compose exec robot bash
   colcon build --packages-up-to th_bringup && source install/setup.bash
   ros2 launch th_bringup robot.launch.py \
     motors_config:=src/th_bringup/config/walker_v2_robot.yaml \
     policy_spec:=policies/walker_v2/v6/policy_spec.json \
     model_path:=policies/walker_v2/v6/policy.onnx walk_event:='A pressed'
   ```
   The log shows `foxglove_bridge ... Server listening on port 8765`.
3. **Connect.** In Foxglove: *Open connection* → *Foxglove WebSocket* →
   `ws://humanoids:8765` (or the Jetson's Tailscale IP, as for the bench camera).
4. **Load the layout.** Top bar → layout menu → *Import from file…* → pick
   [`layouts/walker_v2.json`](layouts/walker_v2.json) from this repo. Foxglove
   keeps it in your account, so next time just pick it from the layout menu. If
   you rearrange panels and want to share the result, *Export…* it over this file
   and open a PR.

## What the layout shows

**Overview**: the robot and what it is doing right now.

| Panel | Source | What it tells you |
|---|---|---|
| Robot | `/robot_description`, `/tf` | The WalkerV2 model with every joint at its measured angle. The saved camera starts centered behind the robot and follows `base_smooth` (position and heading, low-passed ~0.8 s), so it glides with the robot instead of jolting each step. In sim the base follows MuJoCo's ground truth; on hardware it tilts with the IMU at a fixed height (no position estimate yet). |
| Command radar | `/dashboard/command` | Top-down and fixed in place; the robot's forward is up. Blue: commanded velocity. Green: actual walking velocity, averaged over two steps so step sway cancels (sim only for now). Orange arc: turn command. Rings at 0.25 and 0.5 m/s. |
| Lean level | `/dashboard/lean` | Top-down bubble level, fixed in place: the dot sits where the body leans (forward = up, left = left), with a 2 s trail. Green under 10°, yellow under 25°, red beyond; the red ring is the 57° fall cutoff. |
| State, Health | `/policy_bridge/mode`, `/robot/health` | WALK / STAND / DISARMED, and OK / WARNING / FAULT for the motors, IMU and battery (details on the System tab) |
| Battery dial | `/dashboard/telemetry` | 30-42 V for the 10S pack, with a numeric voltage readout. NO DATA in sim. |
| Current dials | `/dashboard/telemetry` | Numeric estimates in A: total (30 A BMS) and per leg. Uses `/robot/current_*` calculations. See below. |
| Camera | `/camera/image/compressed` | The bench camera ([tools/bench_camera](../../tools/bench_camera/README.md)) |

**Joints**: one plot per joint (hip pitch, roll, yaw, knee, ankle), each with
the left (blue) and right (orange) leg in degrees, measured as a solid line and
the policy target as a thin light line. Compact legends label each trace. If measured does not follow target, that joint's
gains, calibration or motor are the suspects. Plus the state history.

**System**: battery voltage over the last 5 minutes, estimated current (total
and per leg), knee and ankle torque, separate charts for CPU load (%) and hottest motor temperature (°C), the
Health table (one row per motor with faults, temperature, torque and bus
voltage), and the log (warnings and errors only).

### Current estimate

The motors report torque, speed and supply voltage but not the current they
draw from the battery, and RobStride does not publish the winding resistance
or torque constant needed to compute it. So the dashboard estimates it from
the power the motors deliver: `sum(|torque x speed|) / (voltage x 0.85)`.
Braking counts as draw too. This is not an upper bound on battery current:
it misses holding current at standstill and everything else on the pack; set `battery.base_current` in the robot yaml for the Jetson and
converters once measured. A current sensor on the battery lead would make it a
real measurement. `/battery.current` stays unknown (NaN) until a measurement
is available; the separate `/robot/current_*` topics are estimates.

Each leg estimate requires fresh torque and speed from every motor in that leg.
Missing or invalid feedback produces NO DATA instead of a misleading partial
sum; the total requires both legs. When bus voltage is unavailable, the dials
label the nominal 36 V assumption. The displayed number can exceed the dial
range, with an explicit warning. Dial markers expire after one second if the
monitor stops. Voltage and percentage topics publish NaN when readings expire.
A stopped WebSocket connection should always be checked in Foxglove before a
presentation; panels can retain their last frame when the data source stops.

## Battery

The RobStride motors report their supply voltage (read about once a second per
motor, without slowing the control loop), and the dashboard shows the median.
For a charge estimate, set the pack's series cell count in the robot yaml:

```yaml
battery:
  cells: 10               # the 36 V VATOSO pack is 10S: 42 V full, 30 V empty
  low_cell_voltage: 3.3   # Health turns red below this, per cell (33 V)
```

The percentage comes from a resting 18650 Li-ion voltage curve, so it reads
low while the motors pull hard and recovers at rest. On a bench power supply
it reads whatever that voltage maps to (48 V shows as full). The sim has no battery, so
those panels stay empty there.

## Try it in the sim (no robot needed)

In the CPU container on any laptop (see the
[th_bringup sim section](../th_bringup/README.md#simulated-robot-simtrue)):

```bash
ros2 launch th_bringup robot.launch.py sim:=true \
  motors_config:=src/th_bringup/config/walker_v2_robot.yaml \
  policy_spec:=policies/walker_v2/v6/policy_spec.json \
  model_path:=policies/walker_v2/v6/policy.onnx walk_event:='A pressed'
# second shell: stands, walks forward, checks it stays up
python3 tools/walker/sim_check.py policies/walker_v2/v6/policy_spec.json --vx 0.3
```

Connect Foxglove to `ws://localhost:8765` (Docker Desktop on a Mac: the cpu
service uses host networking; if your Docker does not support that, add
`ports: ["8765:8765"]`). You will see the robot hang on the gantry, stand, and
walk across the grid.

## Recordings

`record:=true` now also records `/joint_states`, `/tf`, `/robot_description`,
`/battery`, `/diagnostics`, `/robot/*`, `/dashboard/*` and `/rosout`. Open the bag folder's
`.mcap` file in Foxglove (*Open local file*) and the same layout replays the run.
The camera is not recorded, to keep bags small.

## Launch arguments

`robot.launch.py`: `foxglove:=false` turns it all off; `camera_url:=''` turns
off the camera, or points it at another MJPEG stream. On its own:

```bash
ros2 launch th_foxglove foxglove.launch.py \
  robot_config:=src/th_bringup/config/walker_v2_robot.yaml \
  camera_url:=http://localhost:8080/stream
```

| Argument | Default | Meaning |
|---|---|---|
| `robot_config` | `''` | Robot yaml: motor list, IMU mounting, battery |
| `urdf` | th_description `walker_v2.urdf` | Model for the 3D view |
| `port` | `8765` | foxglove_bridge port |
| `camera_url` | `''` | MJPEG stream; `''` = no camera |
| `camera_rate` | `10.0` | Max camera frames/s sent (Wi-Fi bandwidth) |

## Troubleshooting

- **Robot parts are rotated 90° and pulled apart.** The 3D panel's *Scene → Mesh
  up-axis* must be **Z-up** (the layout sets it). Foxglove's default, Y-up, turns
  every CAD STL mesh on its side.
- **Robot is grey boxes or missing.** The meshes come through the bridge as
  `package://th_description/...`; make sure `th_description` is built
  (`colcon build --packages-up-to th_bringup` does it).
- **No camera.** Check `http://humanoids:8080/stream` in a browser; if that
  works, the `mjpeg_camera` log says why it can't connect.
- **Laggy over Wi-Fi.** Lower `camera_rate`, or close the Joints tab (each plot
  subscribes to its topics).
- **Battery shows nothing.** On hardware, see whether
  `ros2 topic echo /left_hip_pitch/motor/bus_voltage` prints; in sim it is
  expected.

## Editing the layout

The JSON is generated from `tools/make_layout.py`. From the repository root:

```bash
python3 src/th_foxglove/tools/make_layout.py src/th_foxglove/layouts/walker_v2.json
```

Import the regenerated file in Foxglove. The Overview opens first. The dials
use standard ROS markers, so no Foxglove extension is needed. Rebuild
`th_foxglove` and restart its monitor when updating marker code.

The robot view follows the smoothed heading frame, with a 0.5 m target height,
2.4 m distance, and rear azimuth. If you pan away, hover over that 3D panel and
press **1** to recenter on the followed frame. Reimport the layout to restore
the saved rear angle as well.

Before presenting on hardware, check joint directions against the robot, IMU
lean direction, voltage against a meter, and the actual camera stream. The
simulation cannot verify those hardware measurements. Record a short MCAP run
with `record:=true` for a reproducible presentation backup; camera images are
not included in that recording.

Command radar readouts use compact tuples: blue `cmd (vx, vy, wz)` and green
`actual (vx, vy)`. The first two entries are forward / left velocity in m/s;
`wz` is turn rate in rad/s. Lean reads `tilt° (forward, left)°`.
