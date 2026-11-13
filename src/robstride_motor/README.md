# robstride_motor

ROS 2 node for driving a single RobStride RS06 or RS03 actuator over real CAN (SocketCAN), using the motor's CSP (Cyclic Synchronous Position) mode. This talks to real hardware — it is not a simulation.

## Hardware setup

### Wiring — you need three wires, not two

A CAN connection needs `CAN_H` and `CAN_L` (the differential signal pair), **plus a common ground (`GND`) between the adapter and the motor.** The adapter and motor are powered from completely separate sources, so without a shared ground the signaling degrades under load — this exact issue cost us a full debugging session (symptom: commands appeared to send fine, but the motor never replied, and the CAN interface degraded from `ERROR-ACTIVE` to `ERROR-PASSIVE` the moment real traffic was sent). If you only see two wires connected to your adapter or motor connector, you are very likely missing the ground.

### Power

**CAN wiring does not power the motor.** The RS06 needs a separate power connection (its own supply, correct voltage/current for the motor — check the motor's own spec, not the CAN adapter or your computer's USB port) wired and switched on independently of the CAN connection. An unpowered motor will never acknowledge CAN frames no matter how correct the wiring or code is.

### CAN adapter

We used a `gs_usb`/CANable-compatible USB-CAN adapter (shows up in Linux as `can0` via the kernel's `gs_usb` driver). Bitrate **must be 1,000,000 (1 Mbps)** — this is the RS06's expected bus speed; a mismatched bitrate means frames go out but are never recognized as valid by the motor.

## Software setup

### Bring up the CAN interface (host machine)

```bash
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```
You're looking for `state ERROR-ACTIVE` and `bitrate 1000000`. If `can0` doesn't exist at all, the adapter isn't being recognized — see the WSL2 section below if that applies to you.

### If you're on WSL2 (Windows)

Two gotchas we hit repeatedly:

1. **Docker Desktop's engine cannot see host CAN hardware at all**, even with `--network host`. Docker Desktop's engine runs inside its own isolated WSL distro, completely separate from the one where `can0` lives. The fix: install Docker Engine (`docker-ce`, not Docker Desktop) directly inside your main Ubuntu WSL distro, via `curl -fsSL https://get.docker.com | sudo sh`. This makes `--network host` share the real network namespace where `can0` actually exists.

2. **The USB-CAN adapter's attachment to WSL2 does not persist.** It can silently drop (after a sleep/wake cycle, a reconnect, etc.), and when it does, `can0` disappears from `ip link show` entirely with no error — it just isn't there. Fix, from a **Windows PowerShell window** (not WSL):
   ```powershell
   usbipd list
   usbipd attach --wsl --busid <busid shown for your adapter>
   ```
   Then redo the `ip link set can0 up ...` step above — the interface comes back as a fresh, unconfigured device each time it's reattached.

### Find the motor ID (if unknown or unresponsive)

If the node logs `Could not read a finite starting position; refusing to enable`, the motor may not actually be at the ID you're assuming — ours turned out to be configured to `1`, not the `127` we initially assumed from the actuator team.

Requires `can-utils` (`candump`/`cansend`) and the node **not** running (it shares `can0`).

The quick way is the bundled script. It is read-only (it only sends the Get Device ID request, never enables or configures anything) and prints every motor that answers:
```bash
src/robstride_motor/scripts/find_motor_id.sh can0
```
Then pass the ID you want explicitly as `-p motor_id:=<id>`. The node deliberately never auto-selects an ID, so it can't silently command the wrong motor.

To do it by hand, or see what the script does, watch for replies in one terminal:
```bash
candump can0
```

Ping every possible ID (0–127) with a "Get Device ID" request (comm type `0`) in another:
```bash
for id in $(seq 0 127); do
  printf -v hexid '%08X' $((0xFF00 | id))
  cansend can0 ${hexid}#0000000000000000
  sleep 0.02
done
```

A live motor replies with something like:
```
can0  000001FE   [8]  6E 45 3B 4E 20 71 30 0B
```
— comm type `0`, the responding motor's own ID embedded in the middle bits (`01` here → ID `1`), destination `0xFE`, and a non-zero UUID payload. Ignore any reply with an all-zero payload — that's a bogus/stale response, not a real motor. Whatever ID actually answers is the one to pass as `-p motor_id:=...` when running the node.

### Build

Inside your ROS 2 Jazzy environment (we build inside a Docker container with `--network host` so it shares the host's `can0`):
```bash
colcon build --packages-select robstride_motor
source install/setup.bash
```

### Run

```bash
ros2 run robstride_motor robstride_motor_node --ros-args \
  -p motor_id:=127 \
  -p can_interface:=can0 \
  -p motor_model:=RS06
```

`motor_id` must be in **0–127** and has **no default** — it must be supplied, or the node will throw on startup rather than silently commanding the wrong device. `can_interface` defaults to `can0`; `motor_model` defaults to `RS06`. Supported models are looked up from a table in the node: `RS06` (verified on hardware) and `RS03` (same protocol and ±4π position range per the RobStride manuals, not yet run on hardware). Anything else makes the node refuse to start.

| Parameter | Default | Meaning |
|---|---|---|
| `speed_limit` | `1.0` | CSP speed limit written at startup, rad/s |
| `current_limit` | `3.0` | Current limit written at startup, A |
| `feedback_rate` | `50.0` | Position/velocity polling rate, Hz |
| `can_timeout` | `0.5` | Motor-side watchdog, s (`0` disables, max 10) |

## Topics

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `motor/target_position` | `std_msgs/Float64` | Subscribe | Radians. Clamped to ±4π before being sent to the motor. |
| `motor/actual_position` | `std_msgs/Float64` | Publish | Radians, read from `mechPos` (`0x7019`) at `feedback_rate`. |
| `motor/actual_velocity` | `std_msgs/Float64` | Publish | Rad/s, read from `mechVel` (`0x701B`) right after the position. |
| `motor/actual_torque` | `std_msgs/Float64` | Publish | Nm, from the motor's status frames. |
| `motor/temperature` | `std_msgs/Float64` | Publish | °C, from the motor's status frames. |
| `motor/status` | `diagnostic_msgs/DiagnosticStatus` | Publish | Level OK (running), WARN (not running) or ERROR (faults), with mode, faults, the last fault report, temperature and torque as values. |
| `motor/clear_faults` | `std_srvs/Trigger` | Service | Clears faults and re-enables at the current position (the same safe startup sequence). |

Each poll first re-sends the current target (the watchdog keepalive), then reads position and velocity with a 5 ms reply timeout, so a silent motor skips a sample instead of stalling the node. Position and velocity are only published when both reads succeed. Torque, temperature and status are published each poll from the latest status frame, and stop (with a warning) if no status frame arrived in 200 ms.

```bash
ros2 topic echo /motor/status
ros2 service call /motor/clear_faults std_srvs/srv/Trigger
```

### Status, faults and the watchdog

At startup the node turns on the motor's **active reporting** (comm type `0x18`), so the motor sends a status frame (comm type `2`/`0x18`) every 10 ms. Each one carries:

- **mode**: reset (disabled), calibrating or running
- **fault flags**: undervoltage, overcurrent, overtemperature, magnetic encoder fault, stall, uncalibrated
- **position, velocity, torque**: 16-bit values over the model's range (RS03: ±20 rad/s, ±60 Nm; RS06: ±50 rad/s, ±36 Nm). Only torque is published from here; position and velocity come from the float registers above.
- **temperature**, in 0.1 °C steps

The node also decodes **fault reports** (comm type `0x15`: driver fault, overvoltage, per-phase overcurrent, stall overload, position-init and hardware-ID faults, and the overtemperature warning). Changes in mode or faults are logged once.

The **CAN watchdog** (`canTimeout`, parameter `0x7028`, 20000 counts per second) makes the motor stop itself if it hears nothing for `can_timeout`. That covers what the node cannot: a crash, `SIGKILL`, the computer losing power, or a pulled CAN adapter. While the node runs, its 50 Hz keepalive holds the motor at its last target even if nothing upstream publishes. After a fault or a watchdog stop, the motor is in reset mode and ignores targets; call `motor/clear_faults` to recover. That service runs the startup sequence again, so the motor re-enables where it is instead of jumping.

Example:
```bash
ros2 topic pub --once /motor/target_position std_msgs/msg/Float64 "{data: 0.76}"
ros2 topic echo /motor/actual_position
```

## How this actually gets the motor moving

This is the part that took the most debugging to get right, so it's worth writing down explicitly rather than just leaving it in the code.

RobStride's protocol packs a 29-bit extended CAN ID as: 5-bit comm type (bits 28–24), a 16-bit middle field (bits 23–8, meaning depends on comm type), and the target motor's ID (bits 7–0). Our host ID is `0xFF`.

**Startup sequence** (all verified byte-for-byte against real `candump` captures):
1. **Disable** the motor and **clear latched faults** (comm type `4`, data byte 0 = `1`) — clean slate before reconfiguring.
2. **Write `run_mode = 5`** (parameter `0x7005`, comm type `0x12`) — this switches the motor into CSP (Cyclic Synchronous Position) mode.
3. **Write speed limit** (parameter `0x7017`) and **current limit** (parameter `0x7018`) as floats — `speed_limit` and `current_limit`, conservative by default (1.0 rad/s, 3.0 A) for safety on a fresh bring-up.
4. **Write the CAN timeout** (parameter `0x7028`, `uint32`) and **turn on active reporting** (comm type `0x18`).
5. **Read the motor's actual current position** (parameter `0x7019`, "mechPos", comm type `0x11` request / response).
6. **Write that same value back as the initial position target** (parameter `0x7016`, "loc_ref"). This step is critical — skip it, and the motor can jump violently toward whatever stale target it had stored from a previous session the instant it's enabled.
7. **Enable** the motor (comm type `3`).

Steps 1–3 and the original position handling were verified against `candump` captures. The fault clearing, CAN timeout, active reporting and status/fault decoding in steps 1 and 4 follow the RobStride manuals as implemented in other open-source drivers ([motorbridge](https://github.com/motorbridge/motorbridge)), and have not been run on our motors yet.

If a startup CAN write fails or the starting position cannot be read as a finite value, the node attempts a stop, closes the socket, and exits with an error. Successful socket writes only confirm local transmission acceptance; they do not acknowledge that the motor applied the configuration. Normal shutdown and exception unwinding also turn off active reporting and attempt a stop before closing the socket.

From then on, every new value published to `motor/target_position` is just written directly to `loc_ref` — the motor's own internal position-control loop (with factory-tuned gains we never touch) handles actually getting there smoothly. We do **not** send raw torque/Kp/Kd values ourselves (that's a different scheme, "MIT mode," which we tried first and abandoned — CSP mode is what RobStride's own verified working scripts use, and switching to it is what fixed the motor only weakly rumbling instead of actually tracking commanded positions).

## Multi-motor setups

This node drives exactly one motor per instance. For multiple motors, run one instance per motor with a distinct `motor_id`, each in its own ROS namespace so topics don't collide:
```bash
ros2 run robstride_motor robstride_motor_node --ros-args -r __ns:=/left_arm -p motor_id:=1 -p can_interface:=can0
ros2 run robstride_motor robstride_motor_node --ros-args -r __ns:=/right_arm -p motor_id:=2 -p can_interface:=can0
```
Verified on hardware with two RS06 motors (IDs 1 and 2) on one `can0`: each node reported only its own motor's position, and a command to one moved only that motor.

A **namespace** is a prefix ROS 2 adds to a node's topic names, like a folder path. The node's topics are relative (`motor/target_position`, `motor/actual_position`), so `-r __ns:=/left_arm` turns them into:

| Namespace | Command topic | Feedback topic |
|---|---|---|
| none | `/motor/target_position` | `/motor/actual_position` |
| `/left_arm` | `/left_arm/motor/target_position` | `/left_arm/motor/actual_position` |
| `/right_arm` | `/right_arm/motor/target_position` | `/right_arm/motor/actual_position` |

Without namespaces, two nodes would share the same topics and each would receive the other's commands. Publish to the namespaced topic, e.g. `ros2 topic pub --once /right_arm/motor/target_position std_msgs/msg/Float64 "{data: 0.3}"`.

Multiple nodes can safely share the same physical `can0` socket — SocketCAN isn't exclusive, and every node filters replies by checking the responding motor's own ID before trusting them. Whether multiple motors on one limb/bus should eventually be coordinated by a single combined node instead (for synchronized multi-joint motion) is an open design question, not yet decided.

## Testing

```bash
colcon test --packages-select robstride_motor
colcon test-result --verbose
```
Unit tests cover CAN ID packing and parameter encode/decode against real captured byte values, plus position clamping — see `test/test_robstride_motor.cpp`. Protocol tests also cover the CAN timeout, fault-clearing and active-report frames, and decoding status frames and fault reports. Node safety tests exercise invalid IDs and parameters, every startup write failure, non-finite starting positions, the startup and shutdown frame sequence, status and fault frames arriving during a read, and cleanup during exceptions using a fake CAN transport. The discovery script is also tested with mocked CAN utilities and the system default awk.

## Troubleshooting

- **`Could not open CAN socket on 'can0'`** — the interface doesn't exist or isn't visible to this environment. Check `ip link show` on the host, and `ls /sys/class/net/` inside the container.
- **`Could not read a finite starting position; refusing to enable`** — the socket opened, but nothing answered. First confirm the motor is actually at the ID you're using — see [Find the motor ID](#find-the-motor-id-if-unknown-or-unresponsive); this was the root cause the one time we hit it (assumed ID `127`, motor was really at `1`). If the ID is right, check: is the motor actually powered? Is the ground wire connected? Run `candump can0` in parallel to see whether request frames are going out with no reply at all (the motor isn't there, or is at a different ID — see [Find the motor ID](#find-the-motor-id-if-unknown-or-unresponsive)) versus going out and visibly tripping transmit errors (`ip -details link show can0` — look for `ERROR-PASSIVE` or `bus-off`).
- **`CAN write failed`, repeating** — the CAN controller has likely gone bus-off/error-passive from unacknowledged transmits (nothing on the bus is responding). Reset with `sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 1000000`, but this won't stay fixed unless the underlying wiring/power issue is also resolved.

## Known limitations

- Single motor per node; no coordinated multi-joint motion yet.
- Only RS06 and RS03 are in the `motor_model` table (`kMotorLimitsByModel`); RS03 has not been run on hardware yet.
- Status decoding, fault reports, the CAN timeout and active reporting are untested on hardware. Check `motor/status` and `candump can0` on first bring-up; if no status frames arrive, the node warns and keeps running on position/velocity alone.
