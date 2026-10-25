# robstride_motor

ROS 2 node for driving a single RobStride RS06 actuator over real CAN (SocketCAN), using the motor's CSP (Cyclic Synchronous Position) mode. This talks to real hardware — it is not a simulation.

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

If the node logs `Could not read starting position from motor X; refusing to enable`, the motor may not actually be at the ID you're assuming — ours turned out to be configured to `1`, not the `127` we initially assumed from the actuator team.

Requires `can-utils` (`candump`/`cansend`) and the node **not** running (it shares `can0`).

Watch for replies in one terminal:
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

`motor_id` has **no default** — it must be supplied, or the node will throw on startup rather than silently commanding the wrong device. `can_interface` defaults to `can0`; `motor_model` defaults to `RS06`. Supported models are looked up from a table in the node — currently only `RS06` has a verified entry, so anything else causes the node to refuse to start at launch.

## Topics

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `motor/target_position` | `std_msgs/Float64` | Subscribe | Radians. Clamped to ±4π before being sent to the motor. |
| `motor/actual_position` | `std_msgs/Float64` | Publish | Real motor feedback, polled over CAN at 10 Hz — not simulated. |

Example:
```bash
ros2 topic pub --once /motor/target_position std_msgs/msg/Float64 "{data: 0.76}"
ros2 topic echo /motor/actual_position
```

## How this actually gets the motor moving

This is the part that took the most debugging to get right, so it's worth writing down explicitly rather than just leaving it in the code.

RobStride's protocol packs a 29-bit extended CAN ID as: 5-bit comm type (bits 28–24), a 16-bit middle field (bits 23–8, meaning depends on comm type), and the target motor's ID (bits 7–0). Our host ID is `0xFF`.

**Startup sequence** (all verified byte-for-byte against real `candump` captures):
1. **Disable** the motor (comm type `4`) — clean slate before reconfiguring.
2. **Write `run_mode = 5`** (parameter `0x7005`, comm type `0x12`) — this switches the motor into CSP (Cyclic Synchronous Position) mode.
3. **Write speed limit** (parameter `0x7017`) and **current limit** (parameter `0x7018`) as floats — conservative values (1.0 rad/s, 3.0 A) for safety on a fresh bring-up.
4. **Read the motor's actual current position** (parameter `0x7019`, "mechPos", comm type `0x11` request / response).
5. **Write that same value back as the initial position target** (parameter `0x7016`, "loc_ref"). This step is critical — skip it, and the motor can jump violently toward whatever stale target it had stored from a previous session the instant it's enabled.
6. **Enable** the motor (comm type `3`).

From then on, every new value published to `motor/target_position` is just written directly to `loc_ref` — the motor's own internal position-control loop (with factory-tuned gains we never touch) handles actually getting there smoothly. We do **not** send raw torque/Kp/Kd values ourselves (that's a different scheme, "MIT mode," which we tried first and abandoned — CSP mode is what RobStride's own verified working scripts use, and switching to it is what fixed the motor only weakly rumbling instead of actually tracking commanded positions).

## Multi-motor setups (not yet built, but straightforward)

This node drives exactly one motor per instance. For multiple motors, run one instance per motor with a distinct `motor_id`, each in its own ROS namespace so topics don't collide:
```bash
ros2 run robstride_motor robstride_motor_node --ros-args -r __ns:=/left_arm -p motor_id:=127 -p can_interface:=can0
ros2 run robstride_motor robstride_motor_node --ros-args -r __ns:=/right_arm -p motor_id:=5 -p can_interface:=can0
```
Multiple nodes can safely share the same physical `can0` socket — SocketCAN isn't exclusive, and every node filters replies by checking the responding motor's own ID before trusting them. Whether multiple motors on one limb/bus should eventually be coordinated by a single combined node instead (for synchronized multi-joint motion) is an open design question, not yet decided.

## Testing

```bash
colcon test --packages-select robstride_motor
colcon test-result --verbose
```
Unit tests cover CAN ID packing and parameter encode/decode against real captured byte values, plus position clamping — see `test/test_robstride_motor.cpp`.

## Troubleshooting

- **`Could not open CAN socket on 'can0'`** — the interface doesn't exist or isn't visible to this environment. Check `ip link show` on the host, and `ls /sys/class/net/` inside the container.
- **`Could not read starting position from motor X; refusing to enable`** — the socket opened, but nothing answered. Check: is the motor actually powered? Is the ground wire connected? Run `candump can0` in parallel to see whether request frames are going out with no reply at all (the motor isn't there, or is at a different ID — see [Find the motor ID](#find-the-motor-id-if-unknown-or-unresponsive)) versus going out and visibly tripping transmit errors (`ip -details link show can0` — look for `ERROR-PASSIVE` or `bus-off`).
- **`CAN write failed`, repeating** — the CAN controller has likely gone bus-off/error-passive from unacknowledged transmits (nothing on the bus is responding). Reset with `sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 1000000`, but this won't stay fixed unless the underlying wiring/power issue is also resolved.

## Known limitations

- Single motor per node; no coordinated multi-joint motion yet.
- Only RS06 is supported as a `motor_model` today — the lookup is table-driven (`kMotorLimitsByModel` in the node), so adding a verified second model is a one-line addition rather than a restructure. We don't yet have verified position/speed/current ranges for any other RobStride model, which is the actual blocker, not the code structure.
- No CAN watchdog or fault-code handling — a motor-side fault isn't currently surfaced back to ROS.