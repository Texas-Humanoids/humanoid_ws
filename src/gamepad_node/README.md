# gamepad_node

Publishes `cmd_vel` and `gamepad/button_event` from a controller or a Linux keyboard.

## Run

Build and source the workspace, then start either mode:

```bash
ros2 launch gamepad_node gamepad_node.launch.py
ros2 launch gamepad_node gamepad_node.launch.py input_mode:=keyboard
```

Controller mode starts `joy_node`. Hold LB to enable motion: the left stick controls
forward/back and side-to-side translation, and right stick X controls turning.
Keyboard mode uses W/S for forward/back, A/D for side-to-side, and Left/Right arrows
for turning. It reads a local `/dev/input/event*` device and requires read access
to that device. Keyboard input is global while the node runs.

Speeds, axis indices, keyboard device and timeouts are configured in
`config/gamepad_node.yaml`. Supply another file with `config:=/path/to/file.yaml`.

## Stop behavior

- Releasing LB publishes a zero command. The controller watchdog also sends a
  stop when `joy` messages stop arriving for `joy_timeout` seconds.
- Releasing all movement keys publishes zero velocity. Losing the keyboard device
  clears all held keys and keeps publishing zero velocity.
- An evdev queue overflow clears held keys immediately. Events are discarded
  through the next `SYN_REPORT`, then `EVIOCGKEY` restores the complete current
  key state. If that query fails, movement stays zero until recovery succeeds.
- SIGINT (Ctrl+C), SIGTERM and normal ROS shutdown send a final zero command before
  the ROS context closes. Normal callbacks cannot publish motion after that stop.
  Shutdown waits up to 250 ms for reliable subscribers to acknowledge the command.

A downstream command timeout is still needed for abrupt process or transport loss,
including SIGKILL, where the node cannot send a stop.

## Tests

On Linux with ROS 2 Jazzy and the package dependencies installed:

```bash
colcon build --packages-select gamepad_node
source install/setup.bash
colcon test --packages-select gamepad_node
colcon test-result --verbose
```

The tests cover input mapping and validation, keyboard overflow and recovery,
controller enable/release, button events, the watchdog, invalid startup parameters,
and receipt of the final stop over DDS for SIGINT/SIGTERM, including continued input
during shutdown. They do not require a physical controller or keyboard. Hardware
axis layout, device permissions and USB disconnect behavior need a hardware check.
