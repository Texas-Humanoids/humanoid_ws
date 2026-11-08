# gamepad_node

Converts controller or keyboard input into `cmd_vel` (`geometry_msgs/Twist`) and
`gamepad/button_event` (`std_msgs/String`, one event per press).

```bash
# Install dependencies, build, and source the workspace (ROS 2 Jazzy / Linux).
rosdep install --from-paths src/gamepad_node --ignore-src -y
colcon build --packages-select gamepad_node
source install/setup.bash
ros2 launch gamepad_node gamepad_node.launch.py
# Or use the local keyboard (requires /dev/input read access, usually the input group):
ros2 launch gamepad_node gamepad_node.launch.py input_mode:=keyboard
```

| Motion | Controller (Xbox 360 / X-input) | Keyboard |
|---|---|---|
| Forward/back (`linear.x`) | Left stick Y | W/S |
| Side-to-side (`linear.y`, left positive) | Left stick X | A/D |
| Turn (`angular.z`, left positive) | Right stick X | Left/Right arrows |

Speeds and input settings live in `config/gamepad_node.yaml`; override the file
with `config:=/path/to/file.yaml`. Keyboard speed is multiplied by `keyboard_scaling`.
Keyboard input is global while the node runs. Opposite keys cancel; axes combine.

An Xbox Wireless controller over Bluetooth reports a different layout (LB is
button 6, right stick X is axis 2). Use the matching file:
`ros2 launch gamepad_node gamepad_node.launch.py config:=$(ros2 pkg prefix gamepad_node)/share/gamepad_node/config/gamepad_node_bluetooth.yaml`.
Enable, forward, side-to-side and stop-on-release were checked this way on the
Jetson. Button event names still follow the wired layout (LB is reported as
`BACK`), and the turn sign needs one more check.

In Docker, the container needs the input devices and their group, e.g.
`-v /dev/input:/dev/input --device-cgroup-rule "c 13:* rmw" --group-add $(getent group input | cut -d: -f3)`.
Mounting `/dev/input` (rather than `--device`) keeps working when a Bluetooth
controller reconnects under a new event number.

Hold LB to enable the controller. Releasing LB or losing controller input sends a
stop. Keyboard device loss or queue overflow clears held keys; after overflow,
`EVIOCGKEY` restores current state at the next synchronization boundary. Failed
queries keep motion stopped. SIGINT/SIGTERM send a final stop before ROS closes;
callbacks cannot restart motion afterward. Downstream command timeouts are still
needed for abrupt process or transport loss, including SIGKILL.

Run `colcon test --packages-select gamepad_node` and `colcon test-result --verbose`.
Tests cover mapping, validation, keyboard recovery, the controller watchdog, and
shutdown over DDS, including continued input. Physical axis layout, device access
and USB disconnect behavior need a hardware check.
