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

| Motion | Controller (Xbox, USB or Bluetooth) | Keyboard |
|---|---|---|
| Forward/back (`linear.x`) | Left stick Y | W/S |
| Side-to-side (`linear.y`, left positive) | Left stick X | A/D |
| Turn (`angular.z`, left positive) | Right stick X | Left/Right arrows |

Speeds and input settings live in `config/gamepad_node.yaml`; override the file
with `config:=/path/to/file.yaml`. Keyboard speed is multiplied by `keyboard_scaling`.
Keyboard input is global while the node runs. Opposite keys cancel; axes combine.

Controller mode uses `joy`'s `game_controller_node`, which reports the same
button and axis order for every controller SDL recognizes (SDL game controller
layout: LB = button 9, START = 6, BACK = 4, right stick X = axis 2). Check a new
controller with `ros2 run joy game_controller_node` and `ros2 topic echo /joy`.

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
