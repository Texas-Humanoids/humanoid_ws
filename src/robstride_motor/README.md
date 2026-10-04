# robstride_motor

A ROS 2 single-motor simulation scaffold. It accepts an absolute target angle in radians on `/motor/target_position` and publishes the simulated angle on `/motor/simulated_position` at about 20 Hz. It does not talk over CAN or control a physical motor.

First, get into the dev container as described in the [root README](../../README.md#docker-setup).

## VS Code

From **Terminal > Run Task...**:

1. **robstride_motor: run simulator** builds the workspace and starts the node. It prints `Simulation only: no physical motor is connected.`
2. **robstride_motor: watch feedback** streams the simulated position.
3. **robstride_motor: send target** asks for a target in radians (default `1.0`). The feedback moves toward it.

To stop a task, click in its terminal and press Ctrl+C. **Test workspace** runs the checks.

## Command line

In one container shell, after `colcon build`:

```bash
ros2 run robstride_motor robstride_motor_node
```

In a second shell (`docker compose exec cpu bash`):

```bash
ros2 topic pub --once /motor/target_position std_msgs/msg/Float64 '{data: 1.0}'
ros2 topic echo /motor/simulated_position
```

To run the checks:

```bash
colcon test --packages-select robstride_motor && colcon test-result --verbose
```

## What to expect

- The feedback starts at `0.0`, moves toward the target, and then holds there. Going from 0 to 1 radian takes about 2.5 seconds, because the position moves at most 0.02 radians every 50 milliseconds.
- Targets are absolute, so sending `1.0` twice does not add another radian. A new target replaces the old one.
- NaN and infinity are ignored with a warning.
- Ctrl+C on the simulator prints `Simulation stopped. No hardware stop command was sent.`
- The checks are lint only and don't test the simulation's behavior.
