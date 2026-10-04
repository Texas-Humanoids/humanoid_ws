# robstride_motor

A ROS 2 single-motor simulation scaffold. It accepts an absolute target angle in radians and publishes a simulated angle at approximately 20 Hz. It does not communicate over CAN or control a physical motor, and has no graphical interface.

## Build and start

Use the CPU Docker workflow in [the root README](../../README.md). Docker must be running first (`colima start` if you use Colima on macOS). Run these commands from the repository root on your host:

```bash
export LOCAL_UID=$(id -u) LOCAL_GID=$(id -g)
docker compose --profile cpu build
docker compose --profile cpu up -d
docker compose exec cpu bash
```

In the container shell:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run robstride_motor robstride_motor_node
```

Leave this terminal running. The node starts at zero and prints `Simulation only: no physical motor is connected.`

## Send a command and watch feedback

Open a second host terminal in the repository root and enter the same container:

```bash
docker compose exec cpu bash
```

In this second container shell:

```bash
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

ros2 topic pub --once /motor/target_position std_msgs/msg/Float64 '{data: 1.0}'
ros2 topic echo /motor/simulated_position
```

The first terminal logs the target. Feedback approaches `1.0` radians, then keeps publishing that value. Moving from zero to one radian takes approximately 2.5 seconds: the simulation moves at most 0.02 radians every 50 milliseconds.

Press Ctrl+C in the second terminal to stop the feedback display. The simulator in the first terminal keeps running. Send another target and watch again:

```bash
ros2 topic pub --once /motor/target_position std_msgs/msg/Float64 '{data: -0.5}'
ros2 topic echo /motor/simulated_position
```

Targets are absolute positions: sending `1.0` twice does not add another radian. A new target replaces the old one. NaN and infinity are rejected.

## Checks

In a container shell with ROS sourced:

```bash
colcon test --packages-select robstride_motor
colcon test-result --verbose
```

The configured checks cover linting, not simulation behavior.

## Stop

Press Ctrl+C in the first terminal to stop the simulator. It should print `Simulation stopped. No hardware stop command was sent.` Press Ctrl+C in the second terminal if the feedback display is running, then type `exit` in both container shells.

On the host, from the repository root:

```bash
docker compose --profile cpu down
```

If you started Colima for this session, also run `colima stop` to shut down its Linux VM.
