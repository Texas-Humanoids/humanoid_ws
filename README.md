# humanoid_ws

Software for the Texas Humanoids robot, a humanoid based on the
[Berkeley Humanoid Lite](https://github.com/HybridRobotics/berkeley-humanoid-lite),
built on **ROS 2 Jazzy**.

This repo is a ROS 2 workspace. See each package's README for its status and usage.

## Repository layout

```
humanoid_ws/
├── src/                    ROS 2 packages
│   ├── th_description/     robot model (URDF, meshes)
│   ├── th_bringup/         launch files and config that start the robot
│   ├── robstride_driver/   talks to the RobStride motors over CAN
│   ├── th_hardware/        connects ROS controllers to the motors
│   ├── th_controllers/     custom controllers (RL walking policy)
│   ├── th_sensors/         IMU, camera, and other sensor drivers
│   └── th_perception/      GPU perception with Isaac ROS (optional)
├── firmware/               microcontroller code for future custom motors
├── training/               mjlab training (Python/uv, not built by colcon)
├── policies/               trained walking policies
└── tools/                  calibration and setup scripts
```

`th_` = Texas Humanoids. It marks packages we wrote, so they're easy to tell
apart from third-party ones.

Each folder has a README describing what will go in it.

## Setup

We use [pixi](https://pixi.sh) to install ROS 2 and every other dependency into
this folder. It runs natively on macOS, Windows, and Linux, so hardware like
motors, IMUs, and cameras plugs straight into your laptop. The steps are the
same on every OS.

1. Install pixi: see [pixi.sh](https://pixi.sh/latest/installation/).
   On Windows, also install [Visual Studio Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/)
   with the **Desktop development with C++** workload, which is needed for C++ packages.
2. Clone this repo and run, from its root:

```bash
pixi run build
```

The first run downloads ROS 2 (a few GB) into `.pixi/`, then builds the
workspace. Then:

```bash
pixi run test     # build, then run every package's tests
pixi shell        # open a shell with ROS and the workspace ready for `ros2 run ...`
```

`pixi run <command>` also works for one-off commands, for example
`pixi run ros2 topic list`. After pulling changes that touch `pixi.toml`, the next
`pixi run` updates the environment automatically.

In VS Code, **Ctrl/Cmd+Shift+B** builds, and **Terminal > Run Task...** lists the other tasks.

ROS traffic stays on your own computer, so you won't see teammates' nodes on
the same Wi-Fi (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` in `pixi.toml`).

### Hardware

Plug devices into your laptop, then:

| Device | macOS | Windows | Linux |
| --- | --- | --- | --- |
| Webcam (`pixi run webcam`, then `pixi run webcam-view`) | yes. Allow camera access for your terminal the first time | yes | yes |
| RobStride USB-CAN module, which shows up as a serial port (`robstride_driver` will use it, not written yet) | yes, as `/dev/cu.wchusbserial*` | yes, as `COM<n>` | yes, as `/dev/ttyUSB<n>`. Run `sudo usermod -aG dialout $USER` once, then log out and back in |
| RealSense | not supported (Intel dropped macOS) | `realsense-viewer` only, ROS driver not yet | yes. Needs Intel's [udev rules](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md) once |

### GPU (Isaac ROS)

Isaac ROS perception only runs in NVIDIA's containers, on Linux with an NVIDIA GPU
(for example the Jetson). For that, and only that, use Docker:

```bash
docker compose build
docker compose up -d
docker compose exec gpu bash
```

Or use VS Code Dev Containers: **Reopen in Container**.
