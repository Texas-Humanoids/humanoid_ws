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
│   ├── robstride_motor/    single-motor ROS 2 simulation scaffold
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

Everything runs inside a Docker container that has ROS 2 and all the dependencies installed. The steps are the same on Windows, macOS, and Linux.

1. Install [Docker Desktop](https://www.docker.com/products/docker-desktop/) and make sure it's running. On Linux you can use [Docker Engine](https://docs.docker.com/engine/install/) instead.
2. Install [VS Code](https://code.visualstudio.com/) and its [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.
3. Clone this repo and open the folder in VS Code.
4. Run **Dev Containers: Reopen in Container** from the Command Palette (Ctrl/Cmd+Shift+P) and pick **humanoid_ws (CPU)**.

The first time, VS Code builds the image and then the workspace, which takes a few minutes. After that:

- **Ctrl/Cmd+Shift+B** builds the workspace.
- **Terminal > Run Task...** lists everything else, including running and testing each package.
- New terminals already have ROS and the workspace sourced.

Use **humanoid_ws (GPU, Isaac ROS)** only if you have an NVIDIA GPU (Linux or Windows) and need Isaac ROS perception nodes.

On Windows, clone the repo from a WSL terminal (into your Linux home folder) instead of onto `C:`. Builds are much faster there.

### Without VS Code

The same commands work in any terminal, from the repository root:

```bash
docker compose --profile cpu up -d --build   # build the image if needed and start the container
docker compose exec cpu bash                 # open a shell inside it (repeat for more shells)
colcon build                                 # inside the container
docker compose --profile cpu down            # on the host, when you're done
```

Replace `cpu` with `gpu` for the GPU image. On Linux, if your user ID isn't 1000 (check with `id -u`), use VS Code instead: it matches the container user to yours, which this path doesn't.

To run and test the single-motor simulator, see [the robstride_motor README](src/robstride_motor/README.md).
