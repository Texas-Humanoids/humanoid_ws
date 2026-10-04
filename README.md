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

## Docker Setup

Develop everything inside the Docker container, which has all the dependencies installed. There are two images: CPU and GPU. Use the CPU one for almost everything. Use the GPU one only if you have an NVIDIA GPU and need to run Isaac ROS perception nodes.

Docker must be running first. On macOS with Colima, run `colima start`.

### VS Code (recommended)

1. Open this folder in VS Code and install the recommended **Dev Containers** extension when prompted.
2. Run **Dev Containers: Reopen in Container** from the Command Palette and pick **Isaac ROS (CPU)**.

The first time, VS Code builds the image and then the workspace. Once it's done, you're ready:

- **Cmd/Ctrl+Shift+B** builds the workspace.
- **Terminal > Run Task...** lists everything else, including running and testing each package.
- New terminals already have ROS and the workspace sourced.

### Command line

From the repository root (replace `cpu` with `gpu` for the GPU image):

```bash
docker compose --profile cpu up -d --build   # build the image if needed and start the container
docker compose exec cpu bash                 # open a shell inside it (repeat for more shells)
```

Inside the container:

```bash
colcon build
```

Shells you open after that have the workspace sourced. In a shell that was already open, run `source install/setup.bash` once.

When you're done, type `exit` and stop the container with `docker compose --profile cpu down`. Run `colima stop` if you also want to shut down Colima.

To run and test the single-motor simulator, see [the robstride_motor README](src/robstride_motor/README.md).
