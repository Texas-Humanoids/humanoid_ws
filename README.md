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

There are two Dockerfiles in this repo. One is for CPU only, and the other is for GPU. The GPU one is recommended if you have an NVIDIA GPU, and it will allow you to run Isaac ROS perception nodes. The CPU one is for people without an NVIDIA GPU. You should perform as much development as possible on the CPU image, and only use the GPU image when you need to run Isaac ROS nodes.

Make sure to develop everything inside the Docker container, as it will have all the dependencies installed. You can use VSCode Dev Containers to enter the container, which is the recommended way to work with this repo.

Run the Docker commands below from the repository root (`humanoid_ws`). Docker must be running first. On macOS, if you use Colima, start it with `colima start`.

Before creating the container, match its user to your host user so it can write build files. This is especially relevant on macOS, where the default container UID of 1000 usually differs from your own:

```bash
export LOCAL_UID=$(id -u) LOCAL_GID=$(id -g)
```

Run the following commands to build the Docker image and start a container:

```bash
# Build the Docker image (choose either CPU or GPU)
# CPU:
docker compose --profile cpu build  
# GPU:
docker compose --profile gpu build
```
You can then enter the container with VSCode Dev Containers, which is the recommended way to work with this repo.

You can also enter the container from the command line with:

```bash
# Start a container (choose either CPU or GPU, this should only be done if you are not using VSCode Dev Containers)
# CPU:
docker compose --profile cpu up -d
# GPU:
docker compose --profile gpu up -d
```

```bash
docker compose exec cpu bash              # open a shell inside it 
docker compose --profile cpu down         # stop it when done
```

Once inside the container, you can build the workspace with:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The last command makes the built packages available to `ros2 run`. In each new container shell, source both `/opt/ros/jazzy/setup.bash` and `/ws/install/setup.bash` before running ROS commands.

To run and test the single-motor simulator, follow [the robstride_motor README](src/robstride_motor/README.md).

After leaving the container shells with `exit`, stop the container with `docker compose --profile cpu down`. If you started Colima for this session, you can also stop its Linux VM with `colima stop`.
