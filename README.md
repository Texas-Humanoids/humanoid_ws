# humanoid_ws

Software for the Texas Humanoids robot, a humanoid based on the
[Berkeley Humanoid Lite](https://github.com/HybridRobotics/berkeley-humanoid-lite),
built on **ROS 2 Jazzy**.

This repo is a ROS 2 workspace. Right now it only has the folder layout;
code will be added in later pull requests.

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

## Docker Setup

There are two Dockerfiles in this repo. One is for CPU only, and the other is for GPU. The GPU one is recommended if you have an NVIDIA GPU, and it will allow you to run Isaac ROS perception nodes. The CPU one is for people without an NVIDIA GPU. You should perform as much development as possible on the CPU image, and only use the GPU image when you need to run Isaac ROS nodes.

Make sure to develop everything inside the Docker container, as it will have all the dependencies installed. You can use VSCode Dev Containers to enter the container, which is the recommended way to work with this repo.

Run the following commands to build the Docker image and start a container:

```bash
# Build the Docker image (choose either CPU or GPU)
# CPU:
docker compose --profile cpu build  
# GPU:
docker compose --profile gpu build
```
You can then enter the container with VSCode Dev Containers, which is the recommended way to work with this repo. 
(We recommend to use command line instead of VSCode Dev Containers)

You can also enter the container from the command line with:

```bash
# Start a container (choose either CPU or GPU, this should only be done if you are not using VSCode Dev Containers)
# CPU:

sudo docker --context default compose --profile cpu up -d

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
```
