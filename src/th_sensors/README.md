# th_sensors

Drivers for the IMU, cameras, and other sensors. Where a vendor ROS 2 driver
already exists we use it and only keep config here.

## RealSense

This package launches the official
[RealSense ROS 2 wrapper](https://github.com/realsenseai/realsense-ros), which uses
[librealsense](https://github.com/realsenseai/librealsense). Both Dockerfiles
install `ros-jazzy-realsense2-camera` and `ros-jazzy-librealsense2` from the ROS
repository. Keep this SDK/driver pair together; a separate SDK installation or
source checkout is unnecessary for the normal build.

### Host setup and Docker

On the Linux host, install the upstream udev rules once so the container's
non-root user can access the camera. The helper requires `curl`, `sudo`, and
`udevadm` and downloads the rules from SDK release `v2.58.4`:

```bash
bash tools/setup_realsense_udev.sh
```

Unplug and reconnect the camera afterward. Use a USB 3 port and cable. Udev
rules must be installed on the host; installing them inside the container does
not change host device permissions.

From the workspace root, rebuild and start a CPU camera container:

```bash
docker compose -f compose.yaml -f compose.realsense.yaml --profile cpu build
docker compose -f compose.yaml -f compose.realsense.yaml --profile cpu up -d
docker compose -f compose.yaml -f compose.realsense.yaml exec cpu bash
```

For GPU, replace `cpu` with `gpu` in these commands. Use the same two `-f` files
when stopping the camera container.

The optional `compose.realsense.yaml` overlay enables privileged hardware access
and mounts host `/dev` and `/sys`. The packaged SDK uses native Linux V4L2 video
devices and HID/IIO IMU controls, including writable sysfs attributes, rather
than USB alone. These mounts also expose devices created when a camera
reconnects. Use this overlay on the robot or a development machine when camera
access is needed; the base Compose configuration remains suitable for ordinary
development without connected hardware.

For VS Code Dev Containers, change `dockerComposeFile` in the chosen CPU or GPU
`.devcontainer/*/devcontainer.json` to:

```json
"dockerComposeFile": ["../../compose.yaml", "../../compose.realsense.yaml"]
```

Then run **Dev Containers: Rebuild Container**. Keep the existing service and
workspace settings.

### Build and launch

Inside the container, from `/ws`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select th_sensors
source install/setup.bash
ros2 launch th_sensors realsense.launch.py
```

Defaults in [config/realsense.yaml](config/realsense.yaml) enable synchronized
640x480 RGB and depth at 30 Hz, depth aligned to color, and the camera's internal
TF tree. Point clouds and IMU streams are disabled by default. Profiles depend
on the camera model; use supported values for your device.

All upstream launch arguments remain available and command-line values override
our defaults:

```bash
# Choose a camera; the underscore prevents ROS from treating the serial as an integer.
ros2 launch th_sensors realsense.launch.py serial_no:=_831612073525

# Add a colored point cloud.
ros2 launch th_sensors realsense.launch.py pointcloud.enable:=true

# Enable a combined IMU stream on an IMU-equipped camera, such as a D435i or D455.
ros2 launch th_sensors realsense.launch.py \
  enable_gyro:=true enable_accel:=true unite_imu_method:=2

# Change profiles or distinguish multiple cameras (also choose each serial_no).
ros2 launch th_sensors realsense.launch.py \
  camera_name:=front rgb_camera.color_profile:=1280,720,30

ros2 launch th_sensors realsense.launch.py --show-args
```

The configuration is a flat mapping of vendor launch defaults, not a ROS
`ros__parameters` file. The upstream `config_file` argument also accepts a flat
mapping; values in that file take precedence over launch arguments for node
parameters. Prefer command-line overrides or editing our defaults for ordinary
use.

### Topics and checks

With the default namespace and camera name, the node is `/camera/camera`:

| Topic | Data |
| --- | --- |
| `/camera/camera/color/image_raw` | RGB image |
| `/camera/camera/color/camera_info` | RGB calibration |
| `/camera/camera/depth/image_rect_raw` | Depth image |
| `/camera/camera/depth/camera_info` | Depth calibration |
| `/camera/camera/aligned_depth_to_color/image_raw` | Depth aligned to RGB |
| `/camera/camera/aligned_depth_to_color/camera_info` | Aligned-depth calibration |
| `/camera/camera/depth/color/points` | Optional point cloud |
| `/camera/camera/imu` | Optional combined IMU |

```bash
ros2 node list
ros2 topic list
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic echo /camera/camera/color/camera_info --once
```

If no camera is found, check the host USB connection, reinstall/reload the udev
rules and reconnect the camera, and confirm the container was recreated with
the hardware overlay. Close RealSense Viewer or any other process using the
device. For unsupported profiles or missing IMU streams, check the camera model
and the driver startup log.

The driver publishes transforms between camera frames. A calibrated transform
from the robot to `camera_link` still belongs in the robot description/bringup
once the camera mounting pose is known.
