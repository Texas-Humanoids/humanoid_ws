# th_sensors

Drivers for the IMU, cameras, and other sensors. Where a vendor ROS 2 driver
already exists we use it and only keep config here.

- [vectornav_imu](vectornav_imu/README.md): VectorNav VN-100 → `/imu/data`.
  We wrote our own small driver because there is no VectorNav release for
  ROS 2 Jazzy.
