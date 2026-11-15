# walker_v2

WalkerV2, the 10-DOF biped (5 joints per leg, no ankle roll, 14.8 kg).

- `urdf/walker_v2.urdf`: the cleaned training URDF (from `WalkerV2_clean.zip`;
  its README lists the fixes to the raw Onshape export and the conventions:
  x forward, pitch + moves the foot back, knee + flexes).
- `mjcf/walker_v2.xml`: MuJoCo model generated from the URDF by
  `scripts/urdf_to_mjcf.py` (do not edit by hand; rerun the script, which
  needs `pip install mujoco`). It adds a freejoint, an `imu` site at
  `imu_link`, the Isaac Lab armature, one torque motor per joint limited to the
  URDF effort, and IMU sensors. Collision boxes touch the floor but not each
  other: they overlap, so self-collision is off, as in training.
- `mjcf/scene.xml`: the robot on a floor at 1 kHz, plus the `gantry` weld that
  th_sim uses to hold the base up.
- `meshes/`: visual meshes, decimated about 2x from the CAD export (bounding
  boxes within 1 mm). Collision uses boxes, not meshes.

Starting pose: base at 0.67 m with the ready pose (hip -0.2, knee 0.4,
ankle -0.2) puts the feet 1 cm above the floor.
