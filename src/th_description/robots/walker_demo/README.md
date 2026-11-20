# walker_demo

The demo walker: Onshape export `pkg_002` (2026-11-17). It's WalkerV2's 10-DOF layout (5 joints per
leg, no ankle roll) with the battery, Jetson, DC-DC and power distribution in the base. **16.87 kg.**

Everything here is generated. Don't edit it by hand; rerun the builder:
```bash
/c/Users/karti/.venvs/bhl/Scripts/python.exe scripts/build_from_onshape.py <path>/demo_urdf/pkg_002
```

| File | |
|---|---|
| `urdf/walker_demo.urdf` | cleaned URDF: 11 bodies, x forward, box collisions, real effort/velocity |
| `mjcf/walker_demo.xml` | MuJoCo model (same body/joint/geom/site names as walker_v2.xml), measured armature + friction |
| `mjcf/scene.xml` | floor + `gantry` weld (off) |
| `meshes/` | one merged visual mesh per body (full resolution, 28 MB) |
| `build_report.json` | masses per body (raw and symmetrized), boxes, joint signs, warnings |

## What was wrong with the raw export, and what the builder does about it

| # | Problem in `pkg_002.urdf` | Fix |
|---|---|---|
| 1 | **No collision geometry** | one box per body from its mesh bounds, plus a flat 15 x 10.2 x 2 cm sole box under each foot (the sole's lowest points span the full 15 cm length: it's flat) |
| 2 | **Every joint is effort 1 Nm, velocity 1 rad/s** | RobStride peak torque (RS03 60, RS02 17, RS06 36 Nm) and the **measured** 36 V top speed (16.4 / 33.3 / 37.7 rad/s). Also measured armature (0.07 / 0.013 / 0.018) and Coulomb friction (0.40 / 0.12 / 0.18 Nm) from the 11-15 sysid |
| 3 | Robot faces **-y**, origin off to one side (pelvis centre at x = -0.092) | base frame moved to the pelvis centre on the hip-pitch axis, rotated to x forward / y left / z up |
| 4 | **Left and right joint axes are mirrored** (e.g. left knee about -x_raw, right about +x_raw), so the same angle means different motions per side | every axis unified: pitch +y, roll +x, yaw +z. Verified in MuJoCo: hip pitch + = foot back, knee + = flex, roll + = foot +y, ankle + = toes down, identical on both legs |
| 5 | 170 links / 159 fixed joints (one per part) | merged into 11 rigid bodies with exact mass/COM/inertia composition |
| 6 | **Left/right mass asymmetry**: left shank 2.239 kg vs right 2.124 (the left RS06 is 0.745 kg, the right 0.630; datasheet ~0.62), left hip-roll bracket 0.366 vs 0.344 | averaged and mirrored (`SYMMETRIZE`). Weigh the real legs |
| 7 | **No IMU frame** | `imu` site at the pelvis origin. Fine for training (the policy sees angular velocity and gravity direction, which don't depend on where on the rigid base the IMU sits), but **the mounting rotation must be set** in `walker_demo_robot.yaml` |

## Things the export can't tell us (verify on the robot)

- **Joint limits.** The CAD limits are tight and might just be Onshape mate limits:
  - hip roll: **abduction only, 0 adduction** (left [0, 0.785], right [-0.785, 0])
  - knee: **0 to 60 deg** (WalkerV2 was 0 to 120 deg)
  - hip pitch, hip yaw: +-45 deg; ankle: +-90 deg

  With no ankle roll, hip-roll adduction is how the stance leg moves the pelvis over the foot. Zero
  adduction means the lateral balance has to come from body sway alone. That's why there are two
  policies (`cad` and `wide`). Measure the real stops with `tools/walker/bringup/jog.py` before choosing.
- **Mass.** The CAD has no cables, connectors or fasteners. Training randomizes base mass x0.9-1.25,
  but weigh it.
- The base has a lot of mass high up: battery at z = +0.29 m above the hip axis, base COM at +0.125 m.
  Whole-robot COM is 0.556 m above the soles in the ready pose (WalkerV2: 0.568 m).

## Numbers

| | |
|---|---|
| Total mass | 16.869 kg (base 6.94; per leg: hip pitch 1.28, hip roll 0.355, thigh 0.809, shank 2.18, foot 0.338) |
| Hip-pitch axis to sole, legs straight | 0.672 m |
| Ready pose (hip -0.2, knee 0.4, ankle -0.2): base height | 0.6605 m |
| Thigh / shank | 0.320 / 0.260 m (hip pitch to knee / knee to ankle) |
| Hip spacing | 0.290 m (hip-roll axes) |
| Foot | 15 x 10.2 cm flat sole |
