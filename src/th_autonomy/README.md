# th_autonomy

Camera-driven following that drives the walking policy the same way the
gamepad does: by publishing `/cmd_vel`. Nothing about the policy or bridge changes.

```
/camera/image/compressed ──► tag_follower or person_follower ──/cmd_vel──► policy_bridge
/policy_bridge/mode ───────►      ▲
/gamepad/button_event ────────────┤
/joy (LB override) ───────────────┘
```

- **tag_follower**: walks to an AprilTag and stops 1 m in front of it.
- **person_follower**: walks to a person and stops 1.5 m in front of them. Raise a
  hand to claim it. Its pose model needs the GPU (see below).

Both share `follower_node.py` (camera in, `/cmd_vel` out, the gamepad rules
below) and `follow_controller.py` (distance and bearing to a speed command), so
they behave the same; only what they look for differs. Run one at a time:

```bash
ros2 launch th_bringup robot.launch.py \
  motors_config:=src/th_bringup/config/walker_v2_robot.yaml \
  policy_spec:=policies/walker_v2/v6/policy_spec.json \
  model_path:=policies/walker_v2/v6/policy.onnx walk_event:='A pressed' \
  follow:=person   # or follow:=tag
```

Then **START** (stand), **A** (walk), **Y** (follow).

| Control | Effect |
|---|---|
| **Y** | Toggle following. Only turns on while the bridge is in WALK. |
| **B** | Disarms the bridge (only that; it is not a follow control). Following turns off with it. |
| **BACK** | Disarms the bridge and turns following off. The person follower also releases whoever claimed it. |
| Hold **LB** | The gamepad drives; following resumes from standstill on release. |
| Bridge leaves WALK | Following off. |

Whenever it stops publishing it sends one zero command. If the node dies, the
bridge's 0.5 s `/cmd_vel` timeout stops the robot. Use the gamepad in
controller mode: in keyboard mode there is no `/joy` for the LB override.

## Topics

Both followers publish here (`status_namespace`), so the dashboard works with
either. The Foxglove **Overview** shows them: detections over the camera, the
target in the 3D view. The **Follow** tab is a backup with plots for tuning.

| Topic | |
|---|---|
| `/follower/state` | `DISABLED`, `OVERRIDE`, `TRACKING`, `LOST` or `SEARCH` (latched) |
| `/follower/target` | `PointStamped`, the followed tag's or person's center in the camera frame (x right, y down, z forward) |
| `/follower/annotations` | `foxglove_msgs/ImageAnnotations` drawn over the camera's own stream: every detection outlined, the followed one in green with distance and bearing, and the state with the stop distance. No re-encoding. |
| `/follower/markers` | `MarkerArray` for the 3D view: the target (a person-sized cylinder or a flat tag) with a line and label, and an arc on the floor at the stop distance |
| `/follower/image/compressed` | Off by default (`publish_debug_image: true`): a re-encoded copy of the camera with the same drawing, for tools other than Foxglove |

The follower also publishes the static frame `follow_camera` on the robot
(`camera.mount_*`: on the chest above the logo, a guess until measured), so the
3D target moves with the robot. Detection runs while following is off too, so
check the Overview in Foxglove before pressing Y.

## tag_follower

Hold up a tag and the robot turns toward it and walks until it is
`target_distance` (1 m) away. Step back and it follows; step closer and it
backs up slowly. A tag far to the side makes it turn in place first.

### Tags

Default family `tag36h11` (the standard). Print any id; `target_id: -1`
follows the nearest. Set `tag_size` to the measured side of the **outer black
square**. A 16 cm tag reads out to about 4 m at 1280x720; tape it to stiff
cardboard so it stays flat. Print one:

```bash
python3 -c "import cv2; from th_autonomy.tag_detector import render_tag; \
cv2.imwrite('tag0.png', cv2.copyMakeBorder(render_tag('tag36h11', 0, 800), 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255))"
```

## person_follower

**Who it follows:**
- **Raise a hand** (wrist above your head) for half a second to claim the robot.
  A phone held to the ear doesn't count.
  The dashboard shows the count-up ("hand up 60%") in magenta, then your box
  turns green. Press **Y** to start.
- **Y with nobody claimed** takes the largest (nearest) person.
- **While it follows someone in view, nobody else can claim it.** If they are
  lost, a raised hand from anyone takes over.
- **BACK** disarms, stops following and releases the claim, so the next raised hand (or Y)
  picks someone new.

It stops 1.5 m away and never backs up from a person.

**Model:** RTMO-l (OpenMMLab, Apache-2.0), a one-stage pose model: one pass
gives every person's box and 17 body joints, so its cost doesn't grow with the
crowd. It runs on the Jetson's GPU with TensorRT in FP16 (`trt_runner.py`):

| Model | GPU, FP16 (Orin Nano) | CPU (ONNX Runtime, 4 threads) |
|---|---|---|
| RTMO-s | 9 ms | 1.3 s |
| RTMO-m | 16 ms | 2.4 s |
| **RTMO-l** (default) | 23 ms | 4.6 s |

The camera relay sends a frame about every 110 ms, so even the largest model
keeps up with room to spare, and the CPU is left to the motors and policy. On
the CPU alone it is far too slow, so the follower needs the GPU image. Fetch the model once from the workspace root; it goes in `models/`,
which git ignores:

```bash
python3 tools/fetch_person_model.py            # rtmo-l (176 MB); --size m or s for the smaller ones
```

The first start builds a TensorRT engine for the GPU and caches it next to the
model (`models/rtmo-l.trt<version>.fp16.engine`), which takes about 10 minutes.

**Running it on the robot:** the motors and policy stay in the `robot`
container; the follower runs beside it in `perception`, which has the GPU:

```bash
docker compose --profile robot up -d --build
docker compose exec robot bash      # robot.launch.py as usual, with follow:=none
docker compose exec perception bash
colcon build --packages-up-to th_autonomy && source install/setup.bash
ros2 launch th_autonomy follow.launch.py target:=person
```

On the demo walker, where the dashboard runs on its own (`ros.sh dashboard`), one
command does this: `tools/walker/bringup/ros.sh follow` (`person` by default, or
`tag`; `stop`, `status`, `log`). It builds `th_autonomy` if needed and runs the
follower in a `humanoid-ros:gpu` container next to the dashboard.

Both containers share the host network, so they see each other's topics. The
follower runs only on CPU cores 4 and 5 (`cpu_cores`), so it can't delay the
control loop on the others.

**Several people in view:** each person gets a track id that stays with them
(ByteTrack-style matching with motion prediction; doubtful detections, such as
a person half hidden, keep a track alive but never start one). If the followed
person disappears, only a new track with matching clothing takes them back, so
it won't switch to a bystander. After `track.forget_time` (4 s) it gives up and
waits for a raised hand (or Y again).

**Distance** comes from the torso: shoulder midpoint to hip midpoint is about
0.5 m (`track.torso_length`) and barely changes as people turn or swing their
arms. Without a visible torso (too close, cut off) it falls back to the box:
the smaller of `fy * 1.7 m / box height` and `fx * 0.45 m / box width`.

## Camera

It reads the MJPEG relay th_foxglove already runs (`camera_url`, the Arducam
bench camera at 1280x720). The camera must face forward and roughly level.
If it is mounted sideways or upside down, turn it once, in the `mjpeg_camera`
relay (`camera_rotation:=` on foxglove.launch.py or follow.launch.py; default
180 for the walker), so the dashboard and the follower both get upright frames.
Leave the follower's `camera.rotation` at 0 then, or the image is turned twice.
Calibrated intrinsics are for the turned image.

- **Bearing** (which way to turn) is nearly independent of calibration. The
  Arducam has a wide fisheye lens, so with the 90° pinhole guess the bearing
  of a target far off to the side reads smaller than it is (left and right are
  still right).
- **Distance** is proportional to the focal length. Without calibration it
  is guessed from `camera.hfov_deg` (90°). Check it against a tape measure:
  if `/follower/target` z reads 1.2 m at a true 1.0 m, scale `hfov_deg` until it
  matches, or calibrate (`camera.fx/fy/cx/cy/distortion`, at the stream's
  full resolution) with `ros2 run camera_calibration cameracalibrator`.
- **Motion blur** from the gait is the main risk. Detections are smoothed and
  a dropout under `hold_time` is bridged. If the target still drops out
  while walking, lower the camera's exposure in `/etc/default/bench-camera`.

## Tuning (`config/<target>_follower.yaml`, `follow.*`)

Speeds stay inside the policy's training ranges: forward ≤ 0.3 m/s
(trained to 0.5), backing ≤ 0.1 m/s (tags only), yaw ≤ 0.4 rad/s (trained to 0.5), with
acceleration limits so it never sees a step command. Raise `max_forward` once
it walks reliably. `search_yaw_rate` > 0 makes it turn toward where it last
saw a lost target (try 0.25).

## Test

```bash
colcon build --packages-select th_autonomy
colcon test --packages-select th_autonomy
colcon test-result --verbose
```

Tests cover the controller (deadbands, turn-in-place, clamps, ramping,
dropouts, search), detection and pose on rendered tags, the tag node over DDS
(arming rules, Y/B/LB, the stop on disable, follows a rendered tag), pose
model input and output, distance from the torso and the box, raised-hand
detection, track ids through motion, and who is followed (largest on Y, claims
after `claim_time`, no claims while following someone in view, won back by
clothing but not by a stranger, forgotten after `forget_time`). The real model
runs when `models/rtmo-l.onnx` is present, and the person node only with
TensorRT (the GPU image); they are skipped otherwise. Not yet checked on hardware: blur while walking, real distance
accuracy, and the closed loop on the robot.
