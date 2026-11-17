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
- **person_follower**: walks to a person and stops 1.5 m in front of them.

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
| **B** or **BACK** | Following off (BACK also disarms the bridge). |
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

Stand in front of the robot and press **Y**: it locks on to the largest
(nearest) person in view and follows them, stopping 1.5 m away. It never backs
up from a person.

**Setup:** the detector is YOLOX-Nano (Apache-2.0, 3.7 MB), run on the CPU
with ONNX Runtime (already in both Docker images). Fetch it once from the
workspace root; it goes in `models/`, which git ignores:

```bash
python3 tools/fetch_person_model.py
```

**Speed:** about 130 ms per frame on the Orin Nano's CPU, so about 7 Hz. Detection
runs in its own thread, so the 20 Hz command keeps its rate; between frames the
controller holds the last target for `follow.hold_time` (0.6 s).

**Several people in view:** the lock follows the same box from frame to frame
(overlap) and remembers the colors of the person's shirt. If they disappear
(behind someone, out of frame), only a shirt-color match takes them back, so it
won't switch to a bystander. After `track.forget_time` (4 s) it gives up and
locks on to the largest person. Press Y twice to re-lock on whoever is in front.
People in similar clothes can still be confused once the target is out of view.

**Distance** comes from the box: the smaller of `fy * 1.7 m / box height` and
`fx * 0.45 m / box width`. Close up, when the feet are out of frame, the box is
short and the height estimate reads too far; the width estimate catches that,
and both err toward stopping early. Tall or short people shift it by their
height ratio (`track.person_height`).

## Camera

It reads the MJPEG relay th_foxglove already runs (`camera_url`, the Arducam
bench camera at 1280x720). The camera must face forward and roughly level.
If it is mounted sideways or upside down, set `camera.rotation` (90, 180 or
270 degrees clockwise) so frames reach the detector upright; calibrated
intrinsics are then for the turned image.

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
(arming rules, Y/B/LB, the stop on disable, follows a rendered tag), YOLOX
decoding, person distance, and the lock (keeps the same person among others,
won't switch to a stranger, forgets after `forget_time`). The person node and
the real model run when `models/yolox_nano.onnx` is present and are skipped
otherwise. Not yet checked on hardware: blur while walking, real distance
accuracy, and the closed loop on the robot.
