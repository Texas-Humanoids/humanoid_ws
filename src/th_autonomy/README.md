# th_autonomy

Camera-driven behaviors that drive the walking policy the same way the gamepad
does: by publishing `/cmd_vel`. Nothing about the policy or bridge changes.

## tag_follower: walk to an AprilTag

```
/camera/image/compressed ──► tag_follower ──/cmd_vel──► policy_bridge
/policy_bridge/mode ───────►      ▲
/gamepad/button_event ────────────┤
/joy (LB override) ───────────────┘
```

Hold up a tag and the robot turns toward it and walks until it is
`target_distance` (1 m) away. Step back and it follows; step closer and it
backs up slowly. A tag far to the side makes it turn in place first.

```bash
ros2 launch th_bringup robot.launch.py \
  motors_config:=src/th_bringup/config/walker_v2_robot.yaml \
  policy_spec:=policies/walker_v2/v6/policy_spec.json \
  model_path:=policies/walker_v2/v6/policy.onnx walk_event:='A pressed' \
  tag_follow:=true
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

### Topics

| Topic | |
|---|---|
| `~/state` | `DISABLED`, `OVERRIDE`, `TRACKING`, `LOST` or `SEARCH` (latched) |
| `~/target` | `PointStamped`, the followed tag's center in the camera frame (x right, y down, z forward) |
| `~/image/compressed` | Camera view with tags outlined, distance and bearing. Only encoded while subscribed. |

Detection runs while following is off too, so check `~/image/compressed` in
Foxglove before pressing Y.

### Tags

Default family `tag36h11` (the standard). Print any id; `target_id: -1`
follows the nearest. Set `tag_size` to the measured side of the **outer black
square**. A 16 cm tag reads out to about 4 m at 1280x720; tape it to stiff
cardboard so it stays flat. Print one:

```bash
python3 -c "import cv2; from th_autonomy.tag_detector import render_tag; \
cv2.imwrite('tag0.png', cv2.copyMakeBorder(render_tag('tag36h11', 0, 800), 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255))"
```

### Camera

It reads the MJPEG relay th_foxglove already runs (`camera_url`, the Arducam
bench camera at 1280x720). The camera must face forward and roughly level.

- **Bearing** (which way to turn) is nearly independent of calibration.
- **Distance** is proportional to the focal length. Without calibration it
  is guessed from `camera.hfov_deg` (90°). Check it against a tape measure:
  if `~/target` z reads 1.2 m at a true 1.0 m, scale `hfov_deg` until it
  matches, or calibrate (`camera.fx/fy/cx/cy/distortion`, at the stream's
  full resolution) with `ros2 run camera_calibration cameracalibrator`.
- **Motion blur** from the gait is the main risk. Detections are smoothed and
  a dropout under `hold_time` (0.3 s) is bridged. If the tag still drops out
  while walking, lower the camera's exposure in `/etc/default/bench-camera`.

### Tuning (`config/tag_follower.yaml`, `follow.*`)

Speeds stay inside the policy's training ranges: forward ≤ 0.3 m/s
(trained to 0.5), backing ≤ 0.1 m/s, yaw ≤ 0.4 rad/s (trained to 0.5), with
acceleration limits so it never sees a step command. Raise `max_forward` once
it walks reliably. `search_yaw_rate` > 0 makes it turn toward where it last
saw a lost tag (try 0.25).

## Test

```bash
colcon build --packages-select th_autonomy
colcon test --packages-select th_autonomy
colcon test-result --verbose
```

Tests cover the controller (deadbands, turn-in-place, clamps, ramping,
dropouts, search), detection and pose on rendered tags, and the node over DDS
(arming rules, Y/B/LB, the stop on disable, follows a rendered tag).
Not yet checked on hardware: blur while walking, real distance accuracy,
and the closed loop on the robot.
