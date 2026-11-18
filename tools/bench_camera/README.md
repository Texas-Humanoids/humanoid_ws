# bench_camera

Live video feed from the Arducam USB webcam (1080P Low Light, UC684) on the
tabletop bench, so you can watch the motors move while testing code.

It runs [µStreamer](https://github.com/pikvm/ustreamer) on the Jetson as a
systemd service. The camera's own MJPEG frames are passed through without
re-encoding, so it uses almost no CPU and doesn't touch the ROS container.

## Watching the feed

Open in any browser on the same network or tailnet:

- `http://humanoids:8080/stream` — live video
- `http://humanoids:8080/snapshot` — a single JPEG frame
- `http://humanoids:8080/` — µStreamer status page

If `humanoids` doesn't resolve, use the Jetson's Tailscale IP (`tailscale ip -4`
on the Jetson). The camera only streams at full rate while someone is watching.

## Install / update (on the Jetson)

```bash
./tools/bench_camera/install.sh
```

Settings (device, resolution, fps, port) live in `/etc/default/bench-camera`.
After editing them:

```bash
sudo systemctl restart bench-camera
```

Logs: `journalctl -u bench-camera -f`. The startup line "Device doesn't support
setting of HW encoding quality parameters" is harmless.

## Camera notes

The camera shows up as four `/dev/video*` nodes. Use `video-index0` (MJPEG/YUYV,
up to 1920x1080 @ 30 fps). `video-index2` is H.264 only; 1 and 3 are metadata.
The service uses the `/dev/v4l/by-id/...` path so it doesn't depend on
enumeration order.

If the feed turns pink and green with vertical stripes, the camera's own JPEG
encoder has glitched (the camera controls stay at their defaults and the kernel
logs no USB errors). `sudo systemctl restart bench-camera` fixed it on
2026-11-17. If it comes back, replug the camera, or move it off the shared USB 2.0 hub
onto one of the Orin's own ports or a powered hub.

## Lens calibration

[`camera_bench.yaml`](camera_bench.yaml) is the lens calibration for this camera,
in OpenCV `FileStorage` format. The lens is wide (about 92° horizontal) and bends
straight lines near the edges, so anything that measures distances or angles from
the image (e.g. AprilTag distance and bearing) needs it.

| | |
|---|---|
| Camera | Arducam 1080P Low Light (UC684), USB, `video-index0`, MJPEG |
| Resolution | **1280x720 only** (the service default). A different `CAMERA_RESOLUTION` needs a new calibration |
| Focal length | fx 613.4 px, fy 614.9 px |
| Optical centre | cx 644.5 px, cy 373.2 px |
| Distortion (k1, k2, p1, p2, k3) | -0.3359, 0.1158, -0.0020, -0.0007, -0.0187 |
| Reprojection error | 0.52 px over 35 views |
| Horizontal field of view | about 92° |

Load it with OpenCV:

```python
fs = cv2.FileStorage("tools/bench_camera/camera_bench.yaml", cv2.FILE_STORAGE_READ)
K, dist = fs.getNode("camera_matrix").mat(), fs.getNode("dist_coeffs").mat()
```

perception-lab's `live_detect.py` takes it as `--camera-file`. Its ROS node does too
(`camera_file:=`), but only pass it there when the node is fed this camera; the
RealSense publishes its own factory calibration on `camera_info`.

### How it was made (2026-11-17)

`tools/calibrate_camera.py` from perception-lab, reading the live stream at
`http://127.0.0.1:8080/stream` on the Jetson. It uses a 9x7-inner-corner chessboard
(10x8 squares) printed on US Letter, with 20 mm squares checked against the sheet,
`cv2.findChessboardCornersSB`, and the standard 5-coefficient OpenCV model. The
script keeps a frame only when the board is at a new position or size in the image.
Six runs; times are the stream connection times in the bench-camera logs:

| Run | Board | Views | Capture time | Error | fx / fy | Notes |
|---|---|---|---|---|---|---|
| 1 | paper, held far | 25 | 1 min 1 s | 0.80 px | 555 / 550 | centre of the image only |
| 2 | paper, held close | 30 | 2 min 5 s | 2.31 px | 693 / 687 | rejected: the sheet bent when held up close |
| 3 | rigid backing | 30 | 56 s | 0.76 px | 537 / 544 | centre and upper left only, two blurred views |
| 4 | rigid backing | 35 | 1 min 0 s | 0.75 px | 571 / 577 | upper half only |
| 5 | rigid backing | 35 | 43 s | 0.55 px | 637 / 640 | top, centre and bottom right |
| **6** | rigid backing | 35 | 41 s | **0.52 px** | **613 / 615** | **kept**: lowest error; agrees with run 5 on distortion (k1 -0.336 vs -0.348, k2 0.116 vs 0.118) |

Runs 1-4 had poor coverage of the image edges, so focal length and distortion
traded off against each other and the runs disagreed (fx 537-577). Runs 5 and 6
covered the edges and agree within 4% on focal length. Run 6 was captured after
the pink and green glitch above was fixed. Its weak spots are the bottom-left corner
and strongly tilted views.

### Distance check (not yet conclusive)

The board lay flat with the camera pointed straight down, with the lens estimated
at 2 ft (0.610 m) above it. That height was estimated, not measured. Distance is the
median over 5 frames (`cv2.solvePnP`, lens to board plane):

| Calibration | Measured | vs. 0.610 m |
|---|---|---|
| **Run 6 (kept)** | 0.733 m | **+20.3%** |
| Run 5 | 0.761 m | +24.8% |
| Run 4 | 0.681 m | +11.8% |
| Run 1 | 0.662 m | +8.7% |

The 20% gap is either the estimated height (0.733 m is 29 in) or an error in
the calibration's focal length. It has to be settled with a tape-measured height before relying
on absolute distances. Run 6's internal checks are good (0.52 px, fx ≈ fy, centre near
the image centre, agreement with run 5), but they can't rule out a scale error on their own.

### Redoing it

Mount the board flat on something rigid (paper alone bends). Pause briefly at each
position. Cover all four corners and edges, near and far, and tilt the board 30-45° while
moving it (tilting in place is skipped as a duplicate). Write to a new file name and
replace `camera_bench.yaml` only if the result is clearly better.
