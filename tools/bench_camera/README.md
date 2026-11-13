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
