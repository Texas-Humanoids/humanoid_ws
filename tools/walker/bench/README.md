# WalkerV2 bench tools

Scripts from the 2026-11-13 Jetson bench session (two RS06 motors on `can_usb`,
VN-100, Xbox controller). Run them inside the robot container from the repo
root after `source install/setup.bash`. Every motion is small and capped by
the config's `torque_limit`. Keep hands clear, and keep someone at the power switch.

| Script | What it checks |
|---|---|
| `scale_check.py <motor_id>` | Position decoded from MIT status frames vs the motor's own `mechPos` register (true radians). Sends only parameter reads; run it while a motor node is up in MIT mode |
| `move_test.py <ns> <rad> [ramp] [hold]` | Slow visible move on one motor node (e.g. `/m1 1.5708` = quarter turn and back) |
| `hold_test.py <ns> <kp> <rad>` | Hold the motor's face with a whole-hand grip while the target ramps away. Reports torque / error = measured kp |
| `pipeline_watch.py <robot.yaml> <s>` | During `robot.launch.py`: bridge mode, button events, joint angles, IMU tilt |
| `policy_watch.py <robot.yaml> <s>` | During a policy run: command, target vs actual range per joint, tracking error, swing period |
| `bench_two_motors.yaml` | Bench config: ID 1 = left hip pitch, ID 2 = left ankle pitch (STAND gains 80/3), torque capped at 5 Nm |

Start a single motor node directly (not through `ros2 run` from a script) and
stop it with SIGTERM. Non-interactive shells start background jobs with SIGINT
ignored, so Ctrl-C-style stops leave the node running and fighting the next one:

```bash
install/robstride_motor/lib/robstride_motor/robstride_motor_node --ros-args -r __ns:=/m1 \
  -p motor_id:=1 -p motor_model:=RS06 -p can_interface:=can_usb -p control_mode:=mit \
  -p kp:=5.0 -p kd:=0.3 -p torque_limit:=3.0 &
python3 tools/walker/bench/move_test.py /m1 1.5708
kill -TERM %1
```

For the full pipeline:

```bash
python3 tools/walker/calibrate_offsets.py tools/walker/bench/bench_two_motors.yaml \
  policies/walker_v2/v6/policy_spec.json --write   # with the launch below running
ros2 launch th_bringup robot.launch.py motors_config:=tools/walker/bench/bench_two_motors.yaml \
  policy_spec:=policies/walker_v2/v6/policy_spec.json \
  model_path:=policies/walker_v2/v6/policy.onnx walk_event:='A pressed'
```

What the session showed:
- MIT mode tracks and damps; positions match `mechPos` within 2.5 mrad.
- Measured kp is within about 10 % of commanded.
- STAND → WALK → tilt cutoff → BACK all work.
- v6 runs at 50 Hz and follows the stick (hold **LB**).
- The ankle switches from kp 80 in STAND to 30 at WALK.
