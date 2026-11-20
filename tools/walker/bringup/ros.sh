#!/usr/bin/env bash
# Drive the ROS stack (compose "robot" container) from the Jetson host, non-interactively.
#
#   tools/walker/bringup/ros.sh up                 # start the robot container
#   tools/walker/bringup/ros.sh build              # colcon build (needed after source changes)
#   tools/walker/bringup/ros.sh launch [cad|cad_safe|wide] [extra args]  # detached, slow gamepad; log in bringup_logs/
#   tools/walker/bringup/ros.sh launch stand           # STAND-ONLY DEMO: START stands on stiff PD, A does nothing, B disarms
#   tools/walker/bringup/ros.sh launch stand-balance   # same + slow COM centering on the ankles (stand_balance.hpp)
#   tools/walker/bringup/ros.sh launch stand-auto      # same as stand + auto-lean: holds +1.5 N m ankle torque, +-0.03 rad
#   tools/walker/bringup/ros.sh launch stand-auto-y    # auto-lean waits for Y (press once it carries its own weight), +-0.015 rad; X = one balanced squat (SQUAT_DEPTH, default 2 cm)
#   tools/walker/bringup/ros.sh status             # is it running, bridge mode, last log lines
#   tools/walker/bringup/ros.sh check [sec] [policy]   # IMU gravity/gyro, observation by joint, cmd_vel, mode
#   tools/walker/bringup/ros.sh log [n]            # last n lines + warnings/errors of the current launch
#   tools/walker/bringup/ros.sh stop               # SIGINT the launch, then kill stragglers (motors go limp)
#   tools/walker/bringup/ros.sh boot               # after a reboot/power cut: CAN up, clocks locked, container up
#   tools/walker/bringup/ros.sh restart [policy]   # after a USB/CAN drop: stop, CAN up, launch, health
#   tools/walker/bringup/ros.sh recover            # drives dropped out but CAN is fine: re-enable them, no restart
#   tools/walker/bringup/ros.sh health             # one-screen status from the logs (starts no ROS process)
#   tools/walker/bringup/ros.sh dashboard [stop|status|log]  # Foxglove dashboard (ws://<jetson>:8765) in its own
#                                                  # container, capped at 1 CPU core; import th_foxglove/layouts/walker_demo.json
#   tools/walker/bringup/ros.sh follow [person|tag|stop|status|log]  # person (default) or AprilTag follower on
#                                                  # the GPU: detections over the dashboard camera; Y follows in WALK
set -uo pipefail
cd "$(dirname "$0")/../../.." || exit 1          # repo root (~/humanoid_ws, /ws in the container)
DC="docker compose --profile robot"
ROS='source /opt/ros/jazzy/setup.bash && cd /ws && source install/setup.bash'
LOGDIR=bringup_logs
mkdir -p "$LOGDIR"
# What launch records (ros2 bag, into bringup_logs/bag_*): policy in/out, joints, IMU, gamepad,
# bridge mode, motor torque/status/targets. Lean on purpose: recording every motor topic at
# 100 Hz loads the Jetson enough to cause feedback stalls. RECORD=0 ros.sh launch ... turns it off.
REC='/policy/.*|/policy_bridge/.*|/cmd_vel|/gamepad/.*|/joy|/imu/data|/joint_states|/[a-z_]+/motor/(actual_torque|status|target_position)|/rosout'


running() { $DC exec -T robot pgrep -f "ros2 launch th_bringup" >/dev/null 2>&1; }

# The IMU's FTDI USB-serial must flush every 1 ms: at the 16 ms default the IMU reaches the policy in
# ~55 ms bursts and every policy steps within a second (2026-11-19). Permanent fix: install
# tools/jetson/99-ftdi-latency.rules into /etc/udev/rules.d. This only checks; it changes nothing.
imu_latency_check() {
  for p in /sys/bus/usb-serial/devices/ttyUSB*/latency_timer; do
    [ -e "$p" ] || continue
    v=$(cat "$p")
    if [ "$v" != 1 ]; then
      echo "!! IMU serial latency_timer is ${v} ms (needs 1): policies will get stale IMU. Fix now:"
      echo "!!   echo 1 | sudo tee $p    (permanent: tools/jetson/99-ftdi-latency.rules)"
    else
      echo "IMU serial latency_timer 1 ms (ok)"
    fi
  done
}

case "${1:-}" in
  up)
    $DC up -d && $DC ps ;;
  build)
    $DC exec -T robot bash -lc "source /opt/ros/jazzy/setup.bash && cd /ws && \
      colcon build --packages-up-to th_bringup 2>&1 | tail -25" ;;
  launch)
    v=${2:-cad}; v_arg=$v; shift; [ $# -gt 0 ] && shift
    if running; then echo "already running: ros.sh stop first"; exit 1; fi
    # Conservative limits (sized in sim, 2026-11-19): torque caps go into each drive's firmware
    # torque limit; speed caps are the bridge's speed trip (disarm after 50 ms over).
    #   stand: uses <= 7.6 N m and <= 0.74 rad/s through 0.3-0.5 m/s pushes; caps change nothing in sim.
    #   policy (cad_v1): uses up to 37 N m knee / 29 ankle walking with pushes, <= 7.4 rad/s ankle;
    #   with these caps it still finishes the scripted walk + pushes without a fall.
    # Override with TORQUE_CAPS=... / SPEED_CAPS=... (type=value,...), or "none" for the yaml values.
    we="A pressed"; bal=""
    tq="hip_pitch=25,hip_roll=30,hip_yaw=10,knee_pitch=35,ankle_pitch=20"
    sp="hip_pitch=4.5,hip_roll=2.5,hip_yaw=5,knee_pitch=8,ankle_pitch=10"
    case $v in   # stand-only demo: no policy control. The policy node still runs (the bridge needs its
                 # spec for the gains and stand pose) but no button hands it the motors.
      stand|stand-balance|stand-auto|stand-auto-y)
        we="(walking disabled: stand-only demo)"
        bal="stand_balance_gain:=0.0"; [ "$v" = stand-balance ] && bal="stand_balance_gain:=0.0015"
        # auto-lean: saved D-pad lean (yaml stand_balance.lean) + a slow trim holding the measured
        # balance point (mean ankle torque +1.5 N m), correcting at most +-0.03 rad around the lean.
        # Override the target with AUTO_TARGET=... (N m).
        [ "$v" = stand-auto ] && bal="stand_balance_gain:=0.0015 stand_balance_max_lean:=${AUTO_MAX:-0.03} stand_balance_target_torque:=${AUTO_TARGET:-1.5}"
        # stand-auto-y: same trim, but it only starts when Y is pressed (lower the robot to full
        # weight first), and corrects at most +-0.015 rad: an always-on trim wound up during
        # lowering and swung 0.06 rad, wider than the balance window (2026-11-19 floor tests).
        # + X: one scripted balanced squat (SquatMotion; IK from th_training/squat_study.py). Depth
        # SQUAT_DEPTH m, default 0.02 = half the sim's worst-case limit with the trim on.
        [ "$v" = stand-auto-y ] && bal="stand_balance_gain:=0.0015 stand_balance_max_lean:=${AUTO_MAX:-0.015} stand_balance_target_torque:=${AUTO_TARGET:-1.5} stand_balance_arm_event:='Y pressed' stand_squat_event:='X pressed' stand_squat_depth:=${SQUAT_DEPTH:-0.02}"
        tq="hip_pitch=15,hip_roll=15,hip_yaw=6,knee_pitch=20,ankle_pitch=10"
        sp="hip_pitch=2,hip_roll=2,hip_yaw=2,knee_pitch=2,ankle_pitch=2"
        v=cad ;;
    esac
    tq=${TORQUE_CAPS:-$tq}; sp=${SPEED_CAPS:-$sp}
    [ "$tq" = none ] && tq=""; [ "$sp" = none ] && sp=""
    walk="walk_event:='$we' torque_caps:='$tq' speed_caps:='$sp' $bal"
    # policy folder: policies/walker_demo/<name> (e.g. cad_v3), else <name>_v1 (cad -> cad_v1)
    pol=policies/walker_demo/$v; [ -f "$pol/policy_spec.json" ] || pol=policies/walker_demo/${v}_v1
    [ -f "$pol/policy_spec.json" ] || { echo "no policy at policies/walker_demo/$v or ${v}_v1"; exit 1; }
    stamp=$(date +%m%d_%H%M%S)
    log=$LOGDIR/launch_${stamp}_$v.log
    rec=""
    [ "${RECORD:-1}" = 1 ] && rec="record:=true record_path:=/ws/$LOGDIR/bag_${stamp}_$v record_topics:='$REC'"
    ln -sf "$(basename "$log")" $LOGDIR/latest.log
    # A power cut loses whatever Linux has not written to the SSD yet (up to ~30 s): sync every
    # 5 s while the launch runs (every 1 s + an uncached recorder stalled the NVMe, 2026-11-19 21:00),
    # and keep the launch log unbuffered.
    $DC exec -d robot bash -c "sleep 5; while pgrep -f '[r]os2 launch th_bringup' >/dev/null; do sync; sleep 5; done"
    $DC exec -d -e PYTHONUNBUFFERED=1 robot bash -lc "$ROS && exec ros2 launch th_bringup robot.launch.py \
      motors_config:=src/th_bringup/config/walker_demo_robot.yaml \
      policy_spec:=$pol/policy_spec.json \
      model_path:=$pol/policy.onnx \
      $walk gamepad_config:=src/th_bringup/config/gamepad_demo.yaml foxglove:=false $rec $* > /ws/$log 2>&1"
    sleep 10
    echo "launched ($v): $log"; [ -n "$rec" ] && echo "recording: $LOGDIR/bag_${stamp}_$v"
    echo "limits: torque caps [${tq:-yaml}]  speed trip [${sp:-yaml}]"; tail -n 25 "$log"
    grep -i -E "error|refus|exception|died|not running" "$log" | head -10
    # A RobStride drive can boot into a different output turn (reads 2 pi off); START would then
    # swing that joint through a full turn (the speed trip stops it). Check every launch.
    imu_latency_check
    sleep 3; python3 tools/walker/bringup/turn_check.py "$log" --fix | tail -11
    if [ "${PIPESTATUS[0]}" = 2 ] && [ -z "${TURN_FIXED:-}" ]; then
      echo "== a drive booted one turn off: offset fixed in the yaml; restarting the stack once"
      "$0" stop >/dev/null; TURN_FIXED=1 exec "$0" launch "$v_arg" "$@"
    fi ;;
  status)
    if running; then echo "launch: RUNNING"; else echo "launch: not running"; fi
    $DC exec -T robot bash -lc "$ROS && timeout 4 ros2 topic echo --once /policy_bridge/mode 2>/dev/null | head -2" || true
    [ -e $LOGDIR/latest.log ] && tail -n 8 $LOGDIR/latest.log ;;
  check)
    $DC exec -T robot bash -lc "$ROS && python3 /ws/tools/walker/bringup/ros_obs.py \
      --spec /ws/policies/walker_demo/${3:-cad}_v1/policy_spec.json --seconds ${2:-3}" ;;
  log)
    tail -n "${2:-40}" $LOGDIR/latest.log
    echo "--- warnings/errors:"; grep -i -E "warn|error|late|disarm|refus" $LOGDIR/latest.log | tail -n 20 ;;
  stop)
    $DC exec -T robot bash -lc "pkill -INT -f 'ros2 launch th_bringup'; sleep 3; \
      pkill -9 -f 'robstride_motor_node|policy_bridge|policy_node|gamepad_node|vectornav' ; true"
    sleep 1; running && echo "STILL RUNNING" || echo "stopped (motors go limp on their 0.2 s CAN timeout)" ;;
  boot)
    tools/jetson/can_status.sh 2>&1 | grep -E "^can_(left|right)|motors:"
    sudo -n jetson_clocks && echo "clocks locked (until the next reboot)"
    imu_latency_check
    docker inspect -f '{{.State.Status}}' tagfollow_demo 2>/dev/null | grep -q running &&       { docker stop tagfollow_demo >/dev/null; echo "paused tagfollow_demo (Foxglove): docker start tagfollow_demo"; }
    $DC up -d 2>&1 | tail -1 ;;
  restart)
    $DC up -d >/dev/null 2>&1   # the container itself may be down (reboot, docker restart)
    "$0" stop
    tools/jetson/can_status.sh 2>&1 | grep -E "^can_(left|right)|motors:"
    shift; "$0" launch "$@" | grep -E "^launched|^recording"
    sleep 12; "$0" health ;;
  recover)
    $DC exec -T robot bash -lc "$ROS && python3 /ws/tools/walker/bringup/clear_all.py" ;;
  health)
    L=$LOGDIR/$(readlink $LOGDIR/latest.log 2>/dev/null)
    for i in can_left can_right; do
      echo "$i: $(ip -br link show $i 2>/dev/null | awk '{print $2}' || echo MISSING) $(ip link show $i 2>/dev/null | grep -o 'qlen [0-9]*')"
    done
    running && echo "launch: RUNNING ($L)" || { echo "launch: not running"; exit 0; }
    echo "drives: $(grep -a -c 'Motor mode: running' $L) enabled at start, $(grep -a -c 'left run mode' $L) dropped out since, $(grep -a -c 'Faults cleared' $L) re-enabled"
    echo "CAN write failures (last 200 lines): $(tail -n 200 $L | grep -a -c 'CAN write failed')"
    echo "controller: $(grep -a -q 'Opened game controller' $L && echo connected || echo NOT FOUND)"
    echo "bridge: $(grep -a 'policy_bridge' $L | grep -a -v -E 'Subscribed|rosbag' | tail -1 | sed -E 's/.*\[policy_bridge\]: //' | cut -c1-110)"
    ls -d $LOGDIR/bag_* 2>/dev/null | tail -1 | sed 's/^/recording: /' ;;
  dashboard)
    # Foxglove dashboard (th_foxglove: foxglove_bridge + robot_state_publisher + robot_monitor)
    # in its OWN container, apart from the motor stack: hard-capped at 1 CPU core (docker
    # --cpus) and low priority, so it can never starve the motor nodes, and it can be started
    # or stopped any time without touching the robot. Uses this workspace's install (rebuild
    # th_foxglove + th_description after changes) and an image that has foxglove_bridge
    # (the robot image humanoid-ros:cpu predates it). Then in the Foxglove desktop app:
    # Open connection -> Foxglove WebSocket -> ws://<jetson>:8765, import
    # src/th_foxglove/layouts/walker_demo.json.
    NAME=walker_dashboard
    IMAGE=${DASH_IMAGE:-humanoid-ros:cpu-opencv-fm}
    case "${2:-start}" in
      stop) docker stop $NAME >/dev/null 2>&1 && echo "dashboard stopped" || echo "dashboard not running"; exit 0 ;;
      status) docker ps --filter name=$NAME --format '{{.Names}} {{.Status}}' | grep . || echo "dashboard not running"
              docker stats --no-stream --format 'cpu {{.CPUPerc}} mem {{.MemUsage}}' $NAME 2>/dev/null; exit 0 ;;
      log) docker logs --tail ${3:-40} $NAME 2>&1; exit 0 ;;
    esac
    if docker ps --format '{{.Names}}' | grep -qx tagfollow_demo; then
      echo "tagfollow_demo (the old dashboard) is running on port 8765: docker stop tagfollow_demo first"; exit 1
    fi
    docker ps --format '{{.Names}}' | grep -qx $NAME && { echo "dashboard already running (ros.sh dashboard stop)"; exit 0; }
    docker image inspect $IMAGE >/dev/null 2>&1 || { echo "no image $IMAGE (set DASH_IMAGE)"; exit 1; }
    URDF=/ws/install/th_description/share/th_description/robots/walker_demo/urdf/walker_demo.urdf
    [ -f "install/th_description/share/th_description/robots/walker_demo/urdf/walker_demo.urdf" ] || {
      echo "walker_demo URDF not installed: copy src/th_description/robots/walker_demo, then"
      echo "  $DC exec -T robot bash -lc \"$ROS && colcon build --packages-select th_description th_foxglove\""; exit 1; }
    # Camera on by default (bench-camera uStreamer), turned upright (mounted upside down);
    # CAMERA_URL=none turns it off, CAMERA_ROTATION=0 if it is ever remounted.
    CAMERA_URL=${CAMERA_URL:-http://localhost:8080/stream}; cam=""
    [ "$CAMERA_URL" != none ] && cam="camera_url:=$CAMERA_URL camera_rotation:=${CAMERA_ROTATION:-180}"
    # --ipc host + the host's /dev/shm: the robot container bind-mounts /dev (so it uses the
    # host's /dev/shm), and Fast DDS talks to "same host" participants through shared memory.
    # With a private /dev/shm here, the motor nodes' publishers stalled ~4 s after discovering
    # the dashboard and 8 drives timed out (2026-11-19 22:14, twice). Same as tagfollow_demo.
    docker run -d --rm --name $NAME --network host --ipc host -v /dev/shm:/dev/shm \
      --cpus ${DASH_CPUS:-1.0} -v "$PWD":/ws \
      $IMAGE bash -lc "source /opt/ros/jazzy/setup.bash && cd /ws && source install/setup.bash && \
      exec nice -n 10 ros2 launch th_foxglove foxglove.launch.py \
        robot_config:=src/th_bringup/config/walker_demo_robot.yaml urdf:=$URDF base_height:=0.661 $cam" >/dev/null || exit 1
    sleep 8
    docker logs $NAME 2>&1 | grep -a -E "listening|Monitoring|ERROR|Error|not installed" | head -6
    echo "dashboard: ws://$(hostname -I | awk '{print $1}'):8765 (or the Jetson's Tailscale IP); stop: ros.sh dashboard stop" ;;
  follow)
    # th_autonomy follower in its own GPU container (humanoid-ros:gpu, the compose "perception"
    # image), beside the motor stack and the dashboard. It reads the dashboard's camera relay
    # (/camera/image/compressed, already turned upright, so the follower's camera.rotation stays 0)
    # and publishes /follower/annotations and /follower/markers, which the dashboard's Camera and
    # 3D panels draw. Detection always runs; it only sends /cmd_vel after Y while the bridge is in
    # WALK (th_autonomy/README.md). The person model must be in models/ (tools/fetch_person_model.py);
    # its TensorRT engine is cached there after the first start (~10 min).
    NAME=walker_follow
    IMAGE=${FOLLOW_IMAGE:-humanoid-ros:gpu}
    case "${2:-person}" in
      stop) docker stop $NAME >/dev/null 2>&1 && echo "follower stopped" || echo "follower not running"; exit 0 ;;
      status) docker ps --filter name=$NAME --format '{{.Names}} {{.Status}}' | grep . || { echo "follower not running"; exit 0; }
              docker stats --no-stream --format 'cpu {{.CPUPerc}} mem {{.MemUsage}}' $NAME 2>/dev/null
              docker logs --tail 200 $NAME 2>&1 | grep -a -E "Ready|running on|state" | tail -3; exit 0 ;;
      log) docker logs --tail ${3:-40} $NAME 2>&1 | grep -a -v -E "^(=+|)$|NVIDIA|CUDA|[Ll]icense"; exit 0 ;;
      person|tag) TARGET=${2:-person} ;;
      *) echo "ros.sh follow [person|tag|stop|status|log]"; exit 1 ;;
    esac
    docker ps --format '{{.Names}}' | grep -qx $NAME && { echo "follower already running (ros.sh follow stop)"; exit 0; }
    docker ps --format '{{.Names}}' | grep -qx walker_dashboard ||
      echo "note: the dashboard (ros.sh dashboard) is not running, so there is no camera relay to read yet"
    docker image inspect $IMAGE >/dev/null 2>&1 || { echo "no image $IMAGE: $DC build perception (or set FOLLOW_IMAGE)"; exit 1; }
    if [ "$TARGET" = person ] && ! ls models/rtmo-l.onnx >/dev/null 2>&1; then
      echo "no models/rtmo-l.onnx: python3 tools/fetch_person_model.py"; exit 1
    fi
    # Same DDS setup as the dashboard (--ipc host + the host's /dev/shm), or a new participant can
    # stall the motor nodes. --rm, so a rebuilt install/ is picked up on the next start.
    GPU="docker run --rm --runtime nvidia -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
      --network host --ipc host -v /dev/shm:/dev/shm -v $PWD:/ws"
    [ -f install/th_autonomy/share/th_autonomy/package.xml ] || {
      echo "building th_autonomy"; $GPU $IMAGE bash -lc "source /opt/ros/jazzy/setup.bash && cd /ws && \
        colcon build --packages-select th_autonomy 2>&1 | tail -2" | grep -a -v -E "^(=+|)$|NVIDIA|CUDA|[Ll]icense"; }
    $GPU -d --name $NAME $IMAGE bash -lc "$ROS && exec ros2 run th_autonomy ${TARGET}_follower \
      --ros-args --params-file src/th_autonomy/config/${TARGET}_follower.yaml" >/dev/null || exit 1
    sleep 15
    docker logs $NAME 2>&1 | grep -a -E "Ready|running on|ERROR|Error|Traceback" | head -4
    echo "$TARGET follower running: detections show on the dashboard's Camera panel; stop: ros.sh follow stop" ;;
  *)
    sed -n 2,18p "$0"; exit 1 ;;
esac
