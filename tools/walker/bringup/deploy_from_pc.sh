#!/usr/bin/env bash
# Deploy the stand-only demo + safety changes from this PC to the Jetson.
#   bash jetson_snapshot/deploy_demo.sh          # dry run: show what differs on the Jetson, change nothing
#   bash jetson_snapshot/deploy_demo.sh --go     # back up, copy, build, run the bridge unit tests
# Never runs while a launch is active (motors could be live). LF line endings are enforced.
set -euo pipefail
J=humanoids@100.118.244.11
R=/home/humanoids/humanoid_ws
WS=/c/Users/karti/OneDrive/Documents/humanoids_sw
SAFE=$WS/humanoid_ws_safety
SYS=$WS/humanoid_ws_sysid
STAMP=$(date +%Y%m%d_%H%M%S)
STAGE=$(mktemp -d)
SSH="ssh -o BatchMode=yes -o ConnectTimeout=15 $J"

# local source -> path on the Jetson (relative to ~/humanoid_ws)
declare -A FILES=(
  [$SAFE/src/th_policy_bridge/include/th_policy_bridge/heading_hold.hpp]=src/th_policy_bridge/include/th_policy_bridge/heading_hold.hpp
  [$SAFE/src/th_policy_bridge/include/th_policy_bridge/joint_mapping.hpp]=src/th_policy_bridge/include/th_policy_bridge/joint_mapping.hpp
  [$SAFE/src/th_policy_bridge/include/th_policy_bridge/observation.hpp]=src/th_policy_bridge/include/th_policy_bridge/observation.hpp
  [$SAFE/src/th_policy_bridge/include/th_policy_bridge/safety.hpp]=src/th_policy_bridge/include/th_policy_bridge/safety.hpp
  [$SAFE/src/th_policy_bridge/include/th_policy_bridge/stand_balance.hpp]=src/th_policy_bridge/include/th_policy_bridge/stand_balance.hpp
  [$SAFE/src/th_policy_bridge/src/policy_bridge.cpp]=src/th_policy_bridge/src/policy_bridge.cpp
  [$SAFE/src/th_policy_bridge/CMakeLists.txt]=src/th_policy_bridge/CMakeLists.txt
  [$SAFE/src/th_policy_bridge/package.xml]=src/th_policy_bridge/package.xml
  [$SAFE/src/th_policy_bridge/test/test_bridge_math.cpp]=src/th_policy_bridge/test/test_bridge_math.cpp
  [$SAFE/src/th_policy_bridge/test/test_pipeline.py]=src/th_policy_bridge/test/test_pipeline.py
  [$WS/jetson_snapshot/robot.launch.patched.py]=src/th_bringup/launch/robot.launch.py
  [$SAFE/src/th_bringup/config/mcap_nochunk.yaml]=src/th_bringup/config/mcap_nochunk.yaml
  [$SAFE/tools/jetson/99-ftdi-latency.rules]=tools/jetson/99-ftdi-latency.rules
  [$SYS/tools/walker/bringup/gyro_check.py]=tools/walker/bringup/gyro_check.py
  [$SYS/tools/walker/bringup/imu_timing.py]=tools/walker/bringup/imu_timing.py
  [$SYS/tools/walker/bringup/obs_dump.py]=tools/walker/bringup/obs_dump.py
  [$SYS/tools/walker/bringup/obs_series.py]=tools/walker/bringup/obs_series.py
  [$SYS/tools/walker/bringup/bag_window.py]=tools/walker/bringup/bag_window.py
  [$SYS/tools/walker/bringup/bag_export.py]=tools/walker/bringup/bag_export.py
  [$SYS/src/th_bringup/config/walker_demo_robot.yaml]=src/th_bringup/config/walker_demo_robot.yaml
  [$SYS/tools/walker/bringup/ros.sh]=tools/walker/bringup/ros.sh
  [$SYS/tools/walker/bringup/bag_summary.py]=tools/walker/bringup/bag_summary.py
  [$SYS/tools/walker/bringup/clear_all.py]=tools/walker/bringup/clear_all.py
  [$SYS/tools/walker/bringup/DEMO_STAND.md]=tools/walker/bringup/DEMO_STAND.md
  [$SYS/tools/walker/bringup/turn_check.py]=tools/walker/bringup/turn_check.py
  [$SYS/tools/walker/bringup/stand_summary.py]=tools/walker/bringup/stand_summary.py
  # Foxglove dashboard (ros.sh dashboard) and the walker_demo model for its 3D view
  [$SAFE/src/th_foxglove/th_foxglove/robot_monitor.py]=src/th_foxglove/th_foxglove/robot_monitor.py
  [$SAFE/src/th_foxglove/th_foxglove/telemetry.py]=src/th_foxglove/th_foxglove/telemetry.py
  [$SAFE/src/th_foxglove/launch/foxglove.launch.py]=src/th_foxglove/launch/foxglove.launch.py
  [$SAFE/src/th_foxglove/tools/make_layout.py]=src/th_foxglove/tools/make_layout.py
  [$SAFE/src/th_foxglove/tools/check_layout_topics.py]=src/th_foxglove/tools/check_layout_topics.py
  [$SAFE/src/th_foxglove/layouts/walker_demo.json]=src/th_foxglove/layouts/walker_demo.json
  [$SAFE/src/th_foxglove/layouts/walker_v2.json]=src/th_foxglove/layouts/walker_v2.json
  [$SAFE/src/th_foxglove/test/test_telemetry.py]=src/th_foxglove/test/test_telemetry.py
  [$SAFE/src/th_foxglove/test/test_sampling.py]=src/th_foxglove/test/test_sampling.py
  [$WS/humanoid_ws/src/th_description/robots/walker_demo/urdf/walker_demo.urdf]=src/th_description/robots/walker_demo/urdf/walker_demo.urdf
  [$WS/humanoid_ws/src/th_description/robots/walker_demo/meshes/base_link.stl]=src/th_description/robots/walker_demo/meshes/base_link.stl
  [$WS/humanoid_ws/src/th_description/robots/walker_demo/meshes/left_foot_link.stl]=src/th_description/robots/walker_demo/meshes/left_foot_link.stl
  [$WS/humanoid_ws/src/th_description/robots/walker_demo/meshes/left_hip_pitch_link.stl]=src/th_description/robots/walker_demo/meshes/left_hip_pitch_link.stl
  [$WS/humanoid_ws/src/th_description/robots/walker_demo/meshes/left_hip_roll_link.stl]=src/th_description/robots/walker_demo/meshes/left_hip_roll_link.stl
  [$WS/humanoid_ws/src/th_description/robots/walker_demo/meshes/left_shank_link.stl]=src/th_description/robots/walker_demo/meshes/left_shank_link.stl
  [$WS/humanoid_ws/src/th_description/robots/walker_demo/meshes/left_thigh_link.stl]=src/th_description/robots/walker_demo/meshes/left_thigh_link.stl
  [$WS/humanoid_ws/src/th_description/robots/walker_demo/meshes/right_foot_link.stl]=src/th_description/robots/walker_demo/meshes/right_foot_link.stl
  [$WS/humanoid_ws/src/th_description/robots/walker_demo/meshes/right_hip_pitch_link.stl]=src/th_description/robots/walker_demo/meshes/right_hip_pitch_link.stl
  [$WS/humanoid_ws/src/th_description/robots/walker_demo/meshes/right_hip_roll_link.stl]=src/th_description/robots/walker_demo/meshes/right_hip_roll_link.stl
  [$WS/humanoid_ws/src/th_description/robots/walker_demo/meshes/right_shank_link.stl]=src/th_description/robots/walker_demo/meshes/right_shank_link.stl
  [$WS/humanoid_ws/src/th_description/robots/walker_demo/meshes/right_thigh_link.stl]=src/th_description/robots/walker_demo/meshes/right_thigh_link.stl
)

$SSH true || { echo "Jetson not reachable"; exit 1; }
if $SSH "cd $R && docker compose --profile robot exec -T robot pgrep -f 'ros2 launch th_bringup'" >/dev/null 2>&1; then
  echo "A launch is running on the Jetson: stop it first (tools/walker/bringup/ros.sh stop)"; exit 1
fi

# stage with LF endings, then diff against what is on the Jetson now
for src in "${!FILES[@]}"; do
  dst=${FILES[$src]}
  mkdir -p "$STAGE/new/$(dirname "$dst")"
  case "$dst" in
    *.stl|*.STL|*.dae|*.png|*.onnx) cp "$src" "$STAGE/new/$dst" ;;   # binary: never strip CRs
    *) sed 's/\r$//' "$src" > "$STAGE/new/$dst" ;;
  esac
done
mkdir -p "$STAGE/old"
$SSH "cd $R && tar czf - --ignore-failed-read ${FILES[*]} 2>/dev/null" | tar xzf - -C "$STAGE/old" || true
echo "== differences (Jetson now -> this deploy):"
for dst in "${FILES[@]}"; do
  if [ ! -e "$STAGE/old/$dst" ]; then echo "  NEW      $dst"
  elif ! cmp -s "$STAGE/old/$dst" "$STAGE/new/$dst"; then
    echo "  CHANGED  $dst ($(diff "$STAGE/old/$dst" "$STAGE/new/$dst" | grep -c '^[<>]') lines)"
  fi
done
echo "  (full diffs: diff -r $STAGE/old $STAGE/new)"

if [ "${1:-}" != "--go" ]; then
  echo "dry run: nothing changed. Review the diffs (look for teammates' edits), then rerun with --go"; exit 0
fi

echo "== backup -> ~/bak_20261119/pre_demo_$STAMP"
$SSH "cd $R && mkdir -p ~/bak_20261119/pre_demo_$STAMP && tar czf ~/bak_20261119/pre_demo_$STAMP/files.tgz --ignore-failed-read ${FILES[*]} 2>/dev/null; ls -la ~/bak_20261119/pre_demo_$STAMP"
echo "== copy"
(cd "$STAGE/new" && tar czf - .) | $SSH "cd $R && tar xzf - && chmod +x tools/walker/bringup/ros.sh tools/walker/bringup/*.py"
echo "== build"
$SSH "cd $R && docker compose --profile robot up -d >/dev/null 2>&1; docker compose --profile robot exec -T robot bash -lc 'source /opt/ros/jazzy/setup.bash && cd /ws && colcon build --packages-select th_policy_bridge th_bringup th_foxglove th_description 2>&1 | tail -8'"
echo "== unit tests (th_policy_bridge)"
$SSH "cd $R && docker compose --profile robot exec -T robot bash -lc 'source /opt/ros/jazzy/setup.bash && cd /ws && source install/setup.bash && colcon test --packages-select th_policy_bridge --event-handlers console_direct- >/dev/null 2>&1; colcon test-result --verbose 2>&1 | tail -25'"
echo "deployed. Rollback: ssh $J 'cd $R && tar xzf ~/bak_20261119/pre_demo_$STAMP/files.tgz' and rebuild"
