# Lab playbook: "do step N"

What Claude Code runs on the Jetson for each step, what it reports, and what it asks the person at the
robot. The physical procedure and pass criteria behind each step are in README.md (phases 0-8).

```bash
J=humanoids@100.118.244.11            # Tailscale; key auth, passwordless sudo
ssh $J 'cd ~/humanoid_ws && <cmd>'    # every command below runs from the repo root
BU="python3 tools/walker/bringup/bringup.py"   # --policy cad_safe|cad|wide picks the spec (default pose, used ranges)
ROS=tools/walker/bringup/ros.sh
```
Plain `python3`, no sudo: raw CAN doesn't need root, and root would make the yaml and session
files root-owned. `robot.launch.py` and `jog.py` must not run while `bringup.py` moves motors (it
refuses if it sees someone else's MIT frames).

**"stop" / "estop" at any time:** `$ROS stop; $BU estop`. Then tell them to cut motor power if
anything is still moving.

| Step | Robot state | Claude runs | Reports / asks |
|---|---|---|---|
| **0** prep (any time, robot can be off) | | from the laptop: push files (below); on the Jetson: `python3 -m pytest tools/walker/bringup/test_jog.py -q`, `$ROS up`, `$ROS build` (rebuilds robstride_motor_node: the RS03 enable fix is newer than the binary), `tools/jetson/can_status.sh` | tests pass, build ok, which adapters are up |
| **1** power + CAN | powered, legs hanging, PSU ~3 A | `tools/jetson/can_status.sh; $BU scan` | table of 10 IDs: adapter, VBUS, temp, faults, MIT/mechPos problems. If an adapter carries the wrong leg, fix `can_interface` in the yaml (ask first) |
| **2** each actuator | hanging, clear of people | `$BU scan` (quick), then `$BU wiggle` (all, ankles first; or `--ids 15,25`) | per motor: reached +-0.1, tracking error, peak and mean torque, PASS/FAIL. Asks: "any grinding, binding, cable pull?" |
| **3** directions | hanging | for each of the 10 joints: `$BU pulse <joint>`, ask the printed QUESTION, then `$BU set-direction <joint> +1` or `-1` | one question per joint ("did the left knee FLEX?"). Order: left ankle, knee, hip yaw, hip roll, hip pitch, then right |
| **4a** offsets | held in the **zero pose** (legs straight down, feet flat, toes forward) | `$BU calibrate` (then `$BU read`) | offsets written; refuses if anything moved (> 0.01 rad spread) |
| **4b** hard stops | limp; the person moves each joint by hand to BOTH stops | `$BU ranges --seconds 90` (ask them to start, then run) | measured range vs what the policy uses; suggested yaml limits. `--write-limits` only after they agree |
| **4c** power-cycle | cut + restore motor power, zero pose again | `$BU check-cal` | PASS = offsets survive power cycles; FAIL with ~0.7 rad jumps = calibrate every power-up |
| **5** holding | hanging | `$BU pose default --hold 5`; `$BU squat --cycles 2`; then `$BU pose default --hold 5 --policy-gains` | tracking/torques per joint. Asks: "both knees bent forward the same? feet flat? any buzzing at policy gains?" |
| **6** ROS, disarmed | hanging; IMU and gamepad connected | `$ROS launch cad` (default; cad_safe failed its sweep 2026-11-18), `$ROS check` repeatedly while they: hold still, tilt nose-down, tilt left side down, yaw left, push the stick | mode DISARMED; gravity (0,0,-1), nose-down x+, left-down y+, yaw-left gyro z+; joint_pos_rel in the zero pose = minus default; cmd_vel ~0 when centred. Fix `imu.rotation` if wrong (then relaunch) |
| **7** STAND | gantry slack, feet just above / on the floor | `$ROS status` (mode), ask them to press START, `$ROS check`, `$ROS log` | mode STAND, no "feedback late"/disarm in the log, tilt from gravity |
| **8** WALK | gantry catching | ask them to press A, drive slowly; `$ROS log` every so often; `$ROS stop` when done | mode WALK, warnings (feedback late = the 820 ms stall), any disarm reason. BACK disarms = robot drops into the gantry |
| **end** | | pull the yaml, session and logs back to the laptop (below) | |

## Push (step 0) and pull (end), from the laptop
```bash
cd /c/Users/karti/OneDrive/Documents/humanoids_sw/humanoid_ws_sysid
tar czf - --exclude='*.mp4' --exclude='__pycache__' tools/walker/bringup src/th_bringup/config/walker_demo_robot.yaml \
  src/th_bringup/config/gamepad_demo.yaml src/th_bringup/launch/robot.launch.py \
  policies/walker_demo | ssh $J 'cd ~/humanoid_ws && tar xzf - && chmod +x tools/walker/bringup/*.py tools/walker/bringup/ros.sh'
# end of session: bring back what the robot taught us
scp $J:~/humanoid_ws/src/th_bringup/config/{walker_demo_robot.yaml,bringup_session.json} src/th_bringup/config/
scp -r $J:~/humanoid_ws/bringup_logs ../sysid_data/2026-11-18_bringup_logs
```
The push overwrites the Jetson's `walker_demo_robot.yaml`. Once directions/offsets exist on the
Jetson, push only `tools/` and `policies/`, or pull the yaml first.
