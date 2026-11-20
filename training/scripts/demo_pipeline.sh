#!/usr/bin/env bash
# One command per tuning iteration: train -> package -> MuJoCo sweep -> Isaac/PhysX sweep -> scorecard.
#
#   training/scripts/demo_pipeline.sh <task suffix> <name> [iterations]
#   training/scripts/demo_pipeline.sh Cad          cad_v2          # TH-Velocity-Flat-WalkerDemo-Cad
#   training/scripts/demo_pipeline.sh Cad-Safe     cad_safe_v2 3000
#   SKIP_TRAIN=1 RUN_NAME=v1 training/scripts/demo_pipeline.sh Cad cad_v1   # re-validate an existing run
#
# Output: humanoid_ws_sysid/policies/walker_demo/<name>/ (policy.onnx, policy_spec.json, sweeps, video)
# and the scorecard of every policy there. ~40 min training + ~3 min validation on the 5060 Ti.
# Run it from your own terminal (Claude Code background commands can be cut off).
set -euo pipefail
SUFFIX=$1; NAME=$2; ITERS=${3:-3000}; RUN_NAME=${RUN_NAME:-$NAME}
HERE=$(cd "$(dirname "$0")" && pwd)
TRAIN=$(dirname "$HERE")                                         # humanoid_ws/training
WS=$(cd "$TRAIN/../.." && pwd)                                   # humanoids_sw
OUT=$WS/humanoid_ws_sysid/policies/walker_demo/$NAME
RUNS=${RUNS:-/c/Users/karti/mjlab_runs}
MJ=${MJ_VENV:-/c/Users/karti/.venvs/mjlab}/Scripts/python.exe   # mjlab training venv
PY=${BHL_VENV:-/c/Users/karti/.venvs/bhl}/Scripts/python.exe    # MuJoCo + Isaac validation venv
export PYTHONIOENCODING=utf-8 PYTHONPATH=$TRAIN/src OMNI_KIT_ACCEPT_EULA=YES
cd "$RUNS"

if [ -z "${SKIP_TRAIN:-}" ]; then
  echo "== train TH-Velocity-Flat-WalkerDemo-$SUFFIX ($ITERS its) -> run name $NAME"
  "$MJ" -m th_training.scripts.train "TH-Velocity-Flat-WalkerDemo-$SUFFIX" --env.scene.num-envs 4096 \
    --agent.max-iterations "$ITERS" --agent.logger tensorboard --agent.run-name "$RUN_NAME" 2>&1 | tee "train_$NAME.log" \
    | grep --line-buffered -E "Learning iteration .*(00|99)/|Traceback|Error"
fi
EXP=walker_demo_$(echo "$SUFFIX" | tr 'A-Z-' 'a-z_')                  # Cad-Safe -> walker_demo_cad_safe
RUN=$(ls -d logs/rsl_rl/$EXP/*_"$RUN_NAME" | tail -1)
echo "== package $RUN -> $OUT"
mkdir -p "$OUT"
cp "$RUN"/*_"$RUN_NAME".onnx "$OUT/policy.onnx"
cp "$RUN/policy_spec.json" "$RUN/params/env.yaml" "$RUN/params/agent.yaml" "$OUT/"
{ ls "$RUN" | grep model_ | sort -t_ -k2 -n | tail -1; echo "$RUN"; } > "$OUT/checkpoint.txt"

echo "== MuJoCo sweep"
"$PY" -m th_training.sim2sim_demo sweep --onnx "$OUT/policy.onnx" --spec "$OUT/policy_spec.json" --out "$OUT/sim2sim_sweep.json" | tee "$OUT/sim2sim_sweep.txt"
"$PY" -m th_training.sim2sim_demo eval --onnx "$OUT/policy.onnx" --spec "$OUT/policy_spec.json" --video "$OUT/sim2sim_nominal.mp4" > "$OUT/sim2sim_nominal.json"
"$PY" -m th_training.used_ranges "$OUT"

echo "== Isaac/PhysX sweep"
cd "$WS"
"$PY" -m th_training.isaac_check_demo --onnx "$OUT/policy.onnx" --spec "$OUT/policy_spec.json" --out "$OUT/isaac_sweep.json" > "$OUT/isaac_sweep.log" 2>&1
"$PY" -m th_training.isaac_check_demo --onnx "$OUT/policy.onnx" --spec "$OUT/policy_spec.json" --friction 0.4 \
  --conditions nominal,combined --out "$OUT/isaac_mu04.json" > "$OUT/isaac_mu04.log" 2>&1

echo "== scorecard"
"$PY" -m th_training.compare_demo --md "$OUT/../SCORECARD.md"
