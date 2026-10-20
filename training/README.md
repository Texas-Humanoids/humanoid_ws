# training

Trains our walking policies in simulation, then exports them as `.onnx` files
to [`../policies/`](../policies/).

We're trying two simulators side by side to see which one we prefer:

| Folder | Simulator | Runs on |
| --- | --- | --- |
| [`mjlab/`](mjlab/) | [mjlab](https://github.com/mujocolab/mjlab) (MuJoCo Warp) | Linux + NVIDIA GPU; macOS CPU for small tests |
| [`isaaclab/`](isaaclab/) | [Isaac Lab](https://isaac-sim.github.io/IsaacLab/) 2.3 (Isaac Sim 5.1) | Linux + NVIDIA RTX GPU only |

Each folder is its own Python project managed with
[uv](https://docs.astral.sh/uv/), with its own virtual environment. They have
to be separate because the two simulators need different versions of torch
and other packages.

None of this is a ROS package. The `COLCON_IGNORE` file here tells
`colcon build` to skip the whole folder.

## Why it lives in this repo

The policy and the controller that runs it on the robot (`th_controllers`)
must agree on joint order, observations, action scaling, default pose, and
gains. Keeping training here means a change to any of those updates both
sides in the same pull request.

Both simulators will load the robot from `../src/th_description`, not from
their own copies, so they're always training the same robot.

Training outputs (`logs/`, checkpoints, `wandb/`) are ignored by git. Only the
policies we actually deploy get committed, to `../policies/`.
