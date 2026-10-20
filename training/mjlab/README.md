# training/mjlab

Policy training with [mjlab](https://github.com/mujocolab/mjlab), pinned to
1.6.0. See [`../README.md`](../README.md) for how this fits with the other
simulator.

## Setup

```bash
cd training/mjlab
uv sync
uv run python -c "import mjlab, th_mjlab"
```

macOS works for small CPU experiments. Real training runs should use Linux
with an NVIDIA GPU.

## What will go here

- `src/th_mjlab/`: our robot configs and task environments (standing,
  velocity tracking), plus ONNX export
- `tests/`: quick CPU checks that the environments build and step
