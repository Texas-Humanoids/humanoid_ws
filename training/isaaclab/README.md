# training/isaaclab

Policy training with [Isaac Lab](https://isaac-sim.github.io/IsaacLab/)
2.3.2, the latest generally available release, on Isaac Sim 5.1. See
[`../README.md`](../README.md) for how this fits with the other simulator.

## Requirements

From the [Isaac Sim 5.1 requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html):

- Ubuntu 22.04 or 24.04, x86_64. **There is no macOS support.**
- At least a GeForce RTX 4080 (16 GB VRAM) and 32 GB RAM
- NVIDIA driver 580.65.06 (the version Isaac Sim 5.1 was tested on)

## Setup

```bash
cd training/isaaclab
uv sync
```

This is a large download. Then check that it works:

```bash
uv run isaacsim     # opens the Isaac Sim window
uv run python -c "from isaaclab.app import AppLauncher; AppLauncher(headless=True).app.close()"
```

The first launch can take over 10 minutes while Isaac Sim downloads its
extensions. It also asks you to accept the NVIDIA Omniverse license (EULA):
read it and accept it yourself.
Don't set `OMNI_KIT_ACCEPT_EULA` in shared config to skip the prompt for
others.

## Comparing against mjlab

Isaac Lab 2.3 uses PhysX, mjlab uses MuJoCo Warp, so a policy that works in
both is more likely to work on the real robot. Keep in mind that they also
use different major versions of RSL-RL (3.0 here, 5.4 in mjlab), so training
curves won't be directly comparable.

## Notes on `pyproject.toml`

The lockfile is Linux x86_64 only. Two settings are explained in comments
there: a release-date cutoff (Isaac Lab 2.3.2 doesn't cap most of its
dependencies), and a `pywin32` override (NVIDIA's PyPI stub for
`isaacsim-core` declares it for every platform).

## What will go here

- `src/th_isaaclab/`: our robot configs and tasks, matching the mjlab ones,
  plus our own train and play scripts. The pip package doesn't include
  Isaac Lab's scripts; those live in its GitHub repo.
