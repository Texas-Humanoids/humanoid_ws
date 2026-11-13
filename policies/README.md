# policies

Trained walking policies exported as `.onnx` files. They are trained in
[`../training/`](../training/); only the policies we actually deploy are
committed here, not training checkpoints or logs.

- `policy_humanoid.onnx`: Berkeley Humanoid Lite (bench tests; fetched by `tools/fetch_berkeley_policy.py`).
- [`walker_v2/v6/`](walker_v2/v6/README.md): WalkerV2 walking policy, the current hardware candidate.
