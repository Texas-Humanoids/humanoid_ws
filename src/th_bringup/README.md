# th_bringup

Workspace-level launch files for bringing up robot software.

## Xbox teleoperation

Build and source the workspace, connect the Xbox controller, then run:

```bash
ros2 launch th_bringup xbox_teleop.launch.py
```

This starts `joy_node` and `gamepad_node`, which publish `/joy` and `/cmd_vel`.
Hold LB to enable the Xbox command, as described in the `gamepad_node` README.
To check the controls without a controller, run:

```bash
ros2 launch th_bringup xbox_teleop.launch.py input_mode:=keyboard
```

The combined launch can also start policy inference when explicitly requested:

```bash
ros2 launch th_bringup xbox_teleop.launch.py start_policy:=true model_path:=/absolute/path/to/policy.onnx
```

Policy inference still requires a publisher of the complete 75-value
`/policy/observation` message. The gamepad's `/cmd_vel` is not converted into
that observation by this launch file.

Starting the motor node is a separate, explicit opt-in because it enables the
connected RS06 motor during node startup:

```bash
ros2 launch th_bringup xbox_teleop.launch.py start_motor:=true motor_id:=1
```

That starts the motor node but does **not** connect `/cmd_vel` or policy output
to the motor. The policy publishes 22 joint positions while this motor node
accepts one scalar target. Do not enable the motor until the correct motor ID,
mounting, limits, physical cutoff, and operator setup have been verified.
