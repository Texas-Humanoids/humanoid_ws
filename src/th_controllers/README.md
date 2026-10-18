# th_controllers

Custom ros2_control controllers, such as the one that runs our trained RL
walking policy.

`th_policy_integration/` contains the issue #18 observation builder that
connects gamepad velocity commands and live robot state to the policy node.
See its README for the topic contract and current hardware gaps.
