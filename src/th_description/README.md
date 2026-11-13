# th_description

The robot models: links, joints, joint limits, and meshes. Most other packages
read them (RViz, simulation, ros2_control, training).

Each robot gets its own folder under `robots/`:

| Folder | Robot | Status |
| --- | --- | --- |
| `robots/berkeley_bot/` | BerkeleyBot: the stock Berkeley Humanoid Lite | URDF + MJCF + meshes |
| `robots/walker_v2/` | WalkerV2: 10-DOF biped (RobStride RS02/RS03/RS06) | URDF + MJCF + meshes (see its README) |
| `robots/jank_bot/` | JankBot | placeholder |
| `robots/beta_bot/` | BetaBot | placeholder |
| `robots/bevo_bot/` | BevoBot | placeholder |

Inside a robot folder, use the same layout as `berkeley_bot/`:

```
<robot>/
├── urdf/      for ROS (RViz, ros2_control)
├── mjcf/      for MuJoCo / mjlab
└── meshes/    shared by both
```
