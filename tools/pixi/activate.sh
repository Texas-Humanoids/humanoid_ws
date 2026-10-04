# Sourced by pixi in every `pixi run` and `pixi shell`. ROS itself is already
# set up by the environment; this adds our workspace once it has been built.
if [ -f "$PIXI_PROJECT_ROOT/install/local_setup.sh" ]; then
  . "$PIXI_PROJECT_ROOT/install/local_setup.sh"
fi
