@[if DEVELSPACE]@
# Env variables in develspace.
export SIMULATION_SHARE_DIR="@(CATKIN_DEVEL_PREFIX)/share/simulation"
@[else]@
# Env variables in installspace.
export SIMULATION_SHARE_DIR="$CATKIN_ENV_HOOK_WORKSPACE/share/simulation"
@[end if]@
