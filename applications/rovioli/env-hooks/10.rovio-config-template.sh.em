@[if DEVELSPACE]@
# Env variables in develspace.
export ROVIO_CONFIG_DIR="@(CMAKE_CURRENT_SOURCE_DIR)/share/"
@[else]@
# Env variables in installspace.
export ROVIO_CONFIG_DIR="$CATKIN_ENV_HOOK_WORKSPACE/share/"
@[end if]@