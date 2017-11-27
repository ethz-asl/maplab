# Generated from loop-closure-handler/env-hooks/40.loopclosure.sh.em.
@[if DEVELSPACE]@
# Env variables in develspace.
export MAPLAB_LOOPCLOSURE_DIR="@(CATKIN_DEVEL_PREFIX)/share/loopclosure"
@[else]@
# Env variables in installspace.
export MAPLAB_LOOPCLOSURE_DIR="$CATKIN_ENV_HOOK_WORKSPACE/share/loopclosure"
@[end if]@