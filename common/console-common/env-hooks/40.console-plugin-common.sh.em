@[if DEVELSPACE]@
# Env variables in develspace.
export MAPLAB_CONSOLE_PLUGINS_LIBRARY_LIST="@(CATKIN_DEVEL_PREFIX)/@(CATKIN_GLOBAL_SHARE_DESTINATION)/console-plugins.txt"
@[else]@
# Env variables in installspace.
export MAPLAB_CONSOLE_PLUGINS_LIBRARY_LIST="$CATKIN_DEVEL_PREFIX/$CATKIN_GLOBAL_SHARE_DESTINATION/console-plugins.txt"
@[end if]@
