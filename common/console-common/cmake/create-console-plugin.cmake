macro(create_console_plugin LIBRARY_NAME)
  # dlopen needs the full name (including lib and .so) to be able to open the
  # library. A path is not necessary, as the library will be searched for in
  # the paths given by LD_LIBRARY_PATH (set by ROS to include the catkin
  # workspace and ros distro lib folders) and other paths. See ld.so(8) for more
  # details.
  SET(LIBRARY_FILE_NAME "${CMAKE_SHARED_LIBRARY_PREFIX}${LIBRARY_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}")
  FILE(WRITE "${CMAKE_BINARY_DIR}/51.maplab-console-${LIBRARY_NAME}.sh"
       "export MAPLAB_CONSOLE_PLUGINS=\"${LIBRARY_FILE_NAME};$MAPLAB_CONSOLE_PLUGINS\"\n")
  catkin_add_env_hooks(51.maplab-console-${LIBRARY_NAME}
                       SHELLS sh
                       DIRECTORY ${CMAKE_BINARY_DIR})

  message(STATUS "Added library \"${LIBRARY_NAME}\" to list in \"${PLUGIN_LIST_FILE_PATH}\".")
endmacro()
