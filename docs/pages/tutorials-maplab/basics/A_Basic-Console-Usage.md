## Basic Console Usage

Install [maplab](Installation-Ubuntu) and make sure to compile the console:
```sh
cd ~/maplab_ws
catkin build maplab
```

Make sure that your development environment is correctly sourced:
```sh
source ~/maplab_ws/devel/setup.bash
```

You can now launch the maplab console with
```sh
rosrun maplab_console maplab_console
```

This command requires a running roscore for visualization.
If the program gets stuck looking for the roscore, you can abort the search with Ctrl-C.
(You can start a roscore with with `roscore&`.)

If you want to start maplab in a ros-free environment, you can use the `ros_free` flag:
```sh
rosrun maplab_console maplab_console --ros_free
```

You should now be greeted with the maplab console prompt:

![Console Prompt](images/maplab-console-prompt.png)

The console supports tab completion and a command history, which can be accessed with the arrow keys or Ctrl-P and Ctrl-N.

### Getting Help
To get information about the available commands, you can use the `help` command.
When no argument is provided, `help` will list all installed plugins and tell you how to get more help.

![Help](images/maplab-help.png)

- Use `help --all` to get a description of all available commands.
- Use `help --plugin <plugin_name>` to get a description of the commands provided by the plugin `plugin_name`.
E.g., type `help --plugin console` to see the commands provided by the `console` plugin:

    ![Help for Plugin](images/maplab-help-plugin.png)
- Use `help --command_filter <search_term>` to search all command names and descriptions for `search_term`.
You can optionally add the `plugin` flag to limit the search to commands provied by a given plugin.

### Exiting maplab
To exit maplab, you can type `q` or `exit`.
