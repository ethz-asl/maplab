## Coding Examples: Creating a custom console plugin

In this tutorial, we will write a console plugin with a command to print a `Hello world` message and a command to interact with a VI map.
The code that is produced in this tutorial can be downloaded from [here](http://robotics.ethz.ch/~asl-datasets/maplab/coding_examples/hello-world-plugin.tar.gz).

### Creating a package for your plugin
First, create a new package:
```bash
cd ~/maplab_ws/src/
mkdir hello-world-plugin
cd hello-world-plugin
mkdir src
```

Create the following file and save it as `CMakeLists.txt`:
```cmake
cmake_minimum_required (VERSION 2.8)
project(hello_world_plugin)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-fPIC -shared)

cs_add_library(${PROJECT_NAME} src/hello-world-plugin.cc)
create_console_plugin(${PROJECT_NAME})

cs_install()
cs_export()
```

The important call here is `create_console_plugin` which ensures that this package will be listed as a console plugin.

Also create a file `package.xml` with the following contents:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<package format="2">
  <name>hello_world_plugin</name>
  <version>0.0.0</version>
  <description>A plugin to show how to add new console plugins</description>
  <maintainer email="your-email@domain.tld">Maplab user</maintainer>
  <license>Apache 2.0</license>

  <buildtool_depend>catkin_simple</buildtool_depend>
  <buildtool_depend>catkin</buildtool_depend>

  <depend>console_common</depend>
</package>
```

The console plugin needs to depend on the package `console_common`, everything else is free.

### Creating a plugin and adding commands
Finally, we can take care of the C++ code. Create a file `src/hello-world-plugin.cc`:
```cpp
#include <iostream>
#include <string>
#include <map-manager/map-manager.h>
#include <vi-map/vi-map.h>
#include <console-common/console.h>

// Your new plugin needs to derive from ConsolePluginBase.
// (Alternatively, you can derive from ConsolePluginBaseWithPlotter if you need
// RViz plotting abilities for your VI map.)
class HelloWorldPlugin : public common::ConsolePluginBase {
 public:
  // Every plugin needs to implement a getPluginId function which returns a
  // string that gives each plugin a unique name.
  std::string getPluginId() const override {
    return "hello_world_plugin";
  }

  // The constructor takes a pointer to the Console object which we can forward
  // to the constructor of ConsolePluginBase.
  HelloWorldPlugin(common::Console* console)
      : common::ConsolePluginBase(console) {
    // You can add your commands in here.
    addCommand(
        {"hello_world", "hello"},  // Map "hello_world" and "hello" to this
                                   // command.
        [this]() -> int {  // Function to call when this command is entered.
          // This function can do anything you want. Check the other plugins
          // under ~/maplab_ws/src/maplab/console-plugins for more examples.

          // Here, we just print a message to the terminal.
          std::cout << "Hello world!" << std::endl;

          // Every console command returns an integer, you can take one from
          // the CommandStatus enum. kSuccess returns everything is fine.
          // Other commonly used return values are common::kUnknownError and
          // common::kStupidUserError.
          return common::kSuccess;
        },

        // This is the description of your command. This will get printed when
        // you run `help` in the console.
        "This command prints \"Hello World!\" to the console.",

        // This specifies the execution method of your command. For most
        // commands, it is sufficient to run them in sync with
        // common::Processing::Sync.
        common::Processing::Sync);
  }
};

// Finally, call the MAPLAB_CREATE_CONSOLE_PLUGIN macro to create your console
// plugin.
MAPLAB_CREATE_CONSOLE_PLUGIN(HelloWorldPlugin);
```

### Interfacing with a VI map
Add the following dependencies to your `package.xml`:
```xml
  <depend>map_manager</depend>
  <depend>vi_map</depend>
```

In `src/hello-world-plugin.cc`, add the following includes:
```cpp
#include <string>
#include <map-manager/map-manager.h>
#include <vi-map/vi-map.h>
```

Then you can add your new command in the constructor of your previously created class:
```cpp
  HelloWorldPlugin(common::Console* console)
      : common::ConsolePluginBase(console) {

    ...

    addCommand(
        {"my_vi_map_command"},

        [this]() -> int {
          // Get the currently selected map.
          std::string selected_map_key;

          // This function will write the name of the selected map key into
          // selected_map_key. The function will return false and print an error
          // message if no map key is selected.
          if (!getSelectedMapKeyIfSet(&selected_map_key)) {
            return common::kStupidUserError;
          }

          // Create a map manager instance.
          vi_map::VIMapManager map_manager;

          // Get and lock the map which blocks all other access to the map.
          vi_map::VIMapManager::MapWriteAccess map =
              map_manager.getMapWriteAccess(selected_map_key);

          // Now run your algorithm on the VI map.
          // E.g., we can get the number of missions and print it.
          const size_t num_missions = map->numMissions();
          std::cout << "The VI map " << selected_map_key << " contains "
                    << num_missions << " missions." << std::endl;

          return common::kSuccess;
        },

        "This command will run an awesome VI map algorithm.",
        common::Processing::Sync);
  }
```

### Testing your plugin
After you have created your plugin, you can compile your plugin:
```
catkin build hello_world_plugin
```
and start maplab:
```
rosrun maplab_console maplab_console -v=1
```
With `-v=1`, you set the default verbosity level to 1. This is useful because this will print a list of all loaded plugins on program start, so you can check if your plugin is loaded correctly. You should see something like this in the output:
```
I1124 13:16:10.916294 26783 maplab-console.cc:100] RVIZ visualization initialized!
...
I1124 13:16:10.916895 26783 maplab-console.cc:126] Installed plugin hello_world_plugin from /home/user/catkin_ws/devel/lib/libhello_world_plugin.so.
...
```

Now, in the maplab console, you can check `help` to see a list of all installed plugins. Entering `help --plugin hello_world_plugin` will give you help for all commands in your new plugin:
```
maplab <>:/$ help --plugin hello_world_plugin 
Showing help for plugin hello_world_plugin.
  hello_world, hello
    This command prints "Hello World!" to the console.
  my_vi_map_command
    This command will run an awesome VI map algorithm.
```

You can also call your new command and see that it works as intended:
```
maplab <>:/$ hello_world 
Hello world!
```

Or if you have a map loaded, you can use the other command:
```
maplab <my_map>:/$ my_vi_map_command 
The VI map my_map contains 1 missions.
```

### Further reading
- You can now extend your package to contain arbitrary code in your commands. E.g., you may want to make a console command out of [this example](Coding-Examples%3A-Working-with-the-VI-Map).
- If you're interested to learn more about the inner workings of the plugin system, check out [this article](Console-plugin-system).
