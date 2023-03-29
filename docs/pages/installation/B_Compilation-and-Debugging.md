## Compilation and Debugging

### Use ccache
ccache is a fast compiler cache. It is a program that sits in front of gcc and monitors what is being compiled. If a file was compiled before in the exact same state, then ccache will serve a compilation request from cache and thus lead to "instant" compilation. Check the install section page for instructions.

### Compile the package you need
Avoid running ```catkin build``` without specifying a package since this builds the entire workspace which takes a long time.
You can specify individual packages simply by adding their names:
```
catkin build maplab
```

### Build single packages efficiently
If you only changed a single package you can also restrict the build process to this single package:
```
catkin build maplab --no-deps
```
However, depending on the changes you've made, you might get a SEGFAULT when running. This can happen especially when changing headers that are included elsewhere. If you get a crash after using `--no-deps`, try first recompiling everything with `catkin build maplab`.

### Compile in Release/Debug mode
Most packages are not built in Release mode. To build packages in Release mode:
```
catkin build maplab --cmake-args -DCMAKE_BUILD_TYPE=Release
```
To set this as the default behavior in your current catkin profile:
```
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```
If you are actively developing on maplab, debug symbols will sooner or later be helpful, therefore, we recommend to include them when building. This flag will enable all the optimizations of the release mode, but include debug symbols to give the debugger a change to figure out where in the code the program currently is:
```
-DCMAKE_BUILD_TYPE=RelWithDebInfo
```
In order to compile all packages in debug mode, i.e. no optimizations plus debug symbols, use:
```
-DCMAKE_BUILD_TYPE=Debug
```
**Important:** Switching these modes does not automatically rebuild packages that have been build before, therefore, clean the workspace first to be sure the new mode is adopted by all packages.
```
catkin clean --yes
```

### Change individual packages to Debug mode
If you want to efficiently debug a program it makes sense to switch only the package that the error is contained in to ```Debug``` mode.
```
cd ~/catkin_ws/build/maplab
ccmake .
```
You will now see a window similar to this:
```
 BUILD_SHARED_LIBS                ON
 CATKIN_DEVEL_PREFIX              /Users/sheep/catkin_ws/devel
 CATKIN_ENABLE_TESTING            ON
 CATKIN_PACKAGE_PREFIX
 CATKIN_SKIP_TESTING              OFF
 CMAKE_BUILD_TYPE                 Release
 CMAKE_INSTALL_PREFIX             /Users/sheep/catkin_ws/install
 ...
```
Navigate to the line which says ```CMAKE_BUILD_TYPE``` using the cursor keys and press ```Enter```. Now replace ```Debug``` with ```Release```. Press ```c``` to configure, followed by ```g``` to generate. (you might now press ```e``` to hide warnings.). Finally press ```q``` to quit.
Now make the package by invoking:
```
cd ~/catkin_ws/build/maplab && make -j8
```

**HINT:** Don't forget to switch back to ```Release``` once you are done with debugging.

### Run gdb with arguments
In order to run a process with arguments in gdb, the most efficient way is to use:
```
gdb --ex run --args ~/devel/lib/maplab/maplab --use_external_memory=false
```
Where ```--use_external_memory=false``` is just an example flag.

For adding gdb or valgrind to ROS launch files see examples [here](http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20Nodes%20in%20Valgrind%20or%20GDB).
