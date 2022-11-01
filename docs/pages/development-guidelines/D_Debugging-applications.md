## Debugging applications

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
Even more efficient is invoking ```make -j8``` in the build folder of the package
```
cd ~/maplab_ws/build/maplab && make -j8
```

### Compile in Release mode
Most packages are not built in Release mode. To build packages in Release mode:
```
catkin build maplab --cmake-args -DCMAKE_BUILD_TYPE=Release
```
To set this as the default behavior in your current catkin profile:
```
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Change individual packages to Debug mode
If you want to debug a program it makes sense to switch the package that the error is contained in to ```Debug``` mode.
```
cd ~/catkin_ws/build/maplab
ccmake .
```
If ``ccmake`` is not installed on your system, you have to install it using:
```
sudo apt-get install cmake-curses-gui
```
If ``ccmake`` is there, you will now see a window similar to this:
```
 BUILD_SHARED_LIBS                ON
 CATKIN_DEVEL_PREFIX              /Users/slynen/catkin_ws/devel
 CATKIN_ENABLE_TESTING            ON
 CATKIN_PACKAGE_PREFIX
 CATKIN_SKIP_TESTING              OFF
 CMAKE_BUILD_TYPE                 Release
 CMAKE_INSTALL_PREFIX             /Users/slynen/catkin_ws/install
 ...
```
Navigate to the line which says ```CMAKE_BUILD_TYPE``` using the cursor keys and press ```Enter```. Now replace ```Debug``` with ```Release```. Press ```c``` to configure, followed by ```g``` to generate. (you might now press ```e``` to hide warnings.). Finally press ```q``` to quit.
Now make the package by invoking:
```
cd ~/maplab_ws/build/maplab && make -j8
```

**HINT:** Don't forget to switch back to ```Release``` once you are done with debugging.

### Run gdb with arguments
In order to run a process with arguments in gdb, the most efficient way is to use:
```
gdb --ex run --args ~/devel/lib/maplab/maplab --use_external_memory=false
```
Where ```--use_external_memory=false``` is just an example flag.
