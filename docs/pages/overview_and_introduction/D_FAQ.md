## FAQ

**Content:**
* [Installation](#installation)
* [Sensors](#sensors)
* [ROVIOLI](#rovioli)
* [Optimization](#optimization)
* [Miscellaneous](#miscellaneous)


### Installation

#### Q: Why do I get missing dependencies when building the maplab workspace?

Some of our current dependencies include further catkin packages, which are not a dependency of maplab. If you build the complete workspace, these packages will fail due to missing dependencies of their own. This might change in the future, but for now there are two options:

Please only build maplab: `catkin build maplab`

**or**

Add `CATKIN_IGNORE` packages to the superfluous packages.

#### Q: Why are the dependencies in a separate repository and not submodules in maplab?

The decision to move the dependencies to a separate repository was motivated by the fact that someone might want to integrate maplab in their existing catkin workspace. This will require some additional effort, however, having an external dependencies-repository should make it easier to combine the maplab dependencies with the dependencies of the existing workspace.

#### Q: Compilation failed because protoc was not found.

If you get an issue similar to this:
```
make[2]: /home/user/maplab_ws/devel/.private/maplab_common/bin/protoc: Command not found
make[2]: *** [compiled_proto/maplab-common/id.pb.cc] Error 127
make[2]: *** Waiting for unfinished jobs....
```

Make sure that your workspace layout is `merge`:
```bash
cd ~/$CATKIN_WS
catkin clean  # Necessary to clean up your workspace as your layout will change.
catkin config --merge-devel
``` 

You can check if your workspace is properly set up by running:
```bash
$ catkin config
---------------------------------------------------------------------------
Profile:                     default
Extending:        [explicit] /opt/ros/kinetic
Workspace:                   /home/user/maplab_ws
---------------------------------------------------------------------------
...
---------------------------------------------------------------------------
Devel Space Layout:          merged    # <-- This should say `merged`.
Install Space Layout:        None
---------------------------------------------------------------------------
...
```

#### Q: Why is my opencv3_catkin build freezing?

A known cause for this is an installed MATLAB using a network licence. The build can stall on getting the licence authenticated; so make sure that the licence server can be accessed, connect to the VPN if required.

#### Q: I get an error for libGL.so with Nvidia drivers
```
make[5]: *** No rule to make target '/usr/lib/x86_64-linux-gnu/libGL.so', needed by 'lib/libopencv_viz.so.3.2.0'.  Stop.
```
consider redefining the symbolic link that somehow got lost in your Ubuntu by:
```
sudo rm /usr/lib/x86_64-linux-gnu/libGL.so
sudo ln -s /usr/lib/libGL.so.1 /usr/lib/x86_64-linux-gnu/libGL.so
```

#### Q: ImportError: No module named 'requests'

Install requests by calling:
```
sudo pip install requests
```

#### Q: Why are even some system dependencies wrapped into catkin packages? (eigen, protobuf, glog, opencv, ...)
Since we support both Ubuntu 14.04 and 16.04 and even have some OSX users (no official support), the library versions provided by the system are varying too much and made developing compatible code a very tedious business. That's why we wrapped some of the system dependencies into catkin packages to gain full control over these libraries.


#### Q: I have problems building the {opencv3, ceres, eigen, protobuf,...}_catkin package.

Most catkinized dependencies will first download an archive or checkout a git repository.
If your internet connection is unstable or very slow these dependencies might fail to build.

If you are unable to get access to a stable internet connection, you can try to manually download the zip files/checkout the git repositories and copy them in the appropriate folder. You will find the download/git link and the target folder in the `CMakeLists.txt` file in the `ExternalProject_Add` block. 

**Example - `opencv3_catkin`:**
```cmake
SET(OPENCV_SRC_PATH "opencv3_src")
ExternalProject_Add(opencv3_src
  DEPENDS ${CONTRIB_NAME}
  URL https://github.com/Itseez/opencv/archive/3.2.0.zip  # <========= LINK TO ARCHIVE
  URL_MD5 bfc6a261eb069b709bcfe7e363ef5899
  UPDATE_COMMAND ""
  SOURCE_DIR ${OPENCV_SRC_PATH}                           # <========= TARGET FOLDER
  BINARY_DIR opencv3_build
  PATCH_COMMAND patch -p0 < ${CMAKE_SOURCE_DIR}/fix_python_discovery.patch
  CONFIGURE_COMMAND cd ../${OPENCV_SRC_PATH} && cmake
```
 * Download the archive from: `https://github.com/Itseez/opencv/archive/3.2.0.zip`
 * Extract the opencv folder to: `~/maplab_ws/build/opencv3_catkin/opencv3_src`

#### Q: static assertion failed: Voxblox 3D points and maplab 3D points are not the same!
It seems that you have checked out an incorrect commit of Voxblox. Maplab requires a specific commit that guarantees compatibility. 

Please make sure you have followed the installation instructions, specifically the line:
```
https://github.com/ethz-asl/maplab/wiki/Installation-Ubuntu
```
This should clone the correct versions of all the dependencies. 

Alternatively, you can go to maplab dependencies repository:
https://github.com/ethz-asl/maplab_dependencies
and make sure your local commits of dependencies adhere to the ones in ``master`` branch.


### Sensors

#### Q: What sensors are recommended for maplab?

We are supporting the Skybotix VI-Sensor, Google Tango tablets and the Intel Realsense ZR300 by default and provide example calibrations for them [here](https://github.com/ethz-asl/maplab/tree/master/applications/rovioli/share) and [here](https://github.com/ethz-asl/maplab/wiki/Intel-Realsense-ZR300).

Maplab focuses on visual-inertial mapping, therefore, the minimum requirements are: a gray-scale (preferably fish-eye) camera with global shutter and an IMU with similar or better characteristics as the ones in the sensors above (e.g. ADIS16488 or BMX055). The second critical component is a solid time synchronization between camera and IMU, i.e. ideally both devices are triggered based on the same clock. And finally, you will need to properly calibrate your sensor as described [here](https://github.com/ethz-asl/maplab/wiki/Initial-sensor-calibration-with-Kalibr) and [here](https://github.com/ethz-asl/maplab/wiki/Sensor-calibration-refinement).

Please also have a look at the answer to the issue(s) below.
Related issues: [#19](https://github.com/ethz-asl/maplab/issues/19)

#### Q: How can I use Tango devices to create rosbags?

Use the [Tango ROS Streamer app](http://wiki.ros.org/tango_ros_streamer) to publish the sensor data to ROS and record a rosbag.

#### Q: How can I use the Intel Realsense ZR300?

We've tested this sensor and once properly calibrated, it works well with maplab. We provide a simple ROS node to use the sensor in this repository ([maplab_realsense](https://github.com/ethz-asl/maplab_realsense)). For an example calibration and more details, see the [ZR300 wiki page](https://github.com/ethz-asl/maplab/wiki/Intel-Realsense-ZR300). Related issues: [#32](https://github.com/ethz-asl/maplab/issues/32)

### ROVIOLI

#### Q: How can I make ROVIOLI stop appending "image_raw" to the image topic?

Use the `--vio_camera_topic_suffix` flag to change or remove the suffix when running ROVIOLI:

```bash
--vio_camera_topic_suffix="" # To remove the suffix completely
```

#### Q: How can I disable the graphical output of ROVIOLI to run it on a robot/server?

Use the `--rovio_enable_frame_visualization=false` flag to disable the output.


### Optimization

#### Q: My map only contains very few landmarks. Why?

Most of the commands (such as optimization, loop-closure, visualization, etc.) only work on landmarks that are flagged as 'good'. The default parameters for these quality metrics might not be optimal for your use-case. Have a look at the following page and check if some parameter need to be adapted for your use-case:
https://github.com/ethz-asl/maplab/wiki/Optimizing-VI-Maps

### Miscellaneous

#### Q: Why the sheep?

That's why:

<img src="https://github.com/ethz-asl/maplab/wiki/images/cool_sheep.gif" width="400">
