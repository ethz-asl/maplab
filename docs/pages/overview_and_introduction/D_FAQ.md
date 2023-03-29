## FAQ

**Content:**
* [Installation](#installation)
* [Sensors](#sensors)
* [Map Building](#map-building)
* [Optimization](#optimization)
* [Miscellaneous](#miscellaneous)


### Installation

#### Q: Why do I get missing dependencies when building the maplab workspace?

Some of our current dependencies include further catkin packages, which are not a dependency of maplab. If you build the complete workspace, these packages will fail due to missing dependencies of their own. This might change in the future, but for now there are two options:

Please only build maplab: `catkin build maplab`

**or**

Add `CATKIN_IGNORE` packages to the superfluous packages.

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
Extending:        [explicit] /opt/ros/noetic
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
pip install requests
```

#### Q: Why are even some system dependencies wrapped into catkin packages? (eigen, protobuf, glog, opencv, ...)
To support multiple OS versions and facilitate compiling everything. The library versions provided by the system vary too much and makes developing compatible code a very tedious business. That's why we wrapped some of the system dependencies into catkin packages to gain full control over these libraries.


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
git clone git@github.com:ethz-asl/maplab.git --recursive
```
This should clone the correct versions of all the dependencies.


### Sensors

#### Q: What sensors are recommended for maplab?

For visual-inertial sensors calibrations can then be obtained using [Kalibr](https://github.com/ethz-asl/kalibr). For sensors with factory calibrations that are not supported by maplab (e.g. Kinect with 8 polynomial distortion coefficients), the images can be undistorted beforehand for example using [image_undistort](https://github.com/ethz-asl/image_undistort). In this case the distortion model should have only zeros and the new camera parameters should be copied over after undistortion.

For visual-inertial mapping the minimum requirements are: a gray-scale (preferably fish-eye) camera with global shutter and an IMU. The second critical component is a solid time synchronization between camera and IMU, i.e. ideally both devices are triggered based on the same clock.

### Map Building

#### Q: How can I make maplab stop appending "image_raw" to the image topic?

Use the `--vio_camera_topic_suffix` flag to change or remove the suffix when running ROVIOLI:

```bash
--vio_camera_topic_suffix="" # To remove the suffix completely
```

#### Q: How can I disable the graphical output of ROVIOLI to run it on a robot/server?

Use the `--rovio_enable_frame_visualization=false` flag to disable the output.

#### Q: How do I include every image as a vertex?

By default maplab and ROVIOLI limit the vertex frequncy to 10Hz. You can increase this using the flag `--vio_nframe_sync_max_output_frequency_hz`. This will enable obtaining an exact pose for every image frame. If you want higher frequency (i.e. IMU frequency) when exporting poses using `export_trajectory_to_csv` you can use the flag `--interpolate_to_imu_timestamps=true`.

#### Q: Why are is my map very sparse (low number of vertices)?

If maplab produces extremely sparse (in terms of number of vertices) maps, make sure you are not trying to build a map from a multi-camera system where the individual cameras are not triggered at the same time. Maplab will try to create VisualNFrames where all images have been taken at the same time, which might happen very rarely or not at all, hence, leading to sparse maps. Use the flag `--vio_nframe_sync_tolerance_ns` to increase the tolerance threshold.


### Optimization

#### Q: My map only contains very few landmarks. Why?

Most of the commands (such as optimization, loop-closure, visualization, etc.) only work on landmarks that are flagged as 'good'. The default parameters for these quality metrics might not be optimal for your use-case. Have a look at the following page on landmark quality filtering [TODO: Add link]().

Not seeing any sort of logical structure in the landmarks and bad convergence can often be a sign of a poor calibration or sensor. Make sure the time synchronization is proprly set up and that the transformation matrix between camera and IMU is not inverted for example.

### Miscellaneous

#### Q: Why the sheep?

That's why:

<img src="https://raw.githubusercontent.com/ethz-asl/maplab/master/docs/pages/images/cool_sheep.gif" width="400">
