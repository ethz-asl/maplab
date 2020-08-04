## Known Issues

Please also have a look at the [[FAQ]], there might be a solution to your issue there. We encourage you to contribute to this repository if you have a solution to these issues:

### ROS-free mode

The ROS-free launch of maplab (using the ``-ros_free`` flag) is currently not working.

### Building with unstable internet connection

Building maplab will download and check-out dependencies, which can be very slow depending on the speed of you internet connection. A more detailed answer can be found in the [FAQ](https://github.com/ethz-asl/maplab/wiki/FAQ#q-i-have-problems-building-the-opencv3-ceres-eigen-protobuf_catkin-package).

### Console crash when using gflags the wrong way

The console will crash if gflags (except `bool` gflags) are provided with no argument, e.g. `load --map_folder ` rather than `load --map_folder my_folder`.

### Using newer GCC versions

We observed that with higher GCC versions there can be build errors or even segfaults. E.g. this issue https://github.com/ethz-asl/maplab/issues/30. Try compiling with the default Ubuntu 14.04/16.04 versions of GCC, these are being tested by our build server.

### Very sparse VIMaps (low number of vertices)

If ROVIOLI produces extremely sparse (in terms of number of vertices) maps, make sure you are not trying to build a map from a multi-camera system where the individual cameras are not triggered at the same time. ROVIOLI will try to create VisualNFrames where all images have been taken at the same time, which might happen very rarely or not at all, hence, leading to sparse VIMaps.

Related Issues: [#49](https://github.com/ethz-asl/maplab/issues/49)

