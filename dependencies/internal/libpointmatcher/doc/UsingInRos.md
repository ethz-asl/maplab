| [Tutorials Home](index.md)    | [Previous](LinkingProjects.md) | [Next](Pointclouds.md) |
| ------------- |:-------------:| -----:|

# Using libpointmatcher in ROS

If you want to use libpointmatcher in [ROS](http://www.ros.org/), you can use the [ethzasl_icp_mapping](https://github.com/ethz-asl/ethzasl_icp_mapping) stack. It allows the conversion of pointclouds from ROS message formats to a libpointmatcher-compatible format and provides a mapping node that is already functionnal and that can be customized using YAML configuration files to suite your needs. It also provides an odometry estimation based on the results of the pointcloud registrations. It includes a wide range of launch files that can be used as usage examples of the different nodes of the stack. Finally, it is recommended to install it from sources rather than from apt-get, because the last release was a long time ago, and it might not be working.
