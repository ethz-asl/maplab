#ifndef MAPLAB_ROS_COMMON_GFLAGS_INTERFACE_H_
#define MAPLAB_ROS_COMMON_GFLAGS_INTERFACE_H_

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

namespace ros_common {

void parseGflagsFromRosParams(
    const char* program_name, const ros::NodeHandle& nh_private);

}  // namespace ros_common

#endif  // MAPLAB_ROS_COMMON_GFLAGS_INTERFACE_H_
