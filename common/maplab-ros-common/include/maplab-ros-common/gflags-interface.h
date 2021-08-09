#ifndef MAPLAB_ROS_COMMON_GFLAGS_INTERFACE_H_
#define MAPLAB_ROS_COMMON_GFLAGS_INTERFACE_H_

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

namespace ros_common {

class GflagsParser {
 public:
  explicit GflagsParser(const char* program_name);
  bool parseFromRosParams(const ros::NodeHandle& nh_private) const;

 private:
  const char* program_name_;
};

template <typename T_parser>
T_parser parserInstance(const char* program_name = nullptr) {
  static thread_local T_parser instance(program_name);
  return instance;
}

bool parseGflagsFromRosParams(
    const char* program_name, const ros::NodeHandle& nh_private);

}  // namespace ros_common

#endif  // MAPLAB_ROS_COMMON_GFLAGS_INTERFACE_H_
