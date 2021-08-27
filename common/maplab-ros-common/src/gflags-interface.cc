#include "maplab-ros-common/gflags-interface.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

namespace ros_common {

bool parseGflagsFromRosParams(
    const char* program_name, const ros::NodeHandle& nh_private) {
  std::vector<google::CommandLineFlagInfo> flags;
  google::GetAllFlags(&flags);
  LOG(INFO) << "Parsing gflags from ROS params...";

  std::stringstream ss;

  for (google::CommandLineFlagInfo& flag : flags) {
    if (flag.type == "int32" || flag.type == "uint64" || flag.type == "int64") {
      int32_t ros_param_value;
      if (nh_private.getParam(flag.name, ros_param_value)) {
        ss << "--" << flag.name << "=" << ros_param_value << std::endl;
      }
    } else if (flag.type == "bool") {
      bool ros_param_value;
      if (nh_private.getParam(flag.name, ros_param_value)) {
        ss << "--" << flag.name << "=" << ros_param_value << std::endl;
      }
    } else if (flag.type == "string") {
      std::string ros_param_value;
      if (nh_private.getParam(flag.name, ros_param_value)) {
        if (ros_param_value.find_first_of("\t\n ") != std::string::npos) {
          LOG(WARNING) << "Cannot parse ROS string parameter '" << flag.name
                       << "' as gflag because it contains whitespaces! value: '"
                       << ros_param_value << "'";
        } else {
          ss << "--" << flag.name << "=" << ros_param_value << std::endl;
        }
      }
    } else if (flag.type == "double") {
      double ros_param_value;
      if (nh_private.getParam(flag.name, ros_param_value)) {
        ss << "--" << flag.name << "=" << ros_param_value << std::endl;
      }
    } else {
      LOG(WARNING) << "Cannot parse gflag '" << flag.name
                   << "' from ROS params, type " << flag.type
                   << " is not supported!";
    }
  }
  const std::string ros_param_gflag_file = ss.str();
  if (!google::ReadFlagsFromString(
          ros_param_gflag_file, program_name, false /*errors_are_fatal*/)) {
    return false;
  }
  LOG(INFO) << "\n\nThe following Gflags have been set using ROS params:\n"
            << ros_param_gflag_file << "\n";
  return true;
}

GflagsParser::GflagsParser(const char* program_name)
    : program_name_(program_name) {}

bool GflagsParser::parseFromRosParams(const ros::NodeHandle& nh_private) const {
  return parseGflagsFromRosParams(program_name_, nh_private);
}

}  // namespace ros_common
