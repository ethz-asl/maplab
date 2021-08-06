#include <atomic>
#include <memory>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-ros-common/gflags-interface.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "maplab-node/maplab-ros-node.h"

DEFINE_bool(
    map_save_on_shutdown, true,
    "Save the map on exit. If this is set to false, then the map must "
    "be saved using a service call.");

DECLARE_int32(map_save_every_n_sec);

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "maplab_node");
  ros::NodeHandle nh, nh_private("~");

  // Initialize singleton and parse the current rosparams.
  ros_common::parserInstance<ros_common::GflagsParser>(argv[0]);
  if (!ros_common::parserInstance<ros_common::GflagsParser>()
           .parseFromRosParams(nh_private)) {
    LOG(ERROR) << "Unable to set up Gflags using rosparams! "
               << "Using default parameters.";
  }

  maplab::MaplabRosNode maplab_node(nh, nh_private);
  if (!maplab_node.run()) {
    ROS_FATAL("Failed to start running the maplab node!");
    ros::shutdown();
    return 0;
  }

  ros::Timer map_save_timer;
  if (FLAGS_map_save_every_n_sec > 0) {
    map_save_timer = nh.createTimer(
        ros::Duration(FLAGS_map_save_every_n_sec),
        [&maplab_node](const ros::TimerEvent& /*event*/) {
          maplab_node.saveMapAndContinueMapping();
        });
  }

  ros::Rate rate(1);
  while (ros::ok() && !maplab_node.isDataSourceExhausted()) {
    VLOG_EVERY_N(1, 20) << "\n" << maplab_node.printDeliveryQueueStatistics();
    ros::spinOnce();
    rate.sleep();
  }
  map_save_timer.stop();

  maplab_node.shutdown();
  if (FLAGS_map_save_on_shutdown) {
    maplab_node.saveMapAndFinishMapping();
  }
  return 0;
}
