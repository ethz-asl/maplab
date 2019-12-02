#include <atomic>
#include <memory>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/sigint-breaker.h>
#include <maplab-ros-common/gflags-interface.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "maplab-server-node/maplab-server-ros-node.h"

DEFINE_bool(
    map_save_on_shutdown, true,
    "Save the map on exit. If this is set to false, then the map must "
    "be saved using a service call.");

DEFINE_int32(
    map_visualization_interval_s, 10,
    "Visualize map every N seconds. Disabled if set ot 0.");

DECLARE_int32(map_save_every_n_sec);

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "maplab_server_node");
  ros::NodeHandle nh, nh_private("~");

  ros_common::parseGflagsFromRosParams(argv[0], nh_private);

  maplab::MaplabServerRosNode maplab_server_node(nh, nh_private);

  if (!maplab_server_node.start()) {
    ROS_FATAL("Failed to start running the maplab node!");
    ros::shutdown();
    return 0;
  }

  ros::Timer mav_visualization_timer;
  if (FLAGS_map_visualization_interval_s > 0) {
    mav_visualization_timer = nh.createTimer(
        ros::Duration(FLAGS_map_visualization_interval_s),
        [&maplab_server_node](const ros::TimerEvent& /*event*/) {
          maplab_server_node.visualizeMap();
        });
  }

  ros::waitForShutdown();

  mav_visualization_timer.stop();
  maplab_server_node.shutdown();

  if (FLAGS_map_save_on_shutdown) {
    maplab_server_node.saveMap();
  }
  return 0;
}
