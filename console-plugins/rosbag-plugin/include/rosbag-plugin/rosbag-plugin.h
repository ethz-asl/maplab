#ifndef ROSBAG_PLUGIN_ROSBAG_PLUGIN_H_
#define ROSBAG_PLUGIN_ROSBAG_PLUGIN_H_

#include <string>

#include <console-common/console-plugin-base-with-plotter.h>
#include <console-common/console.h>

namespace rosbag_plugin {

class RosbagPlugin : public common::ConsolePluginBaseWithPlotter {
 public:
  explicit RosbagPlugin(
      common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter);

  std::string getPluginId() const override {
    return "rosbag_plugin";
  }
};

struct Config {
  std::string input_rosbag_path = "";
  std::string output_rosbag_path = "";

  // Settings specifuc to export_pose_to_bag(p2b) command.
  std::string pose_to_bag_T_G_I_topic = "/T_G_I";
  std::string pose_to_bag_T_M_I_topic = "/T_M_I";
  std::string pose_to_bag_T_G_M_topic = "/T_G_M";
  std::string pose_to_bag_world_frame = "map";
  std::string pose_to_bag_odom_frame = "mission";
  std::string pose_to_bag_base_frame = "imu";
  bool pose_to_bag_add_tf = true;
  bool pose_to_bag_add_transform_stamped = true;
  float pose_to_bag_interpolate_to_frequency_hz =
      0.f;  // If set to 0, interpolation is disabled.
  bool rosbag_plugin_pose_to_bag_drop_tf = false;

  void getFromGflags();
};

}  // namespace rosbag_plugin

#endif  // ROSBAG_PLUGIN_ROSBAG_PLUGIN_H_
