#include "rosbag-plugin/rosbag-plugin.h"

#include <chrono>
#include <cstring>
#include <string>
#include <vector>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <aslam/common/timer.h>
#include <console-common/console.h>
#include <gflags/gflags.h>
#include <landmark-triangulation/pose-interpolator.h>
#include <map-manager/map-manager.h>
#include <maplab-common/file-system-tools.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

DECLARE_string(map_mission_list);
DECLARE_bool(overwrite);

DEFINE_string(rosbag_plugin_input_bag, "", "Path to rosbag to be parsed.");

DEFINE_string(rosbag_plugin_output_bag, "", "Path to rosbag to be written.");

DEFINE_string(
    rosbag_plugin_pose_to_bag_world_frame, "map",
    "Name of the global/world frame.");
DEFINE_string(
    rosbag_plugin_pose_to_bag_odom_frame, "mission",
    "Name of the drifting odometry frame.");
DEFINE_string(
    rosbag_plugin_pose_to_bag_base_frame, "imu", "Name of the base frame.");
DEFINE_bool(
    rosbag_plugin_pose_to_bag_add_tf, true,
    "If enabled, the poses of the VIMap will be added to the tf tree of the "
    "rosbag.");
DEFINE_bool(
    rosbag_plugin_pose_to_bag_add_transform_stamped, true,
    "If enabled, the poses of the VIMap will be added to the bag as "
    "TransformStamped.");
DEFINE_double(
    rosbag_plugin_pose_to_bag_interpolate_to_frequency_hz, 100,
    "Frequency at which we want the poses. Will only have an effect if set "
    "higher than the current frequency of the vertices in the map.");
DEFINE_bool(
    rosbag_plugin_pose_to_bag_drop_tf, false,
    "If enabled, original tf tree will not be added to the new rosbag.");

namespace rosbag_plugin {

void Config::getFromGflags() {
  input_rosbag_path = FLAGS_rosbag_plugin_input_bag;
  output_rosbag_path = FLAGS_rosbag_plugin_output_bag;

  // Settings specifuc to export_pose_to_bag(p2b) command.
  pose_to_bag_world_frame = FLAGS_rosbag_plugin_pose_to_bag_world_frame;
  pose_to_bag_odom_frame = FLAGS_rosbag_plugin_pose_to_bag_odom_frame;
  pose_to_bag_base_frame = FLAGS_rosbag_plugin_pose_to_bag_base_frame;
  pose_to_bag_add_tf = FLAGS_rosbag_plugin_pose_to_bag_add_tf;
  pose_to_bag_add_transform_stamped =
      FLAGS_rosbag_plugin_pose_to_bag_add_transform_stamped;
  pose_to_bag_interpolate_to_frequency_hz =
      FLAGS_rosbag_plugin_pose_to_bag_interpolate_to_frequency_hz;
  rosbag_plugin_pose_to_bag_drop_tf = FLAGS_rosbag_plugin_pose_to_bag_drop_tf;
}

common::CommandStatus exportPosesToRosbag(
    const Config& config, const vi_map::VIMap& vi_map) {
  if (config.input_rosbag_path.empty()) {
    LOG(ERROR) << "No input rosbag path provided, use "
                  "--rosbag_plugin_input_bag.";
    return common::kStupidUserError;
  }
  if (config.output_rosbag_path.empty()) {
    LOG(ERROR) << "No output rosbag path provided, use "
                  "--rosbag_plugin_output_bag.";
    return common::kStupidUserError;
  }

  if (!common::fileExists(config.input_rosbag_path)) {
    LOG(ERROR) << "There is no input rosbag at " << config.input_rosbag_path
               << "!";
    return common::kStupidUserError;
  }

  if (common::fileExists(config.output_rosbag_path) && !FLAGS_overwrite) {
    LOG(ERROR) << "There is already a file at " << config.output_rosbag_path
               << "! Use --overwrite.";
    return common::kStupidUserError;
  }

  rosbag::Bag input_bag;
  input_bag.open(config.input_rosbag_path, rosbag::bagmode::Read);
  rosbag::View input_bag_view(input_bag);

  rosbag::Bag output_bag;
  output_bag.open(config.output_rosbag_path, rosbag::bagmode::Write);

  LOG(INFO) << "Copying existing messages into new bag...";
  const std::string tf_topic = "/tf";
  for (const rosbag::MessageInstance& message : input_bag_view) {
    if (!config.rosbag_plugin_pose_to_bag_drop_tf ||
        message.getTopic() != tf_topic) {
      output_bag.write(message.getTopic(), message.getTime(), message);
    }
  }

  vi_map::MissionIdList all_missions;
  vi_map.getAllMissionIdsSortedByTimestamp(&all_missions);

  size_t mission_number = 0u;
  for (const vi_map::MissionId& mission_id : all_missions) {
    // Set odom frame if there are multiple missions, add number.
    std::string odom_frame = config.pose_to_bag_odom_frame;
    std::string base_frame = config.pose_to_bag_base_frame;

    std::string T_G_I_topic = config.pose_to_bag_T_G_I_topic;
    std::string T_M_I_topic = config.pose_to_bag_T_M_I_topic;
    std::string T_G_M_topic = config.pose_to_bag_T_G_M_topic;

    if (all_missions.size() > 1u) {
      odom_frame += "_" + std::to_string(mission_number);
      base_frame += "_" + std::to_string(mission_number);
      T_G_I_topic += "_" + std::to_string(mission_number);
      T_M_I_topic += "_" + std::to_string(mission_number);
      T_G_M_topic += "_" + std::to_string(mission_number);
    }
    ++mission_number;

    const aslam::Transformation& T_G_M =
        vi_map.getMissionBaseFrameForMission(mission_id).get_T_G_M();

    // Check if there is IMU data to interpolate the optional sensor poses.
    landmark_triangulation::VertexToTimeStampMap vertex_to_time_map;
    int64_t min_timestamp_ns;
    int64_t max_timestamp_ns;
    const landmark_triangulation::PoseInterpolator pose_interpolator;
    pose_interpolator.getVertexToTimeStampMap(
        vi_map, mission_id, &vertex_to_time_map, &min_timestamp_ns,
        &max_timestamp_ns);
    if (vertex_to_time_map.size() < 2u) {
      VLOG(2) << "Couldn't find at least one IMU edge to interpolate exact "
                 "optional "
              << "sensor position in mission " << mission_id;
      continue;
    }

    VLOG(1) << "Computing timestamps to interpolate the poses at...";
    std::vector<int64_t> timestamps_from_vertices;
    for (const std::pair<const pose_graph::VertexId, int64_t>&
             vertex_timestamp_pair : vertex_to_time_map) {
      timestamps_from_vertices.emplace_back(vertex_timestamp_pair.second);
    }
    std::sort(timestamps_from_vertices.begin(), timestamps_from_vertices.end());

    constexpr double kOneSecondInNanoseconds = 1e9;
    CHECK_GT(FLAGS_rosbag_plugin_pose_to_bag_interpolate_to_frequency_hz, 0.0);
    const int64_t desired_period_ns =
        kOneSecondInNanoseconds /
        FLAGS_rosbag_plugin_pose_to_bag_interpolate_to_frequency_hz;
    std::vector<int64_t> timestamps_to_be_interpolated;
    for (size_t idx = 1u; idx < timestamps_from_vertices.size(); ++idx) {
      const int64_t period_ns =
          timestamps_from_vertices[idx] - timestamps_from_vertices[idx - 1];
      if (desired_period_ns > period_ns) {
        LOG(WARNING)
            << "The desired frequency is lower than the frequency of the pose "
               "graph, will only write vertex positions to rosbag!";
        break;
      }
      const int64_t num_additional_timestamps = period_ns / desired_period_ns;

      VLOG(3) << "Desired period: " << desired_period_ns
              << "ns VS pose graph period: " << period_ns << "ns";
      VLOG(3) << num_additional_timestamps
              << " additional timestamps are inserted";

      for (int64_t step = 1u; step <= num_additional_timestamps; ++step) {
        const int64_t additional_timestamp_ns =
            timestamps_from_vertices[idx - 1] + step * desired_period_ns;
        if (additional_timestamp_ns == timestamps_from_vertices[idx]) {
          // if we already reached the upper timestamp, stop.
          break;
        }
        CHECK_LT(additional_timestamp_ns, timestamps_from_vertices[idx]);
        timestamps_to_be_interpolated.push_back(additional_timestamp_ns);
      }
    }
    timestamps_to_be_interpolated.insert(
        timestamps_to_be_interpolated.end(), timestamps_from_vertices.begin(),
        timestamps_from_vertices.end());
    std::sort(
        timestamps_to_be_interpolated.begin(),
        timestamps_to_be_interpolated.end());

    // Map of data type to be compatible with interpolator.
    Eigen::Map<const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>>
        timestamps_to_be_interpolated_eigen(
            CHECK_NOTNULL(timestamps_to_be_interpolated.data()),
            timestamps_to_be_interpolated.size());

    VLOG(1) << "Interpolating poses at timestamps...";
    aslam::TransformationVector T_M_I_vector;
    pose_interpolator.getPosesAtTime(
        vi_map, mission_id, timestamps_to_be_interpolated_eigen, &T_M_I_vector);
    CHECK_EQ(
        static_cast<int>(T_M_I_vector.size()),
        timestamps_to_be_interpolated_eigen.size());

    VLOG(1) << "Writing poses into the rosbag...";
    for (size_t idx = 0u; idx < T_M_I_vector.size(); ++idx) {
      const aslam::Transformation& T_M_I = T_M_I_vector[idx];
      const aslam::Transformation T_G_I = T_G_M * T_M_I;

      ros::Time timestamp_ros;
      timestamp_ros.fromNSec(timestamps_to_be_interpolated[idx]);

      // T_M_I
      Eigen::Affine3d T_M_I_eigen;
      T_M_I_eigen.matrix() = T_M_I.getTransformationMatrix();
      tf::Transform T_M_I_tf;
      tf::transformEigenToTF(T_M_I_eigen, T_M_I_tf);
      tf::StampedTransform T_M_I_tf_msg(
          T_M_I_tf, timestamp_ros, odom_frame, base_frame);
      geometry_msgs::TransformStamped T_M_I_geom_msg;
      tf::transformStampedTFToMsg(T_M_I_tf_msg, T_M_I_geom_msg);

      // T_G_M
      Eigen::Affine3d T_G_M_eigen;
      T_G_M_eigen.matrix() = T_G_M.getTransformationMatrix();
      tf::Transform T_G_M_tf;
      tf::transformEigenToTF(T_G_M_eigen, T_G_M_tf);
      tf::StampedTransform T_G_M_tf_msg(
          T_G_M_tf, timestamp_ros, config.pose_to_bag_world_frame, odom_frame);
      geometry_msgs::TransformStamped T_G_M_geom_msg;
      tf::transformStampedTFToMsg(T_G_M_tf_msg, T_G_M_geom_msg);

      // T_G_I
      Eigen::Affine3d T_G_I_eigen;
      T_G_I_eigen.matrix() = T_G_I.getTransformationMatrix();
      tf::Transform T_G_I_tf;
      tf::transformEigenToTF(T_G_I_eigen, T_G_I_tf);
      tf::StampedTransform T_G_I_tf_msg(
          T_G_I_tf, timestamp_ros, config.pose_to_bag_world_frame, odom_frame);
      geometry_msgs::TransformStamped T_G_I_geom_msg;
      tf::transformStampedTFToMsg(T_G_I_tf_msg, T_G_I_geom_msg);

      if (config.pose_to_bag_add_transform_stamped) {
        output_bag.write(T_G_I_topic, timestamp_ros, T_G_I_geom_msg);
        output_bag.write(T_M_I_topic, timestamp_ros, T_M_I_geom_msg);
        output_bag.write(T_G_M_topic, timestamp_ros, T_G_M_geom_msg);
      }
      if (config.pose_to_bag_add_tf) {
        tf::tfMessage tf_msg;
        tf_msg.transforms.push_back(T_G_M_geom_msg);
        tf_msg.transforms.push_back(T_M_I_geom_msg);
        output_bag.write("/tf", timestamp_ros, tf_msg);
      }
    }
  }

  // Finish.
  input_bag.close();
  output_bag.close();
  return common::kSuccess;
}

RosbagPlugin::RosbagPlugin(
    common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter)
    : common::ConsolePluginBaseWithPlotter(console, plotter) {
  addCommand(
      {"pose_to_bag"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        Config config;
        config.getFromGflags();
        return exportPosesToRosbag(config, *map);
      },
      "Export poses into an existing rosbag.", common::Processing::Sync);
}

}  // namespace rosbag_plugin

MAPLAB_CREATE_CONSOLE_PLUGIN_WITH_PLOTTER(rosbag_plugin::RosbagPlugin);
