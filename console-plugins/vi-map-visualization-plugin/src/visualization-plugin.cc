#include "vi-map-visualization-plugin/visualization-plugin.h"

#include <iostream>  //NOLINT
#include <memory>

#include <Eigen/Core>
#include <aslam/common/time.h>
#include <console-common/console.h>
#include <map-manager/map-manager.h>
#include <plotty/matplotlibcpp.hpp>
#include <vi-map/vi-map.h>
#include <visualization/visualizer.h>

DECLARE_string(map_mission);

namespace vi_visualization {

VisualizationPlugin::VisualizationPlugin(common::Console* console)
    : common::ConsolePluginBase(console) {
  addCommand(
      {"visualize_raw_images"},
      [this]() -> int {
        return visualizerCvMatResources(backend::ResourceType::kRawImage);
      },
      "Show mission fly-through of the raw image resources.",
      common::Processing::Sync);

  addCommand(
      {"visualize_undistorted_images"},
      [this]() -> int {
        return visualizerCvMatResources(
            backend::ResourceType::kUndistortedImage);
      },
      "Show mission fly-through of the undistorted image resources.",
      common::Processing::Sync);

  addCommand(
      {"visualize_raw_color_images"},
      [this]() -> int {
        return visualizerCvMatResources(backend::ResourceType::kRawColorImage);
      },
      "Show mission fly-through of the raw color image resources.",
      common::Processing::Sync);
  addCommand(
      {"visualize_undistorted_color_images"},
      [this]() -> int {
        return visualizerCvMatResources(
            backend::ResourceType::kUndistortedColorImage);
      },
      "Show mission fly-through of the undistorted color image resources.",
      common::Processing::Sync);

  addCommand(
      {"visualize_raw_depth_maps"},
      [this]() -> int {
        return visualizerCvMatResources(backend::ResourceType::kRawDepthMap);
      },
      "Show mission fly-through of the raw depth map resources.",
      common::Processing::Sync);

  addCommand(
      {"visualize_optimized_depth_maps"},
      [this]() -> int {
        return visualizerCvMatResources(
            backend::ResourceType::kOptimizedDepthMap);
      },
      "Show mission fly-through of the optimized depth map resources.",
      common::Processing::Sync);

  addCommand(
      {"visualize_disparity_maps"},
      [this]() -> int {
        return visualizerCvMatResources(backend::ResourceType::kDisparityMap);
      },
      "Show mission fly-through of the disparity image resources.",
      common::Processing::Sync);

  addCommand(
      {"plot_vi_states_of_mission"},
      [this]() -> int {
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }

        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapReadAccess map =
            map_manager.getMapReadAccess(selected_map_key);
        vi_map::MissionId mission_id;
        map->ensureMissionIdValid(FLAGS_map_mission, &mission_id);
        if (!mission_id.isValid()) {
          LOG(ERROR) << "The given mission \"" << FLAGS_map_mission
                     << "\" is not valid.";
          return common::kUnknownError;
        }
        plotVIStatesOfMission(*map, mission_id);
        return common::kSuccess;
      },
      "Plot visual-inertial states of the mission.", common::Processing::Sync);
}

void VisualizationPlugin::plotVIStatesOfMission(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id) const {
  pose_graph::VertexIdList vertices;
  map.getAllVertexIdsInMissionAlongGraph(mission_id, &vertices);

  Eigen::VectorXd timestamps_s(vertices.size());
  Eigen::Matrix3Xd p_M_I(3, vertices.size());
  Eigen::Matrix3Xd v_M(3, vertices.size());
  Eigen::Matrix3Xd accel_bias(3, vertices.size());
  Eigen::Matrix3Xd gyro_bias(3, vertices.size());
  size_t idx = 0u;
  for (const pose_graph::VertexId& vertex_id : vertices) {
    const vi_map::Vertex& vertex = map.getVertex(vertex_id);
    timestamps_s(idx, 0) =
        aslam::time::to_seconds(vertex.getMinTimestampNanoseconds());
    p_M_I.col(idx) = vertex.get_p_M_I();
    v_M.col(idx) = vertex.get_v_M();
    accel_bias.col(idx) = vertex.getAccelBias();
    gyro_bias.col(idx) = vertex.getGyroBias();
    ++idx;
  }
  timestamps_s.array() = timestamps_s.array() - timestamps_s(0, 0);

  plotty::subplot(4, 1, 1);
  plotty::plot(timestamps_s, p_M_I.row(0));
  plotty::plot(timestamps_s, p_M_I.row(1));
  plotty::plot(timestamps_s, p_M_I.row(2));
  plotty::xlabel("time [s]");
  plotty::ylabel("p_M [m]");
  plotty::title("Position in mission-frame");

  plotty::subplot(4, 1, 2);
  plotty::plot(timestamps_s, v_M.row(0));
  plotty::plot(timestamps_s, v_M.row(1));
  plotty::plot(timestamps_s, v_M.row(2));
  plotty::xlabel("time [s]");
  plotty::ylabel("v_M [m/s]");
  plotty::title("Velocity in mission-frame.");

  plotty::subplot(4, 1, 3);
  plotty::plot(timestamps_s, accel_bias.row(0));
  plotty::plot(timestamps_s, accel_bias.row(1));
  plotty::plot(timestamps_s, accel_bias.row(2));
  plotty::xlabel("time [s]");
  plotty::ylabel("b_a [m/s^2]");
  plotty::title("Accelerometer bias in IMU-frame.");

  plotty::subplot(4, 1, 4);
  plotty::plot(timestamps_s, gyro_bias.row(0));
  plotty::plot(timestamps_s, gyro_bias.row(1));
  plotty::plot(timestamps_s, gyro_bias.row(2));
  plotty::xlabel("time [s]");
  plotty::ylabel("b_g [rad/s]");
  plotty::title("Gyroscope bias in IMU-frame.");

  plotty::show();
}

int VisualizationPlugin::visualizerCvMatResources(backend::ResourceType type) {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);
  visualizer_.reset(new visualization::Visualizer(map.get()));
  switch (type) {
    case backend::ResourceType::kRawImage:
      return visualizer_->visualizeCvMatResources(
          backend::ResourceType::kRawImage);
    case backend::ResourceType::kUndistortedImage:
      return visualizer_->visualizeCvMatResources(
          backend::ResourceType::kUndistortedImage);
    case backend::ResourceType::kRawColorImage:
      return visualizer_->visualizeCvMatResources(
          backend::ResourceType::kRawColorImage);
    case backend::ResourceType::kUndistortedColorImage:
      return visualizer_->visualizeCvMatResources(
          backend::ResourceType::kUndistortedColorImage);
    case backend::ResourceType::kRawDepthMap:
      return visualizer_->visualizeCvMatResources(
          backend::ResourceType::kRawDepthMap);
    case backend::ResourceType::kOptimizedDepthMap:
      return visualizer_->visualizeCvMatResources(
          backend::ResourceType::kOptimizedDepthMap);
    case backend::ResourceType::kDisparityMap:
      return visualizer_->visualizeCvMatResources(
          backend::ResourceType::kDisparityMap);
    default:
      LOG(FATAL) << "Non-compatible resource type found !";
  }
  return common::kUnknownError;
}
}  // namespace vi_visualization

MAPLAB_CREATE_CONSOLE_PLUGIN(vi_visualization::VisualizationPlugin);
