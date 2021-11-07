#include "vi-map-visualization-plugin/visualization-plugin.h"

#include <iostream>  //NOLINT
#include <memory>

#include <Eigen/Core>
#include <aslam/common/time.h>
#include <console-common/console.h>
#include <plotty/matplotlibcpp.hpp>
#include <visualization/landmark-observer-plotter.h>
#include <visualization/resource-visualization.h>
#include <visualization/viwls-graph-plotter.h>

DECLARE_string(map_mission);

namespace vi_visualization {

VisualizationPlugin::VisualizationPlugin(common::Console* console)
    : common::ConsolePluginBase(console) {
  addCommand(
      {"visualize_raw_images"},
      [this]() -> int {
        return visualizeCvMatResources(backend::ResourceType::kRawImage);
      },
      "Show mission fly-through of the raw image resources.",
      common::Processing::Sync);

  addCommand(
      {"visualize_undistorted_images"},
      [this]() -> int {
        return visualizeCvMatResources(
            backend::ResourceType::kUndistortedImage);
      },
      "Show mission fly-through of the undistorted image resources.",
      common::Processing::Sync);

  addCommand(
      {"visualize_raw_color_images"},
      [this]() -> int {
        return visualizeCvMatResources(backend::ResourceType::kRawColorImage);
      },
      "Show mission fly-through of the raw color image resources.",
      common::Processing::Sync);
  addCommand(
      {"visualize_undistorted_color_images"},
      [this]() -> int {
        return visualizeCvMatResources(
            backend::ResourceType::kUndistortedColorImage);
      },
      "Show mission fly-through of the undistorted color image resources.",
      common::Processing::Sync);

  addCommand(
      {"visualize_raw_depth_maps"},
      [this]() -> int {
        return visualizeCvMatResources(backend::ResourceType::kRawDepthMap);
      },
      "Show mission fly-through of the raw depth map resources.",
      common::Processing::Sync);

  addCommand(
      {"visualize_optimized_depth_maps"},
      [this]() -> int {
        return visualizeCvMatResources(
            backend::ResourceType::kOptimizedDepthMap);
      },
      "Show mission fly-through of the optimized depth map resources.",
      common::Processing::Sync);

  addCommand(
      {"visualize_disparity_maps"},
      [this]() -> int {
        return visualizeCvMatResources(backend::ResourceType::kDisparityMap);
      },
      "Show mission fly-through of the disparity image resources.",
      common::Processing::Sync);

  addCommand(
      {"visualize_raw_depth_map_pointclouds"},
      [this]() -> int {
        return visualizeReprojectedDepthResource(
            backend::ResourceType::kRawDepthMap);
      },
      "Publish all raw depth maps as point clouds.", common::Processing::Sync);

  addCommand(
      {"visualize_optimized_depth_map_pointclouds"},
      [this]() -> int {
        return visualizeReprojectedDepthResource(
            backend::ResourceType::kOptimizedDepthMap);
      },
      "Publish all optimized depth maps as point clouds.",
      common::Processing::Sync);

  addCommand(
      {"visualize_xyz_pointclouds"},
      [this]() -> int {
        return visualizeReprojectedDepthResource(
            backend::ResourceType::kPointCloudXYZ);
      },
      "Publish all xyz point clouds.", common::Processing::Sync);

  addCommand(
      {"visualize_xyzi_pointclouds"},
      [this]() -> int {
        return visualizeReprojectedDepthResource(
            backend::ResourceType::kPointCloudXYZI);
      },
      "Publish all xyz + intensity point clouds.", common::Processing::Sync);

  addCommand(
      {"visualize_xyzrgbn_pointclouds"},
      [this]() -> int {
        return visualizeReprojectedDepthResource(
            backend::ResourceType::kPointCloudXYZRGBN);
      },
      "Publish all xyz + color + normal point clouds.",
      common::Processing::Sync);

  addCommand(
      {"visualize_landmark_observers"},
      [this]() -> int { return visualizeLandmarkObserverRays(); },
      "Show rays to all observer camera positions for selected landmarks.",
      common::Processing::Sync);

  addCommand(
      {"visualize_bounding_boxes"},
      [this]() -> int {
        return visualizeBoundingBoxResources(
            backend::ResourceType::kObjectInstanceBoundingBoxes);
      },
      "Show mission fly-through of the bounding box resources on their "
      "corresponding images. Assumes they are attached as optional resources. "
      "It only supports one camera for now",
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

  addCommand(
      {"visualize_sensor_extrinsics"},
      [this]() -> int { return visualizeSensorExtrinsics(); },
      "Visualizes the extrinsics transformations of all sensors in RViz.",
      common::Processing::Sync);
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

int VisualizationPlugin::visualizeCvMatResources(backend::ResourceType type) {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);
  switch (type) {
    case backend::ResourceType::kRawImage:
    case backend::ResourceType::kUndistortedImage:
    case backend::ResourceType::kRawColorImage:
    case backend::ResourceType::kUndistortedColorImage:
    case backend::ResourceType::kRawDepthMap:
    case backend::ResourceType::kOptimizedDepthMap:
    case backend::ResourceType::kDisparityMap:
      return visualization::visualizeCvMatResources(*map, type);
    default:
      LOG(FATAL)
          << "'" << backend::ResourceTypeNames[static_cast<int>(type)]
          << "' is not a cv::Mat resource and cannot be visualized using "
          << "'visualizeCvMatResources()'!";
  }
  return common::kUnknownError;
}

int VisualizationPlugin::visualizeBoundingBoxResources(
    backend::ResourceType type) {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);  // maybe only need read
  switch (type) {
    case backend::ResourceType::kObjectInstanceBoundingBoxes:
      return visualization::visualizeBoundingBoxResources(*map, type);
    default:
      LOG(FATAL)
          << "'" << backend::ResourceTypeNames[static_cast<int>(type)]
          << "' is not currently supported and cannot be visualized using "
          << "'visualizeBoundingBoxResources()'!";
  }
  return common::kUnknownError;
}

int VisualizationPlugin::visualizeReprojectedDepthResource(
    backend::ResourceType type) {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  const vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(selected_map_key);

  vi_map::MissionIdList mission_ids;
  getAllMissionIds(map, &mission_ids);

  visualization::visualizeReprojectedDepthResource(type, mission_ids, *map);

  return common::kSuccess;
}

int VisualizationPlugin::visualizeSensorExtrinsics() const {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  const vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(selected_map_key);

  visualization::ViwlsGraphRvizPlotter plotter;
  plotter.visualizeSensorExtrinsics(*map);

  return common::kSuccess;
}

int VisualizationPlugin::visualizeLandmarkObserverRays() const {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  const vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(selected_map_key);
  const visualization::LandmarkObserverPlotter landmark_observer_plotter(*map);
  landmark_observer_plotter.visualizeClickedLandmarks();
  return common::kSuccess;
}

int VisualizationPlugin::getAllMissionIds(
    const vi_map::VIMapManager::MapReadAccess& map,
    vi_map::MissionIdList* mission_ids) const {
  if (FLAGS_map_mission.empty()) {
    map->getAllMissionIds(mission_ids);
    if (mission_ids->empty()) {
      LOG(ERROR)
          << "There are no missions available in the loaded map. Aborting.";
      return common::kUnknownError;
    }
  } else {
    vi_map::MissionId mission_id;
    map->ensureMissionIdValid(FLAGS_map_mission, &mission_id);
    if (!mission_id.isValid()) {
      LOG(ERROR) << "Mission ID invalid. Specify a valid mission id with "
                    "--map_mission.";
      return common::kUnknownError;
    }
    mission_ids->emplace_back(mission_id);
  }
  CHECK(!mission_ids->empty());

  return common::kSuccess;
}

}  // namespace vi_visualization

MAPLAB_CREATE_CONSOLE_PLUGIN(vi_visualization::VisualizationPlugin);
