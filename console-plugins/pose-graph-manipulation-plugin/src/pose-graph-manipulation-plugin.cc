#include "pose-graph-manipulation-plugin/pose-graph-manipulation-plugin.h"

#include <glog/logging.h>
#include <map-manager/map-manager.h>
#include <maplab-common/progress-bar.h>
#include <vi-map/vi-map.h>

#include "pose-graph-manipulation-plugin/edge-manipulation.h"
#include "pose-graph-manipulation-plugin/reset-wheel-odometry.h"

DEFINE_string(edge_type, "", "Specify the edge type.");
DEFINE_double(translation_std_dev_meters, -1.0,
              "Translation standard deviation in meters.");
DEFINE_double(orientation_std_dev_degrees, -1.0,
              "Orientation standard deviation in meters.");
DEFINE_double(
    lc_switch_variable_variance, 1e-8,
    "The variance for the switch variable of the loop-closure "
    "edges.");
DEFINE_double(
    lc_switch_variable_value, 1.0,
    "The value for the switch variable of the loop-closure "
    "edges, between 0.0 (edge is ignored) and 1.0 (edge is being "
    "fully trusted).");

namespace pose_graph_manipulation {

PoseGraphManipulationPlugin::PoseGraphManipulationPlugin(
    common::Console* console) : common::ConsolePluginBase(console) {
  CHECK_NOTNULL(console);

  addCommand(
      {"rwot", "reset_vertex_poses_to_wheel_odometry_trajectory"},
      [this]() -> int { return resetVertexPosesToWheelOdometryTrajectory(); },
      "Resets the vertex poses by integrating the relative pose information "
      "stored in the odometry edges of the pose graph.",
      common::Processing::Sync);

  addCommand(
      {"aeu", "assign_edge_uncertainties"},
      [this]()->int { return assignEdgeUncertainties(); },
      "Assigns a specified uncertainty to a certain type of edges. Specify the "
      "edge type with --edge_type. Specify the weight with "
      "--translation_std_dev_meters, and --orientation_std_dev_degrees.",
      common::Processing::Sync);
  addCommand({"asvuflce",
              "assign_switch_variable_uncertainties_for_loop-closure_edges"},
             [this]()->int {
               return assignSwitchVariableUncertaintiesForLoopClosureEdges();
             },
             "Assigns a specified uncertainty to the switch variables of all "
             "loop-closure edges. Specify the switch variable variance with "
             "--lc_switch_variable_variance.",
             common::Processing::Sync);
  addCommand(
      {"asvv", "assign_switch_variable_values_for_loop-closure_edges"},
      [this]() -> int {
        return assignSwitchVariableValuesForLoopClosureEdges();
      },
      "Assigns a specified value to the switch variables of all "
      "loop-closure edges. Specify the switch variable value with "
      "--lc_switch_variable_value.",
      common::Processing::Sync);
}

int PoseGraphManipulationPlugin::resetVertexPosesToWheelOdometryTrajectory()
const {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  return pose_graph_manipulation::resetVertexPosesToWheelOdometryTrajectory(
      map.get());
}

int PoseGraphManipulationPlugin::assignEdgeUncertainties() {
  if (FLAGS_edge_type.empty()) {
    LOG(ERROR) << "Please specify an edge type with --edge_type.";
    return common::kStupidUserError;
  }
  const vi_map::Edge::EdgeType edge_type =
      pose_graph::Edge::stringToEdgeType(FLAGS_edge_type);
  if (edge_type != vi_map::Edge::EdgeType::kLoopClosure &&
      edge_type != vi_map::Edge::EdgeType::kOdometry) {
    LOG(ERROR) << "Please choose a valid edge type.";
    return common::kStupidUserError;
  }

  const double translation_std_dev_meters = FLAGS_translation_std_dev_meters;
  if (translation_std_dev_meters <= 0.0) {
    LOG(ERROR) << "Please specify a strictly positive value for "
               << "--translation_std_dev_meters.";
    return common::kStupidUserError;
  }
  const double orientation_std_dev_degrees = FLAGS_orientation_std_dev_degrees;
  if (orientation_std_dev_degrees <= 0.0) {
    LOG(ERROR) << "Please specify a strictly positive value for "
               << "--translation_std_dev_meters.";
    return common::kStupidUserError;
  }

  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  return pose_graph_manipulation::assignEdgeUncertainties(
      edge_type, translation_std_dev_meters, orientation_std_dev_degrees,
      map.get());
}

int PoseGraphManipulationPlugin::
    assignSwitchVariableUncertaintiesForLoopClosureEdges() {
  const double switch_variable_variance = FLAGS_lc_switch_variable_variance;
  if (switch_variable_variance <= 0.0) {
    LOG(ERROR) << "Please specify a positive variance for the switch variable.";
  }

  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  return pose_graph_manipulation::
      assignSwitchVariableUncertaintiesForLoopClosureEdges(
          switch_variable_variance, map.get());
}

int PoseGraphManipulationPlugin::
    assignSwitchVariableValuesForLoopClosureEdges() {
  const double switch_variable_value = FLAGS_lc_switch_variable_value;
  if (switch_variable_value < 0.0 || switch_variable_value > 1.0) {
    LOG(ERROR) << "Please specify a value for the switch variable in the range "
               << "[0.0, 1.0].";
    return common::kStupidUserError;
  }

  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  return pose_graph_manipulation::assignSwitchVariableValuesForLoopClosureEdges(
      switch_variable_value, map.get());
}

}  // namespace pose_graph_manipulation

MAPLAB_CREATE_CONSOLE_PLUGIN(
    pose_graph_manipulation::PoseGraphManipulationPlugin);
