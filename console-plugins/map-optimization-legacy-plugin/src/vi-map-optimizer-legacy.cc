#include "map-optimization-legacy-plugin/vi-map-optimizer-legacy.h"

#include <functional>
#include <string>
#include <unordered_map>

#include <map-optimization-legacy/ba-optimization-options.h>
#include <map-optimization-legacy/graph-ba-optimizer.h>
#include <maplab-common/file-logger.h>
#include <maplab-common/progress-bar.h>
#include <visualization/viwls-graph-plotter.h>

namespace map_optimization_legacy_plugin {

VIMapOptimizer::VIMapOptimizer()
    : plotter_(nullptr), signal_handler_enabled_(false) {}

VIMapOptimizer::VIMapOptimizer(visualization::ViwlsGraphRvizPlotter* plotter)
    : plotter_(plotter), signal_handler_enabled_(false) {}

VIMapOptimizer::VIMapOptimizer(
    visualization::ViwlsGraphRvizPlotter* plotter, bool signal_handler_enabled)
    : plotter_(plotter), signal_handler_enabled_(signal_handler_enabled) {}

void VIMapOptimizer::visualizePosegraph(const vi_map::VIMap& map) const {
  if (plotter_ == nullptr) {
    VLOG(3) << "You need to initialize with a plotter";
    return;
  }

  plotter_->visualizeMap(map);
}

int VIMapOptimizer::optimizeUsingDefaultOptions(vi_map::VIMap* map) {
  ceres::Solver::Summary summary;
  map_optimization_legacy::BaOptimizationOptions options;
  return optimize(options, map, &summary);
}

int VIMapOptimizer::optimize(
    const map_optimization_legacy::BaOptimizationOptions& options,
    vi_map::VIMap* map) {
  ceres::Solver::Summary summary;
  return optimize(options, map, &summary);
}

int VIMapOptimizer::relaxPosegraphBasedOnLoopclosureEdges(vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  map_optimization_legacy::BaOptimizationOptions relaxation_options;
  relaxation_options.include_visual = true;
  relaxation_options.fix_accel_bias = true;
  relaxation_options.fix_gyro_bias = true;
  relaxation_options.fix_velocity = true;
  relaxation_options.include_loop_closure_edges = true;
  relaxation_options.use_switchable_constraints_for_loop_closure_edges = false;

  ceres::Solver::Summary summary;
  return optimize(relaxation_options, map, &summary);
}

int VIMapOptimizer::optimize(
    const map_optimization_legacy::BaOptimizationOptions& options,
    vi_map::VIMap* map, ceres::Solver::Summary* summary) {
  CHECK_NOTNULL(map);
  CHECK_NOTNULL(summary);

  vi_map::MissionIdSet mission_ids_to_optimize =
      options.mission_ids_to_optimize;
  if (mission_ids_to_optimize.empty()) {
    // Optimize all missions.
    vi_map::MissionIdList all_mission_ids_list;
    map->getAllMissionIds(&all_mission_ids_list);
    mission_ids_to_optimize.insert(
        all_mission_ids_list.begin(), all_mission_ids_list.end());
  } else if (!options.fix_not_selected_missions) {
    // Optimize a subset of missions.
    map->selectMissions(mission_ids_to_optimize);
  }

  if (map->numMissions() == 0u) {
    LOG(ERROR) << "No missions in database.";
    return kNoData;
  }

  VLOG(1) << "Loading data for optimization, num of baseframes: "
          << map->numMissionBaseFrames();

  pose_graph::VertexIdSet fixed_vertices;
  const vi_map::MissionId first_mission_id = *mission_ids_to_optimize.begin();

  const bool vision_only =
      options.include_visual &&
      !(options.include_inertial || options.include_wheel_odometry ||
        options.include_gps);
  const pose_graph::VertexId root_vertex_id =
      map->getMission(first_mission_id).getRootVertexId();
  if (!options.fix_not_selected_missions) {
    if (options.fix_root_vertex) {
      fixed_vertices.insert(root_vertex_id);
    } else if (vision_only) {
      LOG(ERROR) << "Vision only BA requires fixing the root vertex (and the "
                    "second vertex to fix the scale).";
      return common::CommandStatus::kStupidUserError;
    }
  }

  if (options.fix_vertex_poses) {
    pose_graph::VertexIdList all_vertices;
    map->getAllVertexIds(&all_vertices);
    fixed_vertices.insert(all_vertices.begin(), all_vertices.end());
  } else if (options.fix_not_selected_missions) {
    pose_graph::VertexIdList all_vertices;
    map->getAllVertexIds(&all_vertices);
    for (const pose_graph::VertexIdList::value_type& vertex_id : all_vertices) {
      if (mission_ids_to_optimize.count(
              map->getVertex(vertex_id).getMissionId()) == 0u) {
        fixed_vertices.insert(vertex_id);
      }
    }
  }

  if (!options.fix_vertex_poses_except_of_mission.empty()) {
    vi_map::MissionId free_mission_id;
    CHECK(
        map->ensureMissionIdValid(
            options.fix_vertex_poses_except_of_mission, &free_mission_id))
        << "The mission id string of the missions whose vertex poses should "
           "not "
        << "be fixed is invalid: "
        << options.fix_vertex_poses_except_of_mission;
    CHECK(free_mission_id.isValid());
    vi_map::MissionIdList all_mission_ids;
    map->getAllMissionIds(&all_mission_ids);
    for (const vi_map::MissionIdList::value_type& mission_id :
         all_mission_ids) {
      if (mission_id != free_mission_id) {
        pose_graph::VertexIdList vertices_of_mission;
        map->getAllVertexIdsInMission(mission_id, &vertices_of_mission);
        fixed_vertices.insert(
            vertices_of_mission.begin(), vertices_of_mission.end());
      }
    }
  }

  vi_map::MissionBaseFrameIdSet fixed_baseframes;
  if (options.fix_not_selected_missions) {
    vi_map::MissionIdList all_mission_ids;
    map->getAllMissionIds(&all_mission_ids);
    for (const vi_map::MissionIdList::value_type& mission_id :
         all_mission_ids) {
      if (mission_ids_to_optimize.count(mission_id) == 0u) {
        fixed_baseframes.insert(map->getMission(mission_id).getBaseFrameId());
      }
    }
  }

  VLOG(1) << "Fixing " << fixed_vertices.size() << " vertex poses.";

  visualizePosegraph(*map);

  map_optimization_legacy::GraphBaOptimizer optimizer(map);
  optimizer.enableCeresSignalHandlerCallback(signal_handler_enabled_);

  std::function<void(const vi_map::VIMap&)> callback =
      [this](const vi_map::VIMap& map_arg) {
        this->visualizePosegraph(map_arg);
      };  // NOLINT

  if (vision_only) {
    // Vision-only BA requires fixing of two first vertices to fix the scale
    // of the problem.
    pose_graph::VertexId second_vertex_id;
    if (map->getNextVertex(
            root_vertex_id, map->getGraphTraversalEdgeType(first_mission_id),
            &second_vertex_id)) {
      fixed_vertices.insert(second_vertex_id);
    }
    optimizer.visualBaOptimizationWithCallback(
        fixed_vertices, options, callback, summary);
  } else {
    pose_graph::VertexIdSet velocity_prior_for_vertices;
    optimizer.visualInertialBaOptimizationWithCallback(
        fixed_baseframes, fixed_vertices, velocity_prior_for_vertices, options,
        callback, summary);
  }

  visualizePosegraph(*map);

  return common::kSuccess;
}

}  // namespace map_optimization_legacy_plugin
