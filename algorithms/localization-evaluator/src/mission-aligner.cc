#include "localization-evaluator/mission-aligner.h"

#include <loop-closure-handler/loop-detector-node.h>
#include <map-anchoring/map-anchoring.h>
#include <map-optimization/solver-options.h>
#include <map-optimization/vi-map-optimizer.h>
#include <maplab-common/progress-bar.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

namespace localization_evaluator {

void alignAndCooptimizeMissionsWithoutLandmarkMerge(
    const vi_map::MissionId& query_mission_id,
    const vi_map::MissionIdSet& map_missions_ids,
    bool should_align_map_missions, bool optimize_only_query_mission,
    vi_map::VIMap* map) {
  CHECK(map->hasMission(query_mission_id));
  CHECK(!map_missions_ids.empty());
  CHECK_NOTNULL(map);

  loop_detector_node::LoopDetectorNode loop_detector;

  // Add map mission to the DB.
  for (const vi_map::MissionId& mission_id : map_missions_ids) {
    pose_graph::VertexId current_vertex_id =
        map->getMission(mission_id).getRootVertexId();

    VLOG(1) << "Adding map mission to the Loop Closure DB: " << mission_id;
    do {
      loop_detector.addVertexToDatabase(current_vertex_id, *map);
    } while (map->getNextVertex(
        current_vertex_id, map->getGraphTraversalEdgeType(mission_id),
        &current_vertex_id));
  }

  // To revert the landmark merge.
  vi_map::VertexKeyPointToStructureMatchList landmark_merge_revert_vector;

  // Find matches from a mission to structure.
  pose_graph::VertexIdList query_vertices;
  map->getAllVertexIdsInMission(query_mission_id, &query_vertices);

  int vertex_index = 0;
  int num_successful_vertex_queries = 0;
  common::ProgressBar progress_bar(query_vertices.size());
  VLOG(1) << "Querying " << query_vertices.size() << " vertices from mission "
          << query_mission_id << ".";
  for (const pose_graph::VertexId& current_query_vertex : query_vertices) {
    progress_bar.increment();
    ++vertex_index;

    pose::Transformation T_G_I;
    vi_map::LoopClosureConstraint inlier_constraint;

    const bool kMergeLandmarks = false;
    const bool kAddLoopclosureEdges = false;
    unsigned int num_of_lc_matches = 0;
    vi_map::Vertex& vertex = map->getVertex(current_query_vertex);
    if (loop_detector.findVertexInDatabase(
            vertex, kMergeLandmarks, kAddLoopclosureEdges, map, &T_G_I,
            &num_of_lc_matches, &inlier_constraint)) {
      VLOG(3) << "RANSAC successful.";
      ++num_successful_vertex_queries;

      for (const vi_map::VertexKeyPointToStructureMatch& match :
           inlier_constraint.structure_matches) {
        const vi_map::LandmarkId& query_landmark_id =
            vertex.getObservedLandmarkId(
                match.frame_index_query, match.keypoint_index_query);

        const vi_map::LandmarkId resulting_landmark_id = match.landmark_result;
        CHECK(resulting_landmark_id.isValid());

        // WARNING: This introduces map inconsistency that will be recitified
        // below, after the optimization. Most importantly, the backlinks
        // are not updated here.
        vertex.setObservedLandmarkId(
            match.frame_index_query, match.keypoint_index_query,
            resulting_landmark_id);
        vi_map::VertexKeyPointToStructureMatch revert_match = match;
        revert_match.landmark_result = query_landmark_id;
        revert_match.frame_identifier_result.vertex_id = current_query_vertex;
        landmark_merge_revert_vector.emplace_back(revert_match);
      }
    }
  }

  VLOG(1) << num_successful_vertex_queries << " successful vertex queries. "
          << "Found " << landmark_merge_revert_vector.size()
          << " landmark merges.";

  VLOG(1) << "Setting query mission baseframe to unknown, rest to known.";
  map_anchoring::setMissionBaseframeKnownState(query_mission_id, false, map);
  for (const vi_map::MissionId& map_mission_id : map_missions_ids) {
    if (!should_align_map_missions) {
      // Fix baseframes of map missions if they shouldn't be aligned.
      map_anchoring::setMissionBaseframeKnownState(map_mission_id, true, map);
    }
  }

  VLOG(1) << "Anchoring missions.";
  const bool anchor_result =
      map_anchoring::anchorMission(query_mission_id, map);
  if (!anchor_result) {
    LOG(ERROR) << "Anchor failed. Bailing out.";
    return;
  }
  VLOG(1) << "Anchor succeeded.";

  VLOG(1) << "Optimizing missions.";
  ceres::Solver::Options solver_options =
      map_optimization::initSolverOptionsFromFlags();
  map_optimization::ViProblemOptions vi_problem_options =
      map_optimization::ViProblemOptions::initFromGFlags();

  solver_options.max_num_iterations = 5;
  if (optimize_only_query_mission) {
    vi_problem_options.fix_vertices = true;
    vi_problem_options.fix_landmark_positions = true;
  }

  // Initial visual-inertial optimization.
  constexpr bool kEnableSignalHandler = true;
  map_optimization::VIMapOptimizer optimizer(nullptr, kEnableSignalHandler);

  // Optimize all missions in the map
  vi_map::MissionIdSet all_mission_ids;
  map->getAllMissionIds(&all_mission_ids);
  const bool success = optimizer.optimize(
      vi_problem_options, solver_options, all_mission_ids, nullptr, map);
  CHECK(success) << "Optimization failed!";

  for (const vi_map::VertexKeyPointToStructureMatch& revert_item :
       landmark_merge_revert_vector) {
    // Revert to the old landmark references s.t. the missions are independent
    // again. This will also make the map 100% consistent.
    vi_map::Vertex& vertex =
        map->getVertex(revert_item.frame_identifier_result.vertex_id);
    vertex.setObservedLandmarkId(
        revert_item.frame_index_query, revert_item.keypoint_index_query,
        revert_item.landmark_result);
  }
}

}  // namespace localization_evaluator
