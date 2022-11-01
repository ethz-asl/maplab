#ifndef MAP_OPTIMIZATION_OPTIMIZATION_STATE_FIXING_H_
#define MAP_OPTIMIZATION_OPTIMIZATION_STATE_FIXING_H_

#include <vi-map/unique-id.h>

#include "map-optimization/optimization-problem.h"

namespace map_optimization {
inline void fixAllBaseframesInProblem(OptimizationProblem* problem) {
  CHECK_NOTNULL(problem);
  for (const vi_map::MissionId& mission_id : problem->getMissionIds()) {
    double* baseframe_state =
        problem->getOptimizationStateBufferMutable()
            ->get_baseframe_q_GM__G_p_GM_JPL(
                problem->getMapMutable()
                    ->getMissionBaseFrameForMission(mission_id)
                    .id());
    CHECK_NOTNULL(baseframe_state);
    problem->getProblemInformationMutable()
        ->setParameterBlockConstantIfPartOfTheProblem(baseframe_state);
  }
}

inline void fixAllVerticesInProblem(OptimizationProblem* problem) {
  CHECK_NOTNULL(problem);

  OptimizationStateBuffer* buffer =
      CHECK_NOTNULL(problem->getOptimizationStateBufferMutable());
  vi_map::VIMap* map = CHECK_NOTNULL(problem->getMapMutable());

  // For every vertex in the map fix the pose.
  for (const vi_map::MissionId& mission_id : problem->getMissionIds()) {
    pose_graph::VertexIdList vertex_ids;
    map->getAllVertexIdsInMissionAlongGraph(mission_id, &vertex_ids);

    for (const pose_graph::VertexId& vertex_id : vertex_ids) {
      CHECK(vertex_id.isValid());
      double* vertex_q_IM__M_p_MI =
          buffer->get_vertex_q_IM__M_p_MI_JPL(vertex_id);

      // Set constant.
      problem->getProblemInformationMutable()->setParameterBlockConstant(
          vertex_q_IM__M_p_MI);
    }
  }
}
}  // namespace map_optimization

#endif  // MAP_OPTIMIZATION_OPTIMIZATION_STATE_FIXING_H_
