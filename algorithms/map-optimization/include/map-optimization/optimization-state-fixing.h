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
}  // namespace map_optimization

#endif  // MAP_OPTIMIZATION_OPTIMIZATION_STATE_FIXING_H_
