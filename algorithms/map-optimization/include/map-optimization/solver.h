#ifndef MAP_OPTIMIZATION_SOLVER_H_
#define MAP_OPTIMIZATION_SOLVER_H_

#include "map-optimization/optimization-problem.h"

namespace map_optimization {

ceres::TerminationType solve(
    const ceres::Solver::Options& solver_options,
    map_optimization::OptimizationProblem* optimization_problem);

}  // namespace map_optimization
#endif  // MAP_OPTIMIZATION_SOLVER_H_
