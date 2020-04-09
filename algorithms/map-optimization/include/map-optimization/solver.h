#ifndef MAP_OPTIMIZATION_SOLVER_H_
#define MAP_OPTIMIZATION_SOLVER_H_

#include "map-optimization/optimization-problem.h"

namespace map_optimization {

class IterationSummaryCallback : public ceres::IterationCallback {
 public:
  virtual ~IterationSummaryCallback() {}

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
    iteration_summaries.emplace_back(summary);
    return ceres::SOLVER_CONTINUE;
  }
  std::vector<ceres::IterationSummary> iteration_summaries;
};

ceres::TerminationType solve(
    const ceres::Solver::Options& solver_options,
    map_optimization::OptimizationProblem* optimization_problem,
    OptimizationProblemResult* result);

ceres::TerminationType solve(
    const ceres::Solver::Options& solver_options,
    map_optimization::OptimizationProblem* optimization_problem);

}  // namespace map_optimization
#endif  // MAP_OPTIMIZATION_SOLVER_H_
