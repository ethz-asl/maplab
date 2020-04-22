#include "map-optimization/solver.h"

#include <ceres-error-terms/problem-information.h>
#include <ceres/ceres.h>

namespace map_optimization {

ceres::TerminationType solve(
    const ceres::Solver::Options& solver_options,
    map_optimization::OptimizationProblem* optimization_problem) {
  return solve(solver_options, optimization_problem, nullptr /*result*/);
}

ceres::TerminationType solve(
    const ceres::Solver::Options& solver_options,
    map_optimization::OptimizationProblem* optimization_problem,
    OptimizationProblemResult* result) {
  CHECK_NOTNULL(optimization_problem);
  // 'result' cam be a nullptr

  ceres::Problem problem(ceres_error_terms::getDefaultProblemOptions());
  ceres_error_terms::buildCeresProblemFromProblemInformation(
      optimization_problem->getProblemInformationMutable(), &problem);

  IterationSummaryCallback callback;
  ceres::Solver::Options local_options = solver_options;
  if (result != nullptr) {
    // Install callback to retrieve IterationSummaries.
    local_options.callbacks.push_back(&callback);
  }

  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);

  optimization_problem->getOptimizationStateBufferMutable()
      ->copyAllStatesBackToMap(optimization_problem->getMapMutable());

  if (result != nullptr) {
    result->iteration_summaries = callback.iteration_summaries;
    result->solver_summaries.emplace_back(summary);
  }

  return summary.termination_type;
}

}  // namespace map_optimization
