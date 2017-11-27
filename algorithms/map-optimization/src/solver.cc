#include "map-optimization/solver.h"

#include <ceres-error-terms/problem-information.h>
#include <ceres/ceres.h>

namespace map_optimization {

ceres::TerminationType solve(
    const ceres::Solver::Options& solver_options,
    map_optimization::OptimizationProblem* optimization_problem) {
  CHECK_NOTNULL(optimization_problem);

  ceres::Problem problem(ceres_error_terms::getDefaultProblemOptions());
  ceres_error_terms::buildCeresProblemFromProblemInformation(
      optimization_problem->getProblemInformationMutable(), &problem);

  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);

  optimization_problem->getOptimizationStateBufferMutable()
      ->copyAllStatesBackToMap(optimization_problem->getMapMutable());

  LOG(INFO) << summary.FullReport();
  return summary.termination_type;
}

}  // namespace map_optimization
