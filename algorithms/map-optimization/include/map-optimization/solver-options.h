#ifndef MAP_OPTIMIZATION_SOLVER_OPTIONS_H_
#define MAP_OPTIMIZATION_SOLVER_OPTIONS_H_

#include <memory>
#include <vector>

#include <ceres/ceres.h>
#include <maplab-common/threading-helpers.h>

DECLARE_int32(ba_num_iterations);
DECLARE_bool(ba_use_cgnr_linear_solver);
DECLARE_bool(ba_use_jacobi_scaling);

namespace map_optimization {

inline ceres::Solver::Options initSolverOptionsFromFlags() {
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = FLAGS_ba_num_iterations;
  options.function_tolerance = 1e-12;
  options.gradient_tolerance = 1e-10;
  options.parameter_tolerance = 1e-8;
  options.num_threads = common::getNumHardwareThreads();
  options.num_linear_solver_threads = common::getNumHardwareThreads();
  options.jacobi_scaling = FLAGS_ba_use_jacobi_scaling;
  options.initial_trust_region_radius = 1e5;
  options.max_trust_region_radius = 1e20;

  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  if (FLAGS_ba_use_cgnr_linear_solver) {
    options.linear_solver_type = ceres::CGNR;
  } else {
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  }

  options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;

  // Do not copy the data back at every iteration. Remember to turn it on if
  // any callbacks consuming this data are added later.
  options.update_state_every_iteration = false;

  return options;
}

inline void addCallbacksToSolverOptions(
    const std::vector<std::shared_ptr<ceres::IterationCallback>>& callbacks,
    ceres::Solver::Options* options) {
  CHECK_NOTNULL(options);

  for (std::shared_ptr<ceres::IterationCallback> callback : callbacks) {
    CHECK_NOTNULL(callback.get());
    options->callbacks.push_back(callback.get());
  }
  if (!callbacks.empty()) {
    options->update_state_every_iteration = true;
  }
}

}  // namespace map_optimization
#endif  // MAP_OPTIMIZATION_SOLVER_OPTIONS_H_
