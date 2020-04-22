#ifndef MAP_OPTIMIZATION_SOLVER_OPTIONS_H_
#define MAP_OPTIMIZATION_SOLVER_OPTIONS_H_

#include <memory>
#include <vector>

#include <ceres/ceres.h>
#include <maplab-common/threading-helpers.h>

DECLARE_int32(ba_num_iterations);
DECLARE_int32(ba_max_time_seconds);
DECLARE_bool(ba_enable_signal_handler);
DECLARE_string(ba_linear_solver_type);
DECLARE_bool(ba_use_jacobi_scaling);
DECLARE_bool(ba_use_nonmonotonic_steps);
DECLARE_int32(ba_max_consecutive_nonmonotonic_steps);
DECLARE_double(ba_initial_trust_region_radius);
DECLARE_double(ba_max_trust_region_radius);
DECLARE_double(ba_min_trust_region_radius);
DECLARE_double(ba_function_tolerance);
DECLARE_double(ba_gradient_tolerance);
DECLARE_double(ba_parameter_tolerance);
DECLARE_string(ba_sparse_linear_algebra_library_type);

namespace map_optimization {
inline ceres::Solver::Options initSolverOptionsFromFlags() {
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = FLAGS_ba_num_iterations;
  options.max_solver_time_in_seconds = FLAGS_ba_max_time_seconds;
  options.function_tolerance = FLAGS_ba_function_tolerance;
  options.gradient_tolerance = FLAGS_ba_gradient_tolerance;
  options.parameter_tolerance = FLAGS_ba_parameter_tolerance;
  options.use_nonmonotonic_steps = FLAGS_ba_use_nonmonotonic_steps;
  options.max_consecutive_nonmonotonic_steps =
      FLAGS_ba_max_consecutive_nonmonotonic_steps;
  options.num_threads =
      std::max(1u, static_cast<uint32_t>(common::getNumHardwareThreads() / 2u));
  options.jacobi_scaling = FLAGS_ba_use_jacobi_scaling;
  options.initial_trust_region_radius = FLAGS_ba_initial_trust_region_radius;
  options.max_trust_region_radius = FLAGS_ba_max_trust_region_radius;
  options.min_trust_region_radius = FLAGS_ba_min_trust_region_radius;

  if (FLAGS_ba_sparse_linear_algebra_library_type == "SUITE_SPARSE") {
    options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
  } else if (FLAGS_ba_sparse_linear_algebra_library_type == "CX_SPARSE") {
    options.sparse_linear_algebra_library_type = ceres::CX_SPARSE;
  } else if (FLAGS_ba_sparse_linear_algebra_library_type == "EIGEN_SPARSE") {
    options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
  } else {
    LOG(FATAL) << "Unkown type for --ba_sparse_linear_algebra_library_type: "
               << "'" << FLAGS_ba_sparse_linear_algebra_library_type << "'!";
  }

  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

  if (FLAGS_ba_linear_solver_type == "CGNR") {
    options.linear_solver_type = ceres::CGNR;
  } else if (FLAGS_ba_linear_solver_type == "SPARSE_NORMAL_CHOLESKY") {
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  } else if (FLAGS_ba_linear_solver_type == "SPARSE_SCHUR") {
    options.linear_solver_type = ceres::SPARSE_SCHUR;
  } else if (FLAGS_ba_linear_solver_type == "ITERATIVE_SCHUR") {
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  } else {
    LOG(FATAL) << "Unkown type for --ba_linear_solver_type: "
               << "'" << FLAGS_ba_linear_solver_type << "'!";
  }

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
