#include "map-optimization/vi-map-optimizer.h"

#include <map-optimization/augment-loopclosure.h>
#include <map-optimization/callbacks.h>
#include <map-optimization/outlier-rejection-solver.h>
#include <map-optimization/solver-options.h>
#include <map-optimization/solver.h>
#include <map-optimization/vi-optimization-builder.h>
#include <maplab-common/file-logger.h>
#include <maplab-common/progress-bar.h>
#include <visualization/viwls-graph-plotter.h>

#include <functional>
#include <string>
#include <unordered_map>

DEFINE_int32(
    ba_visualize_every_n_iterations, 3,
    "Update the visualization every n optimization iterations.");

namespace map_optimization {

VIMapOptimizer::VIMapOptimizer(
    const visualization::ViwlsGraphRvizPlotter* plotter,
    bool signal_handler_enabled)
    : plotter_(plotter), signal_handler_enabled_(signal_handler_enabled) {}

bool VIMapOptimizer::optimize(
    const map_optimization::ViProblemOptions& options,
    const vi_map::MissionIdSet& missions_to_optimize,
    const map_optimization::OutlierRejectionSolverOptions* const
        outlier_rejection_options,
    vi_map::VIMap* map) {
  // outlier_rejection_options is optional.
  CHECK_NOTNULL(map);

  ceres::Solver::Options solver_options =
      map_optimization::initSolverOptionsFromFlags();
  return optimize(
      options, solver_options, missions_to_optimize, outlier_rejection_options,
      map);
}

bool VIMapOptimizer::optimize(
    const map_optimization::ViProblemOptions& options,
    const ceres::Solver::Options& solver_options,
    const vi_map::MissionIdSet& missions_to_optimize,
    const map_optimization::OutlierRejectionSolverOptions* const
        outlier_rejection_options,
    vi_map::VIMap* map) {
  // outlier_rejection_options is optional.
  CHECK_NOTNULL(map);

  if (missions_to_optimize.empty()) {
    LOG(WARNING) << "Nothing to optimize.";
    return false;
  }

  map_optimization::OptimizationProblem::UniquePtr optimization_problem(
      map_optimization::constructOptimizationProblem(
          missions_to_optimize, options, map));
  CHECK(optimization_problem != nullptr);

  std::vector<std::shared_ptr<ceres::IterationCallback>> callbacks;
  if (plotter_) {
    map_optimization::appendVisualizationCallbacks(
        FLAGS_ba_visualize_every_n_iterations,
        *(optimization_problem->getOptimizationStateBufferMutable()), *plotter_,
        map, &callbacks);
  }
  if (FLAGS_ba_enable_signal_handler) {
    map_optimization::appendSignalHandlerCallback(&callbacks);
  }
  ceres::Solver::Options solver_options_with_callbacks = solver_options;
  map_optimization::addCallbacksToSolverOptions(
      callbacks, &solver_options_with_callbacks);

  if (outlier_rejection_options != nullptr) {
    map_optimization::solveWithOutlierRejection(
        solver_options_with_callbacks, *outlier_rejection_options,
        optimization_problem.get());
  } else {
    map_optimization::solve(
        solver_options_with_callbacks, optimization_problem.get());
  }

  if (plotter_ != nullptr) {
    plotter_->visualizeMap(*map);
  }
  return true;
}

}  // namespace map_optimization
