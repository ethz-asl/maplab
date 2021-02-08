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
    const vi_map::MissionIdSet& missions_to_optimize, vi_map::VIMap* map) {
  return optimize(options, missions_to_optimize, map, nullptr /*result*/);
}

bool VIMapOptimizer::optimize(
    const map_optimization::ViProblemOptions& options,
    const vi_map::MissionIdSet& missions_to_optimize, vi_map::VIMap* map,
    OptimizationProblemResult* result) {
  // 'result' can be a nullptr.
  CHECK_NOTNULL(map);

  if (missions_to_optimize.empty()) {
    LOG(WARNING) << "Nothing to optimize.";
    return false;
  }

  map_optimization::OptimizationProblem::UniquePtr optimization_problem(
      map_optimization::constructOptimizationProblem(
          missions_to_optimize, options, map));
  CHECK(optimization_problem);

  auto v_key = optimization_problem->getProblemBookkeepingMutable()
                   ->keyframes_in_problem;
  auto r_map =
      optimization_problem->getProblemInformationMutable()->residual_blocks;
  VLOG(1) << "-----------------------------------------------------------------"
             "-----------------------------------------------------------------"
             "------------";
  VLOG(1) << v_key.size() << " vs. " << r_map.size();
  LOG(ERROR) << v_key.size() << " vs. " << r_map.size();

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
  ceres::Solver::Options solver_options_with_callbacks = options.solver_options;
  map_optimization::addCallbacksToSolverOptions(
      callbacks, &solver_options_with_callbacks);

  if (options.enable_visual_outlier_rejection) {
    map_optimization::solveWithOutlierRejection(
        solver_options_with_callbacks, options.visual_outlier_rejection_options,
        optimization_problem.get(), result);
  } else {
    map_optimization::solve(
        solver_options_with_callbacks, optimization_problem.get(), result);
  }

  if (plotter_ != nullptr) {
    plotter_->visualizeMap(*map);
  }

  return true;
}

bool VIMapOptimizer::optimizeWithCov(
    const map_optimization::ViProblemOptions& options,
    const vi_map::MissionIdSet& missions_to_optimize,
    const pose_graph::VertexIdList& vertices, vi_map::VIMap* map,
    OptimizationProblemResult* result) {
  // 'result' can be a nullptr.
  CHECK_NOTNULL(map);

  if (missions_to_optimize.empty()) {
    LOG(WARNING) << "Nothing to optimize.";
    return false;
  }

  map_optimization::OptimizationProblem::UniquePtr optimization_problem(
      map_optimization::constructOptimizationProblem(
          missions_to_optimize, options, map));
  CHECK(optimization_problem);

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
  ceres::Solver::Options solver_options_with_callbacks = options.solver_options;
  map_optimization::addCallbacksToSolverOptions(
      callbacks, &solver_options_with_callbacks);

  if (options.enable_visual_outlier_rejection) {
    map_optimization::solveWithOutlierRejection(
        solver_options_with_callbacks, options.visual_outlier_rejection_options,
        optimization_problem.get(), result);
  } else {
    map_optimization::solve(
        solver_options_with_callbacks, optimization_problem.get(), result);
  }

  if (plotter_ != nullptr) {
    plotter_->visualizeMap(*map);
  }

  /*
  ceres::Covariance::Options co_options;
  co_options.num_threads = 12;
  // co_options.sparse_linear_algebra_library_type =
  // ceres::SUITE_SPARSE;
  co_options.algorithm_type = ceres::DENSE_SVD;  // Jacobian is difficient
  // co_options.algorithm_type = ceres::SPARSE_QR;
  co_options.min_reciprocal_condition_number = 1e-6;  // only for DENSE_SVD
  co_options.null_space_rank = -1;                    // only for DENSE_SVD
  // options.apply_loss_function = true;
  ceres::Covariance covariance(co_options);
  std::vector<std::pair<const double*, const double*>> covariance_blocks;
  const auto& v = vertices.back();
  if (!map->hasVertex(v)) {
    return true;
  }
  double* vertex_q_IM__M_p_MI_JPL = optimization_problem.get()
                                        ->getOptimizationStateBufferMutable()
                                        ->get_vertex_q_IM__M_p_MI_JPL(v);
  covariance_blocks.emplace_back(
      std::make_pair(vertex_q_IM__M_p_MI_JPL, vertex_q_IM__M_p_MI_JPL));

  ceres::Problem problem(ceres_error_terms::getDefaultProblemOptions());
  ceres_error_terms::buildCeresProblemFromProblemInformation(
      optimization_problem->getProblemInformationMutable(), &problem);

  CHECK(covariance.Compute(covariance_blocks, &problem));

  std::vector<Eigen::MatrixXd> covariances;
  double cov[7 * 7];
  covariance.GetCovarianceBlock(
      vertex_q_IM__M_p_MI_JPL, vertex_q_IM__M_p_MI_JPL, cov);
  Eigen::MatrixXd covE = Eigen::Map<Eigen::MatrixXd>(cov, 7, 7);

  VLOG(1) << "----------------------------------------------------";
  VLOG(1) << "COV:\n" << covE << "\n";
  VLOG(1) << "----------------------------------------------------";

  covariances.emplace_back(covE);
  */

  return true;
}

std::vector<Eigen::MatrixXd> VIMapOptimizer::getCovarianceForVertices(
    const map_optimization::ViProblemOptions& options,
    const vi_map::MissionIdSet& missions_to_optimize,
    const pose_graph::VertexIdList& vertices, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  if (missions_to_optimize.empty()) {
    LOG(WARNING) << "Nothing to optimize.";
    return {};
  }

  /*
  // compute covariance of next representative w.r.t. current representative
  ceres::Covariance::Options co_options;
  co_options.num_threads = 12;
  // co_options.sparse_linear_algebra_library_type =
  // ceres::SUITE_SPARSE;
  co_options.algorithm_type = ceres::DENSE_SVD;  // Jacobian is difficient
  // co_options.algorithm_type = ceres::SPARSE_QR;
  co_options.min_reciprocal_condition_number = 1e-10;  // only for DENSE_SVD
  co_options.null_space_rank = -1;                     // only for DENSE_SVD
  // options.apply_loss_function = true;
  ceres::Covariance covariance(co_options);


  std::vector<std::pair<const double*, const double*>> covariance_blocks;
  */
  /*
  for (const auto& v : vertices) {
    double* vertex_q_IM__M_p_MI_JPL = optimization_problem.get()
                                          ->getOptimizationStateBufferMutable()
                                          ->get_vertex_q_IM__M_p_MI_JPL(v);
    covariance_blocks.emplace_back(
        std::make_pair(vertex_q_IM__M_p_MI_JPL, vertex_q_IM__M_p_MI_JPL));
  double* vertex_q_IM__M_p_MI_JPL = optimization_problem.get()
                                        ->getOptimizationStateBufferMutable()
                                        ->get_vertex_q_IM__M_p_MI_JPL(v);
  covariance_blocks.emplace_back(
      std::make_pair(vertex_q_IM__M_p_MI_JPL, vertex_q_IM__M_p_MI_JPL));

  }*/
  map_optimization::OptimizationProblem::UniquePtr optimization_problem(
      map_optimization::constructOptimizationProblem(
          missions_to_optimize, options, map));
  CHECK(optimization_problem);
  const auto& v = vertices.back();
  if (!map->hasVertex(v)) {
    return {};
  }
  const vi_map::Vertex& ba_vertex = map->getVertex(v);
  vi_map::LandmarkIdList observed_landmarks;
  ba_vertex.getStoredLandmarkIdList(&observed_landmarks);

  const auto& l = observed_landmarks.back();
  auto landmark_map = optimization_problem->getProblemBookkeepingMutable()
                          ->landmarks_in_problem;
  auto r_map =
      optimization_problem->getProblemInformationMutable()->residual_blocks;
  auto range = landmark_map.equal_range(l);
  std::vector<ceres::ResidualBlockId> to_eval;
  for (auto it = range.first; it != range.second; ++it) {
    auto cost_f = it->second;
    if (r_map[cost_f].active_) {
      to_eval.emplace_back(r_map[cost_f].latest_residual_block_id);
    }
  }

  ceres::Problem::EvaluateOptions eval_options;
  eval_options.residual_blocks = to_eval;

  ceres::Problem problem(ceres_error_terms::getDefaultProblemOptions());
  ceres_error_terms::buildCeresProblemFromProblemInformation(
      optimization_problem->getProblemInformationMutable(), &problem);
  double total_cost = 0;
  std::vector<double> evaluated_residuals;
  problem.Evaluate(
      eval_options, &total_cost, &evaluated_residuals, nullptr, nullptr);

  for (const double eval : evaluated_residuals) {
    LOG(ERROR) << "--- EVAL: " << eval;
  }

  /*
  CHECK(covariance.Compute(covariance_blocks, &problem));

  std::vector<Eigen::MatrixXd> covariances;
  double cov[7 * 7];
  covariance.GetCovarianceBlock(
      vertex_q_IM__M_p_MI_JPL, vertex_q_IM__M_p_MI_JPL, cov);
  Eigen::MatrixXd covE = Eigen::Map<Eigen::MatrixXd>(cov, 7, 7);
  covariances.emplace_back(covE);
  for (const auto& v : vertices) {
    double* vertex_q_IM__M_p_MI_JPL = optimization_problem.get()
                                          ->getOptimizationStateBufferMutable()
                                          ->get_vertex_q_IM__M_p_MI_JPL(v);
    double cov[7 * 7];
    covariance.GetCovarianceBlock(
        vertex_q_IM__M_p_MI_JPL, vertex_q_IM__M_p_MI_JPL, cov);
    Eigen::MatrixXd covariance = Eigen::Map<Eigen::MatrixXd>(cov, 7, 7);
    covariances.emplace_back(covariance);
  }*/
  std::vector<Eigen::MatrixXd> covariances;
  return covariances;
}

}  // namespace map_optimization
