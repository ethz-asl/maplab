#ifndef MAP_OPTIMIZATION_LEGACY_GRAPH_ITERATION_CALLBACK_H_
#define MAP_OPTIMIZATION_LEGACY_GRAPH_ITERATION_CALLBACK_H_

#include <cstdlib>
#include <iomanip>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/statistics/accumulator.h>
#include <ceres-error-terms/common.h>
#include <ceres-error-terms/problem-information.h>
#include <ceres/ceres.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/threading-helpers.h>
#include <vi-map/landmark.h>
#include <vi-map/mission-baseframe.h>
#include <vi-map/pose-graph.h>
#include <vi-map/unique-id.h>

#include "map-optimization-legacy/ba-optimization-options.h"
#include "map-optimization-legacy/graph-ba-optimizer.h"

namespace map_optimization_legacy {
template <typename CallbackType>
class GraphIterationCallback : public ceres::IterationCallback {
 public:
  typedef vi_map::MissionBaseFrameMap MissionBaseFrameMap;

  explicit GraphIterationCallback(
      CallbackType callback, const vi_map::VIMap& map,
      GraphBaOptimizer* optimizer)
      : optimizer_(CHECK_NOTNULL(optimizer)), callback_(callback), map_(map) {}

  explicit GraphIterationCallback(
      std::function<void(const vi_map::VIMap&)> callback,
      const vi_map::VIMap& map)
      : optimizer_(nullptr), callback_(callback), map_(map) {}

  virtual ~GraphIterationCallback() {}

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
    if ((summary.iteration %
         options_.update_visualization_every_n_iterations) == 0 &&
        summary.step_is_successful && summary.step_is_valid) {
      if (optimizer_) {
        optimizer_->copyDataToMap();
      }
      callback_(map_);
    }

    if (options_.show_residual_statistics && optimizer_ != nullptr) {
      // Used for residual block statistics.
      ceres::Problem::EvaluateOptions evaluate_options;
      std::vector<ceres_error_terms::ResidualType> residual_types;
      std::unordered_map<ceres_error_terms::ResidualType, size_t,
                         ceres_error_terms::ResidualTypeHash>
          residual_sizes;

      // We need to initialize, i.e. use problem information to obtain
      // residual block ids, residual block types and sizes.
      evaluate_options.apply_loss_function =
          options_.apply_loss_function_for_residual_stats;
      evaluate_options.num_threads = common::getNumHardwareThreads();

      const ceres_error_terms::ProblemInformation& problem_information =
          optimizer_->getProblemInformation();

      residual_types.reserve(problem_information.residual_blocks.size());
      evaluate_options.residual_blocks.reserve(
          problem_information.residual_blocks.size());

      for (const ceres_error_terms::ProblemInformation::ResidualInformationMap::
               value_type& item : problem_information.residual_blocks) {
        const ceres_error_terms::ResidualInformation& residual = item.second;
        residual_types.push_back(residual.residual_type);
        evaluate_options.residual_blocks.push_back(
            residual.latest_residual_block_id);
        residual_sizes.emplace(
            residual.residual_type, residual.cost_function->num_residuals());
      }
      CHECK_EQ(evaluate_options.residual_blocks.size(), residual_types.size());

      std::shared_ptr<ceres::Problem> problem = optimizer_->getCeresProblem();
      CHECK(problem != nullptr);

      std::vector<double> residuals;
      // Note that Evaluate returns pointers to separate residuals and not
      // residual blocks, we will need to do bookkeeping ourselves.
      problem->Evaluate(
          evaluate_options, nullptr, &residuals, nullptr, nullptr);

      typedef statistics::Accumulator<double, double,
                                      statistics::kInfiniteWindowSize>
          ResidualStat;

      std::unordered_map<ceres_error_terms::ResidualType, ResidualStat,
                         ceres_error_terms::ResidualTypeHash>
          total_cost_per_residual_type;
      std::unordered_map<ceres_error_terms::ResidualType,
                         std::vector<ResidualStat>,
                         ceres_error_terms::ResidualTypeHash>
          detailed_cost_per_residual_type;
      std::unordered_map<ceres_error_terms::ResidualType,
                         std::vector<ResidualStat>,
                         ceres_error_terms::ResidualTypeHash>
          detailed_abs_cost_per_residual_type;
      std::unordered_map<ceres_error_terms::ResidualType, size_t,
                         ceres_error_terms::ResidualTypeHash>
          residual_type_count;
      std::unordered_set<ceres_error_terms::ResidualType,
                         ceres_error_terms::ResidualTypeHash>
          existing_residual_types;
      size_t residuals_index = 0u;
      for (size_t i = 0u; i < residual_types.size(); ++i) {
        const size_t residual_size = residual_sizes[residual_types[i]];

        // Note that if the vector is already resized, nothing will happen here.
        detailed_cost_per_residual_type[residual_types[i]].resize(
            residual_size);
        detailed_abs_cost_per_residual_type[residual_types[i]].resize(
            residual_size);

        for (size_t residual_idx = 0u; residual_idx < residual_size;
             ++residual_idx) {
          CHECK_LT(residuals_index, residuals.size());
          total_cost_per_residual_type[residual_types[i]].Add(
              std::fabs(residuals[residuals_index]));

          std::vector<ResidualStat>& detailed_residual_type_costs =
              detailed_cost_per_residual_type[residual_types[i]];
          std::vector<ResidualStat>& detailed_residual_type_abs_costs =
              detailed_abs_cost_per_residual_type[residual_types[i]];

          CHECK_LT(residual_idx, detailed_residual_type_costs.size());
          CHECK_LT(residual_idx, detailed_residual_type_abs_costs.size());

          detailed_residual_type_costs[residual_idx].Add(
              residuals[residuals_index]);
          detailed_residual_type_abs_costs[residual_idx].Add(
              std::fabs(residuals[residuals_index]));
          ++residuals_index;
        }

        ++(residual_type_count[residual_types[i]]);
        existing_residual_types.emplace(residual_types[i]);
      }
      CHECK_EQ(residuals_index, residuals.size());

      std::string loss_function_msg =
          (evaluate_options.apply_loss_function ? "with loss function"
                                                : "without loss function");

      LOG(INFO) << "Residual statistics " << loss_function_msg << ":";
      for (const ceres_error_terms::ResidualType& residual_type :
           existing_residual_types) {
        CHECK_GT(residual_type_count[residual_type], 0u);
        LOG(INFO) << "-----------------------------------------------------"
                  << "------";
        LOG(INFO) << "Type " << std::setw(2) << std::setfill(' ') << std::right
                  << static_cast<size_t>(residual_type) << std::setw(15)
                  << std::setfill(' ') << "Total cost: " << std::right
                  << total_cost_per_residual_type[residual_type].sum()
                  << std::setw(20) << std::setfill(' ')
                  << "Average abs cost: " << std::right << std::setprecision(3)
                  << total_cost_per_residual_type[residual_type].Mean();
        LOG(INFO) << std::endl;
        LOG(INFO) << "Element" << std::setw(18) << std::setfill(' ')
                  << "Average cost" << std::setw(11) << std::setfill(' ')
                  << "Std dev" << std::setw(22) << std::setfill(' ')
                  << "Average abs cost";
        LOG(INFO) << "-----------------------------------------------------"
                  << "------";

        const size_t residual_size = residual_sizes[residual_type];
        for (size_t i = 0; i < residual_size; ++i) {
          CHECK_LT(i, detailed_cost_per_residual_type[residual_type].size());
          CHECK_LT(
              i, detailed_abs_cost_per_residual_type[residual_type].size());
          LOG(INFO) << "  [" << std::setw(2) << std::setfill(' ') << i << "] "
                    << std::setw(14) << std::setfill(' ') << std::right
                    << std::setprecision(3)
                    << detailed_cost_per_residual_type[residual_type][i].Mean()
                    << std::setw(14) << std::setfill(' ') << std::right
                    << std::setprecision(3)
                    << detailed_cost_per_residual_type[residual_type]
                                                      [i].StandardDeviation()
                    << std::setw(14) << std::setfill(' ') << std::right
                    << std::setprecision(3)
                    << detailed_abs_cost_per_residual_type[residual_type]
                                                          [i].Mean();
        }
        LOG(INFO) << std::endl;
      }
    }
    return ceres::SOLVER_CONTINUE;
  }

 private:
  GraphBaOptimizer* optimizer_;
  std::function<void(const vi_map::VIMap&)> callback_;
  const vi_map::VIMap& map_;

  BaIterationOptions options_;
};

}  // namespace map_optimization_legacy

#endif  // MAP_OPTIMIZATION_LEGACY_GRAPH_ITERATION_CALLBACK_H_
