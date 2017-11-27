#ifndef CERES_ERROR_TERMS_PROBLEM_INFORMATION_H_
#define CERES_ERROR_TERMS_PROBLEM_INFORMATION_H_

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <ceres/problem.h>
#include <maplab-common/accessors.h>

#include "ceres-error-terms/common.h"
#include "ceres-error-terms/residual-information.h"

namespace ceres_error_terms {

struct ProblemInformation {
  void clearProblemInformation() {
    residual_blocks.clear();
    parameterizations.clear();
    constant_parameter_blocks.clear();
    constant_parameter_blocks.clear();
    parameter_block_group_id.clear();
  }

  void addResidualBlock(
      ResidualType residual_type,
      std::shared_ptr<ceres::CostFunction> cost_function,
      std::shared_ptr<ceres::LossFunction> loss_function,
      const std::vector<double*>& parameter_blocks) {
    ResidualInformation residual_info(
        residual_type, cost_function, loss_function, parameter_blocks);
    CHECK(residual_blocks.emplace(cost_function.get(), residual_info).second);
    active_parameter_blocks.insert(
        parameter_blocks.begin(), parameter_blocks.end());
  }

  void setParameterization(
      double* parameter_block,
      const std::shared_ptr<ceres::LocalParameterization>& parameterization) {
    CHECK(parameterization);
    std::pair<ParameterizationsMap::iterator, bool> it_inserted =
        parameterizations.emplace(parameter_block, parameterization);
    CHECK(
        it_inserted.second ||
        it_inserted.first->second.get() == parameterization.get())
        << "Tried to replace already registered parameterization "
        << it_inserted.first->second.get() << " of block " << parameter_block
        << " with parameterization " << parameterization.get() << ". "
        << "Use replaceParameterization(..) to perform this operation.";
  }

  void replaceParameterization(
      double* parameter_block,
      const std::shared_ptr<ceres::LocalParameterization>& parameterization) {
    CHECK_NOTNULL(parameter_block);
    common::getChecked(parameterizations, parameter_block) = parameterization;
  }

  // Will not fail if the parameterization was already set.
  void setParameterizationNonChecked(
      double* parameter_block,
      const std::shared_ptr<ceres::LocalParameterization>& parameterization) {
    CHECK_NOTNULL(parameter_block);
    parameterizations[parameter_block] = parameterization;
  }

  // Returns true if it has been set constant.
  bool setParameterBlockConstantIfPartOfTheProblem(double* parameter_block) {
    CHECK_NOTNULL(parameter_block);
    if (active_parameter_blocks.count(parameter_block) == 0u) {
      return false;
    }
    constant_parameter_blocks.insert(parameter_block);
    return true;
  }

  void setParameterBlockConstant(double* parameter_block) {
    CHECK_NOTNULL(parameter_block);
    constant_parameter_blocks.insert(parameter_block);
  }

  void setParameterBlockVariable(double* parameter_block) {
    CHECK_NOTNULL(parameter_block);
    constant_parameter_blocks.erase(parameter_block);
  }

  bool isParameterBlockConstant(double* parameter_block) const {
    CHECK_NOTNULL(parameter_block);
    return constant_parameter_blocks.count(parameter_block) > 0u;
  }

  struct ParameterBoundInformation {
    int index_in_param_block;
    double lower_bound;
    double upper_bound;
  };

  void setParameterBlockBounds(
      int index_in_param_block, double lower_bound, double upper_bound,
      double* parameter_block) {
    CHECK_NOTNULL(parameter_block);
    CHECK_GT(upper_bound, lower_bound);
    CHECK_GE(index_in_param_block, 0);

    ParameterBoundInformation bound_info;
    bound_info.index_in_param_block = index_in_param_block;
    bound_info.lower_bound = lower_bound;
    bound_info.upper_bound = upper_bound;

    parameter_bounds.emplace(parameter_block, bound_info);
  }

  void setParameterBlockGroupId(double* parameter_block, int parameter_id) {
    CHECK_NOTNULL(parameter_block);
    CHECK_GE(parameter_id, 0);
    const bool inserted =
        parameter_block_group_id.emplace(parameter_block, parameter_id).second;
    VLOG_IF(50, !inserted) << "Group id of parameter block " << parameter_block
                           << " was already set.";
  }

  /// Returns false if no groupid is associated with the parameter block.
  bool getParameterBlockGroupId(const double* parameter_block, int* group_id) {
    CHECK_NOTNULL(parameter_block);
    CHECK_NOTNULL(group_id);
    *group_id = common::getValueOrDefault(
        parameter_block_group_id, parameter_block, kDefaultParameterBlockId);
    return (*group_id != kDefaultParameterBlockId);
  }

  void deactivateCostFunction(ceres::CostFunction* cost_function) {
    CHECK_NOTNULL(cost_function);
    ResidualInformation& residual_info =
        common::getChecked(residual_blocks, cost_function);
    residual_info.active_ = false;

    std::for_each(
        residual_info.parameter_blocks.begin(),
        residual_info.parameter_blocks.end(),
        [this](double* param) { this->active_parameter_blocks.erase(param); });
  }

  typedef std::unordered_map<ceres::CostFunction*, ResidualInformation>
      ResidualInformationMap;
  ResidualInformationMap residual_blocks;

  std::unordered_set<double*> active_parameter_blocks;
  std::unordered_set<double*> constant_parameter_blocks;

  typedef std::unordered_map<double*, ParameterBoundInformation>
      ParameterBoundMap;
  ParameterBoundMap parameter_bounds;

  typedef std::unordered_map<double*,
                             std::shared_ptr<ceres::LocalParameterization> >
      ParameterizationsMap;
  ParameterizationsMap parameterizations;

  // A group type associated with each parameter block. This is required by
  // special solvers that treat certain parameter groups differently. -1 is
  // the default group.
  static constexpr int kDefaultParameterBlockId = -1;
  std::unordered_map<const double*, int> parameter_block_group_id;
};

ceres::Problem::Options getDefaultProblemOptions();

void buildCeresProblemFromProblemInformation(
    ProblemInformation* problem_information, ceres::Problem* problem);

// Build an optimization problem that orders the parameter using their group
// ids. Parameters with an associated group id will be placed last.
// ordered_group_start_index is the index where the ordered group starts.
// Parameter vector: [kDefaultGroupId,
//                    params(groupid_order[0]),  <-- ordered_group_start_index
//                    ...,                           (parameter index NOT block
// index!)
//                    params(groupid_order[n])]
//
// Example:                   | Output:
// -----------------------------------------------------------------------
//  block   params   groupid  |   groupid_ordering = {2, 1}
//    0    (x0, x1)    -1     |    -->
//    1    (x2, x3)     0     |   state: [x0,x1,x2,x3,x6,x4,x5],
//    2    (x4, x5)     1     |                       ^
//    3       (x6)      2     |           ordered_group_start_index = 4
//
// The ordered_group_start_index is set to -1 if all parameters are nuisance
// variables.
void buildOrderedCeresProblemFromProblemInformation(
    ProblemInformation* problem_information,
    const std::vector<int>& groupid_ordering, int* ordered_group_start_index,
    ceres::Problem* problem);

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_PROBLEM_INFORMATION_H_
