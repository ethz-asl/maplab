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
};

ceres::Problem::Options getDefaultProblemOptions();

void buildCeresProblemFromProblemInformation(
    ProblemInformation* problem_information, ceres::Problem* problem);

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_PROBLEM_INFORMATION_H_
