#include "ceres-error-terms/problem-information.h"

#include <map>
#include <vector>

#include <ceres/problem.h>
#include <glog/logging.h>
#include <maplab-common/accessors.h>

#include "ceres-error-terms/common.h"

namespace ceres_error_terms {

ceres::Problem::Options getDefaultProblemOptions() {
  ceres::Problem::Options options;
  options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  return options;
}

void buildCeresProblemFromProblemInformation(
    ProblemInformation* problem_information, ceres::Problem* problem) {
  CHECK_NOTNULL(problem_information);
  CHECK_NOTNULL(problem);

  std::vector<double*> parameter_blocks;
  problem->GetParameterBlocks(&parameter_blocks);
  CHECK(parameter_blocks.empty())
      << "Problem passed to buildProblem must be empty.";

  VLOG(3) << "Build problem from "
          << problem_information->residual_blocks.size()
          << " active residual blocks.";

  // Add all residual blocks to the ceres Problem.
  for (ProblemInformation::ResidualInformationMap::value_type&
           residual_information_item : problem_information->residual_blocks) {
    ResidualInformation& residual_information =
        residual_information_item.second;
    if (!residual_information.active_) {
      continue;
    }
    ceres::ResidualBlockId residual_block_id = problem->AddResidualBlock(
        residual_information.cost_function.get(),
        residual_information.loss_function.get(),
        residual_information.parameter_blocks);
    residual_information.latest_residual_block_id = residual_block_id;
  }

  // Set specified parameter block constant.
  for (double* value : problem_information->constant_parameter_blocks) {
    if (problem->HasParameterBlock(value)) {
      problem->SetParameterBlockConstant(value);
    } else {
      LOG(WARNING)
          << "Parameter block " << value << " is in the constant "
          << "blocks of the problem information, but it is not present "
          << "in the ceres problem, which means it was not present in the "
          << "active residual blocks of the problem information.";
    }
  }

  for (const ProblemInformation::ParameterBoundMap::value_type& bound_info :
       problem_information->parameter_bounds) {
    if (problem->HasParameterBlock(bound_info.first)) {
      problem->SetParameterLowerBound(
          bound_info.first, bound_info.second.index_in_param_block,
          bound_info.second.lower_bound);
      problem->SetParameterUpperBound(
          bound_info.first, bound_info.second.index_in_param_block,
          bound_info.second.upper_bound);
    } else {
      LOG(WARNING)
          << "Parameter block " << bound_info.first << " has upper/lower bound "
          << "information associated, but it is not present "
          << "in the ceres problem, which means it was not present in the "
          << "active residual blocks of the problem information.";
    }
  }

  // Set local parameterizations for all parameter blocks.
  for (const std::pair<double*, std::shared_ptr<ceres::LocalParameterization>>
           parameterization : problem_information->parameterizations) {
    if (problem->HasParameterBlock(parameterization.first)) {
      problem->SetParameterization(
          parameterization.first, parameterization.second.get());
    } else {
      LOG(WARNING)
          << "Parameter block " << parameterization.first
          << " has a parametrization, but the block is not present "
          << "in the ceres problem, which means it was not present in the "
          << "active residual blocks of the problem information.";
    }
  }
}
}  // namespace ceres_error_terms
