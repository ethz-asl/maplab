#include "ceres-error-terms/problem-information.h"

#include <map>
#include <vector>

#include <ceres/problem.h>
#include <glog/logging.h>
#include <maplab-common/accessors.h>

#include "ceres-error-terms/common.h"

namespace ceres_error_terms {

constexpr int ProblemInformation::kDefaultParameterBlockId;

ceres::Problem::Options getDefaultProblemOptions() {
  ceres::Problem::Options options;
  options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  return options;
}

void buildOrderedCeresProblemFromProblemInformation(
    ProblemInformation* problem_information,
    const std::vector<int>& groupid_ordering, int* ordered_group_start_index,
    ceres::Problem* problem) {
  CHECK_NOTNULL(problem_information);
  CHECK_NOTNULL(problem);
  CHECK_NOTNULL(ordered_group_start_index);

  std::vector<double*> parameter_blocks;
  problem->GetParameterBlocks(&parameter_blocks);
  CHECK(parameter_blocks.empty())
      << "Problem passed to buildProblem must be empty.";

  VLOG(3) << "Build problem from "
          << problem_information->residual_blocks.size()
          << " active residual blocks.";

  // Add all parameter blocks to the problem. Usually this step is not
  // required as all blocks are added along with the residual blocks.
  // Here this is done manually to have control over the implicit parameter
  // ordering determined by the underlying parameter vector.
  if (!groupid_ordering.empty()) {
    VLOG(3) << "Enforcing a special groupid ordering using "
            << groupid_ordering.size() << " groups.";

    struct ParamBlockAndSize {
      ParamBlockAndSize(double* _param_block, size_t _block_size)
          : param_block(_param_block), block_size(_block_size) {
        CHECK_NOTNULL(_param_block);
        CHECK_GT(_block_size, 0u);
      }
      double* param_block;
      size_t block_size;

      inline bool operator<(const ParamBlockAndSize& other) const {
        return (param_block < other.param_block);
      }
    };
    typedef std::map<int, std::set<ParamBlockAndSize>> GroupIdParamBlockSetMap;
    GroupIdParamBlockSetMap parameter_blocks_with_ordering;
    std::unordered_set<double*> inserted_param_blocks;

    // Go over all residual blocks and add the associated parameter blocks
    // if it is not assigned to a groupid with associated ordering. Keep
    // all residual blocks with associated ordering for later insertion.
    *ordered_group_start_index = 0;
    for (ProblemInformation::ResidualInformationMap::value_type&
             residual_information_item : problem_information->residual_blocks) {
      ResidualInformation& residual_information =
          residual_information_item.second;

      if (!residual_information.active_) {
        continue;
      }

      // Go over all parameter blocks in this residual block.
      const size_t num_param_blocks =
          residual_information.parameter_blocks.size();

      for (size_t block_idx = 0; block_idx < num_param_blocks; ++block_idx) {
        double* parameter_block =
            CHECK_NOTNULL(residual_information.parameter_blocks[block_idx]);
        const int block_size = residual_information.cost_function
                                   ->parameter_block_sizes()[block_idx];

        int parameter_block_groupid;
        const bool parameter_block_has_groupid =
            problem_information->getParameterBlockGroupId(
                parameter_block, &parameter_block_groupid);

        // Add block if no special ordering is requested, otherwise queue them
        // for later insertion.
        if (!parameter_block_has_groupid ||
            !common::containsValue(groupid_ordering, parameter_block_groupid)) {
          // Make sure that we only add each parameter block once.
          if (inserted_param_blocks.emplace(parameter_block).second) {
            problem->AddParameterBlock(parameter_block, block_size);
            *ordered_group_start_index += block_size;
          }
        } else {
          // Keep the parameter blocks with special ordering requirements.
          parameter_blocks_with_ordering[parameter_block_groupid].emplace(
              ParamBlockAndSize(parameter_block, block_size));
        }
      }
    }

    // Now add all parameter blocks with special ordering in ascending groupid
    // ordering.
    for (const GroupIdParamBlockSetMap::value_type& groupid_paramblockset :
         parameter_blocks_with_ordering) {
      for (const ParamBlockAndSize& paramblock_size :
           groupid_paramblockset.second) {
        problem->AddParameterBlock(
            paramblock_size.param_block, paramblock_size.block_size);
        inserted_param_blocks.emplace(paramblock_size.param_block);
      }
    }
  } else {
    // No parameters are in the calibration group.
    *ordered_group_start_index = -1;
  }

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

void buildCeresProblemFromProblemInformation(
    ProblemInformation* problem_information, ceres::Problem* problem) {
  CHECK_NOTNULL(problem_information);
  CHECK_NOTNULL(problem);

  int ordered_group_start_index;
  buildOrderedCeresProblemFromProblemInformation(
      problem_information, std::vector<int>(), &ordered_group_start_index,
      problem);
}
}  // namespace ceres_error_terms
