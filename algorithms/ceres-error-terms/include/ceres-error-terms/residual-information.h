#ifndef CERES_ERROR_TERMS_RESIDUAL_INFORMATION_H_
#define CERES_ERROR_TERMS_RESIDUAL_INFORMATION_H_

#include <memory>
#include <vector>

#include <ceres/problem.h>
#include <maplab-common/accessors.h>

#include "ceres-error-terms/common.h"

namespace ceres_error_terms {

enum class ResidualType {
  kInvalid,
  kVisualReprojectionError,
  kVisualBearingError,
  kInertial,
  kPosePrior,
  k6DoFGPS,
  kOdometry,
  kLoopClosure,
  kSwitchVariable,
  kVelocityPrior,
  k3DoFGPS,
  kGenericPrior,
};

struct ResidualInformation {
  ResidualInformation()
      : latest_residual_block_id(nullptr),
        residual_type(ResidualType::kInvalid),
        active_(true) {}
  ResidualInformation(
      ResidualType _residual_type,
      std::shared_ptr<ceres::CostFunction> _cost_function,
      std::shared_ptr<ceres::LossFunction> _loss_function,
      const std::vector<double*>& _parameter_blocks)
      : latest_residual_block_id(nullptr),
        residual_type(_residual_type),
        cost_function(_cost_function),
        loss_function(_loss_function),
        parameter_blocks(_parameter_blocks),
        active_(true) {}

  ~ResidualInformation() {}

  ceres::ResidualBlockId latest_residual_block_id;
  ResidualType residual_type;
  std::shared_ptr<ceres::CostFunction> cost_function;
  std::shared_ptr<ceres::LossFunction> loss_function;
  std::vector<double*> parameter_blocks;
  bool active_;
};

struct ResidualTypeHash {
  std::size_t operator()(const ResidualType& residual_type) const {
    return static_cast<std::size_t>(residual_type);
  }
};

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_RESIDUAL_INFORMATION_H_
