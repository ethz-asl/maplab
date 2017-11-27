#ifndef CERES_ERROR_TERMS_COMMON_H_
#define CERES_ERROR_TERMS_COMMON_H_

#include <memory>
#include <vector>

#include <ceres/cost_function.h>
#include <ceres/loss_function.h>
#include <ceres/problem.h>
#include <glog/logging.h>

#include "ceres-error-terms/visual-error-term-base.h"

namespace ceres_error_terms {

namespace visual {
enum VisualErrorType { kLocalKeyframe, kLocalMission, kGlobal };

inline bool isValidVisualErrorTermType(VisualErrorType type) {
  static const bool value =
      (type == VisualErrorType::kLocalKeyframe ||
       type == VisualErrorType::kLocalMission ||
       type == VisualErrorType::kGlobal);
  return value;
}

static const int kResidualSize = 2;
static const int kBearingBasedResidualSize = 1;

static const int kOrientationBlockSize = 4;
static const int kUnit3BlockSize = 4;
static const int kInverseDepthBlockSize = 1;
static const int kPositionBlockSize = 3;
static const int kPoseBlockSize = 7;
}  // namespace visual

namespace poseblocks {
static const int kResidualSize = 6;
static const int kOrientationBlockSize = 4;
static const int kPositionBlockSize = 3;
static const int kPoseSize = 7;
}  // namespace poseblocks

namespace positionblocks {
static const int kResidualSize = 3;
static const int kOrientationBlockSize = 4;
static const int kPositionBlockSize = 3;
static const int kPoseSize = 7;
}  // namespace positionblocks

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_COMMON_H_
