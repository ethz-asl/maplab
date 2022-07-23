#ifndef CERES_ERROR_TERMS_VISUAL_ERROR_TERM_FACTORY_INL_H_
#define CERES_ERROR_TERMS_VISUAL_ERROR_TERM_FACTORY_INL_H_

#include <vector>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera-unified-projection.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/distortion-equidistant.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/cameras/distortion-null.h>
#include <aslam/cameras/distortion-radtan.h>
#include <aslam/cameras/distortion.h>
#include <glog/logging.h>

#include "ceres-error-terms/common.h"

namespace ceres_error_terms {

template <template <typename, typename> class ErrorTerm>
ceres::CostFunction* createVisualCostFunction(
    const Eigen::Vector2d& measurement, double pixel_sigma,
    ceres_error_terms::LandmarkErrorType error_term_type,
    aslam::Camera* camera) {
  CHECK_NOTNULL(camera);
  ceres::CostFunction* error_term = nullptr;
  switch (camera->getType()) {
    case aslam::Camera::Type::kPinhole: {
      aslam::PinholeCamera* derived_camera =
          static_cast<aslam::PinholeCamera*>(camera);
      aslam::Distortion::Type distortion_type =
          camera->getDistortion().getType();
      switch (distortion_type) {
        case aslam::Distortion::Type::kNoDistortion:
          error_term =
              new ErrorTerm<aslam::PinholeCamera, aslam::NullDistortion>(
                  measurement, pixel_sigma, error_term_type, derived_camera);
          break;
        case aslam::Distortion::Type::kEquidistant:
          error_term =
              new ErrorTerm<aslam::PinholeCamera, aslam::EquidistantDistortion>(
                  measurement, pixel_sigma, error_term_type, derived_camera);
          break;
        case aslam::Distortion::Type::kRadTan:
          error_term =
              new ErrorTerm<aslam::PinholeCamera, aslam::RadTanDistortion>(
                  measurement, pixel_sigma, error_term_type, derived_camera);
          break;
        case aslam::Distortion::Type::kFisheye:
          error_term =
              new ErrorTerm<aslam::PinholeCamera, aslam::FisheyeDistortion>(
                  measurement, pixel_sigma, error_term_type, derived_camera);
          break;
        default:
          LOG(FATAL) << "Invalid camera distortion type for ceres error term: "
                     << static_cast<int>(distortion_type);
      }
      break;
    }
    case aslam::Camera::Type::kUnifiedProjection: {
      aslam::UnifiedProjectionCamera* derived_camera =
          static_cast<aslam::UnifiedProjectionCamera*>(camera);
      aslam::Distortion::Type distortion_type =
          camera->getDistortion().getType();
      switch (distortion_type) {
        case aslam::Distortion::Type::kNoDistortion:
          error_term = new ErrorTerm<aslam::UnifiedProjectionCamera,
                                     aslam::NullDistortion>(
              measurement, pixel_sigma, error_term_type, derived_camera);
          break;
        case aslam::Distortion::Type::kEquidistant:
          error_term = new ErrorTerm<aslam::UnifiedProjectionCamera,
                                     aslam::EquidistantDistortion>(
              measurement, pixel_sigma, error_term_type, derived_camera);
          break;
        case aslam::Distortion::Type::kRadTan:
          error_term = new ErrorTerm<aslam::UnifiedProjectionCamera,
                                     aslam::RadTanDistortion>(
              measurement, pixel_sigma, error_term_type, derived_camera);
          break;
        case aslam::Distortion::Type::kFisheye:
          error_term = new ErrorTerm<aslam::UnifiedProjectionCamera,
                                     aslam::FisheyeDistortion>(
              measurement, pixel_sigma, error_term_type, derived_camera);
          break;
        default:
          LOG(FATAL) << "Invalid camera distortion type for ceres error term: "
                     << static_cast<int>(distortion_type);
      }
      break;
    }
    default:
      LOG(FATAL) << "Invalid camera projection type for ceres error term: "
                 << static_cast<int>(camera->getType());
  }
  return error_term;
}

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_VISUAL_ERROR_TERM_FACTORY_INL_H_
