#ifndef ASLAM_CALIBRATION_CAMERA_INITIALIZER_TRAITS_H
#define ASLAM_CALIBRATION_CAMERA_INITIALIZER_TRAITS_H

#include <aslam/cameras/camera-factory.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera-unified-projection.h>
#include <aslam/cameras/distortion-equidistant.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/cameras/distortion-radtan.h>

namespace aslam {
namespace calibration {

template<typename CameraType, typename DistortionType> struct FocalLengthInitializer {
  // Catch all template to fail on unimplemented projection / distortion model combinations.
  static bool initialize(
      const std::vector<TargetObservation::Ptr>& observations,
      Eigen::VectorXd* intrinsics) {
   LOG(FATAL) << "Initialization not implemented for this camera type";
   return false;
  }
};

#define REGISTER_CAMERA_INTRINSICS_INITIALIZER(CAMERA_MODEL, DISTORTION_MODEL, INITIALIZER_FCT) \
  template<>                                                                                    \
  struct FocalLengthInitializer<CAMERA_MODEL, DISTORTION_MODEL> {                               \
    static bool initialize(const std::vector<TargetObservation::Ptr>& observations,             \
                           Eigen::VectorXd* intrinsics) {                                       \
      CHECK_NOTNULL(intrinsics);                                                                \
      return INITIALIZER_FCT(observations, intrinsics);                                         \
    }                                                                                           \
  }


// Hookup each projection/camera model pair with the best suited initialization algorithm.
REGISTER_CAMERA_INTRINSICS_INITIALIZER(aslam::PinholeCamera, aslam::EquidistantDistortion,
                                       aslam::calibration::initFocalLengthVanishingPoints);
REGISTER_CAMERA_INTRINSICS_INITIALIZER(aslam::PinholeCamera, aslam::RadTanDistortion,
                                       aslam::calibration::initFocalLengthVanishingPoints);
REGISTER_CAMERA_INTRINSICS_INITIALIZER(aslam::PinholeCamera, aslam::NullDistortion,
                                       aslam::calibration::initFocalLengthAbsoluteConic);

}  // namespace calibration
}  // namespace aslam

#endif  // ASLAM_CALIBRATION_CAMERA_INITIALIZER_TRAITS_H
