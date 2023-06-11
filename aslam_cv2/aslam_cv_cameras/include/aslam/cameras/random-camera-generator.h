#ifndef ASLAM_RANDOM_CAMERA_GENERATOR_H_
#define ASLAM_RANDOM_CAMERA_GENERATOR_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <aslam/common/macros.h>
#include <aslam/common/pose-types.h>
#include <aslam/common/sensor.h>
#include <aslam/common/unique-id.h>
#include <aslam/common/yaml-serialization.h>
#include <gtest/gtest_prod.h>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera-unified-projection.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/ncamera.h>

namespace aslam {

/// Create a test NCamera object for unit testing.
NCamera::Ptr createTestNCamera(size_t num_cameras);
NCamera::UniquePtr createUniqueTestNCamera(size_t num_cameras);

/// Creates an artificial 4-camera rig in a plane with a camera pointing in
/// each direction. (similar to the V-Charge or JanETH camera system)
NCamera::Ptr createSurroundViewTestNCamera();
NCamera::UniquePtr createSurroundViewUniqueTestNCamera();

/// \brief Create a test camera object for intrinsics unit testing. (with null
/// distortion)
template <typename DistortionType>
PinholeCamera::Ptr createIntrinsicsPinholeTestCamera() {
  Distortion::UniquePtr zeros = DistortionType::createZeroTestDistortion();
  PinholeCamera::Ptr camera(
      new PinholeCamera(400, 400, 319.5, 239.5, 640, 480, zeros));
  CameraId id;
  generateId(&id);
  camera->setId(id);
  return camera;
}

}  // namespace aslam

#endif /* ASLAM_RANDOM_CAMERA_GENERATOR_H_ */
