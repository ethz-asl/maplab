#ifndef ASLAM_CAMERAS_CAMERA_FACTORY_H_
#define ASLAM_CAMERAS_CAMERA_FACTORY_H_

#include <memory>
#include <type_traits>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <aslam/cameras/camera.h>
#include <aslam/cameras/distortion.h>
#include <aslam/common/unique-id.h>
#include <aslam/common/yaml-serialization.h>

namespace aslam {

/// \brief A factory function to create a derived class camera.
///
/// This function takes vectors of intrinsics and distortion parameters
/// and creates a camera.
/// \param[in] intrinsics A vector of projection intrinsic parameters.
/// \param[in] image_width Image width in pixels.
/// \param[in] image_height Image height in pixels.
/// \param[in] distortion_parameters The parameters of the distortion object.
/// \returns A new camera based on the template types.
template <typename CameraType, typename DistortionType>
typename CameraType::Ptr createCamera(const Eigen::VectorXd& intrinsics,
                                      uint32_t image_width, uint32_t image_height,
                                      const Eigen::VectorXd& distortion_parameters) {
  typename aslam::Distortion::UniquePtr distortion(new DistortionType(distortion_parameters));
  typename CameraType::Ptr camera(
      new CameraType(intrinsics, image_width, image_height, distortion));
  aslam::CameraId id;
  generateId(&id);
  camera->setId(id);
  return camera;
}

/// \brief A factory function to create a derived class camera without distortion.
///
/// This function takes vectors of intrinsics and distortion parameters
/// and creates a camera.
/// \param[in] intrinsics A vector of projection intrinsic parameters.
/// \param[in] image_width Image width in pixels.
/// \param[in] image_height Image height in pixels.
/// \returns A new camera based on the template types.
template <typename CameraType>
typename CameraType::Ptr createCamera(const Eigen::VectorXd& intrinsics,
                                      uint32_t image_width, uint32_t image_height) {
  typename CameraType::Ptr camera(new CameraType(intrinsics, image_width, image_height));
  aslam::CameraId id;
  generateId(&id);
  camera->setId(id);
  return camera;
}

/// \brief A factory function to create a derived class camera.
///
/// This function takes vectors of intrinsics and distortion parameters
/// and creates a camera.
/// \param[in] id Id of the camera.
/// \param[in] intrinsics A vector of projection intrinsic parameters.
/// \param[in] image_width Image width in pixels.
/// \param[in] image_height Image height in pixels.
/// \param[in] distortion_parameters The parameters of the distortion object.
/// \param[in] camera_type The camera model.
/// \param[in] distortion_type The distortion model.
/// \returns A new camera based on the provided arguments.
Camera::Ptr createCamera(aslam::CameraId id, const Eigen::VectorXd& intrinsics,
                         uint32_t image_width, uint32_t image_height,
                         const Eigen::VectorXd& distortion_parameters,
                         Camera::Type camera_type,
                         Distortion::Type distortion_type);

/// \brief A factory function to create a derived class camera from a YAML node
Camera::Ptr createCamera(const YAML::Node& yaml_node);

}  // namespace aslam

#endif  // ASLAM_CAMERAS_CAMERA_FACTORY_H_
