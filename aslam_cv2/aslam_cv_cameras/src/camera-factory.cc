#include <aslam/cameras/camera-factory.h>

#include <glog/logging.h>

#include <aslam/cameras/camera.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera-3d-lidar.h>
#include <aslam/cameras/camera-unified-projection.h>
#include <aslam/cameras/distortion-equidistant.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/cameras/distortion-null.h>
#include <aslam/cameras/distortion-radtan.h>
#include <aslam/common/yaml-serialization-eigen.h>

namespace aslam {

Camera::Ptr createCamera(aslam::CameraId id, const Eigen::VectorXd& intrinsics,
                         uint32_t image_width, uint32_t image_height,
                         const Eigen::VectorXd& distortion_parameters,
                         Camera::Type camera_type,
                         Distortion::Type distortion_type) {
  CHECK(id.isValid()) << "Invalid camera id: " << id.hexString();

  Distortion::UniquePtr distortion;
  switch(distortion_type) {
    case Distortion::Type::kNoDistortion:
      distortion.reset(new NullDistortion());
      break;
    case Distortion::Type::kEquidistant:
      distortion.reset(new EquidistantDistortion(distortion_parameters));
      break;
    case Distortion::Type::kFisheye:
      distortion.reset(new FisheyeDistortion(distortion_parameters));
      break;
    case Distortion::Type::kRadTan:
      distortion.reset(new RadTanDistortion(distortion_parameters));
      break;
    default:
      LOG(FATAL) << "Unknown distortion model: "
          << static_cast<std::underlying_type<Distortion::Type>::type>(
              distortion_type);
  }
  CHECK(distortion->distortionParametersValid(distortion_parameters))
      << "Invalid distortion parameters: "
      << distortion_parameters.transpose();

  Camera::Ptr camera;
  switch(camera_type) {
    case Camera::Type::kPinhole:
      camera.reset(new PinholeCamera(intrinsics, image_width, image_height,
                                     distortion));
      break;
    case Camera::Type::kUnifiedProjection:
      camera.reset(new UnifiedProjectionCamera(intrinsics, image_width,
                                               image_height, distortion));
      break;
    default:
      LOG(FATAL) << "Unknown camera model: "
        << static_cast<std::underlying_type<Camera::Type>::type>(camera_type);
  }

  camera->setId(id);
  return camera;
}

Camera::Ptr createCamera(const YAML::Node& yaml_node) {
  if (!yaml_node.IsDefined() || yaml_node.IsNull()) {
    LOG(ERROR) << "Camera YAML node is invalid.";
    return nullptr;
  }

  if (!yaml_node.IsMap()) {
    LOG(ERROR)
        << "Intrinsics node for camera is not a map.";
    return nullptr;
  }

  std::string camera_type;
  if (!YAML::safeGet(yaml_node, "type", &camera_type)) {
    LOG(ERROR) << "Unable to get camera type.";
    return nullptr;
  }

  // Based on the type deserialize the camera
  Camera::Ptr camera;
  if (camera_type == "pinhole") {
    camera = std::dynamic_pointer_cast<Camera>(aligned_shared<PinholeCamera>());
  } else if (camera_type == "unified-projection") {
    camera = std::dynamic_pointer_cast<Camera>(
        aligned_shared<UnifiedProjectionCamera>());
  } else if (camera_type == "camera-3d-lidar") {
    camera = std::dynamic_pointer_cast<Camera>(aligned_shared<Camera3DLidar>());
  } else {
    LOG(ERROR) << "Unknown camera model: \"" << camera_type << "\". "
               << "Valid values are {pinhole, unified-projection, camera-3d-lidar}.";
    return nullptr;
  }

  if (!camera->deserialize(yaml_node)) {
    LOG(ERROR) << "Camera deserialization failed.";
    return nullptr;
  }

  return camera;
}

}  // namespace aslam
