#include <memory>

#include <glog/logging.h>

#include <aslam/cameras/camera.h>
#include <aslam/cameras/distortion-equidistant.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/cameras/distortion-null.h>
#include <aslam/cameras/distortion-radtan.h>
#include <aslam/common/yaml-serialization.h>

// TODO(slynen) Enable commented out PropertyTree support
//#include <sm/PropertyTree.hpp>
namespace aslam {

std::ostream& operator<<(std::ostream& out, const ProjectionResult& state) {
  std::string enum_str;
  typedef ProjectionResult::Status Status;
  switch (state.status_) {
    case Status::KEYPOINT_VISIBLE:
      enum_str = "KEYPOINT_VISIBLE";
      break;
    case Status::KEYPOINT_OUTSIDE_IMAGE_BOX:
      enum_str = "KEYPOINT_OUTSIDE_IMAGE_BOX";
      break;
    case Status::POINT_BEHIND_CAMERA:
      enum_str = "POINT_BEHIND_CAMERA";
      break;
    case Status::PROJECTION_INVALID:
      enum_str = "PROJECTION_INVALID";
      break;
    default:
    case Status::UNINITIALIZED:
      enum_str = "UNINITIALIZED";
      break;
  }
  out << "ProjectionResult: " << enum_str << std::endl;
  return out;
}

/// Camera constructor with distortion
Camera::Camera(
    const Eigen::VectorXd& intrinsics, aslam::Distortion::UniquePtr& distortion,
    const uint32_t image_width, const uint32_t image_height, Type camera_type)
    : line_delay_nanoseconds_(0),
      image_width_(image_width),
      image_height_(image_height),
      is_compressed_(false),
      intrinsics_(intrinsics),
      camera_type_(camera_type),
      distortion_(std::move(distortion)) {
  CHECK_NOTNULL(distortion_.get());
}

/// Camera constructor without distortion
Camera::Camera(
    const Eigen::VectorXd& intrinsics, const uint32_t image_width,
    const uint32_t image_height, Type camera_type)
    : line_delay_nanoseconds_(0),
      image_width_(image_width),
      image_height_(image_height),
      is_compressed_(false),
      intrinsics_(intrinsics),
      camera_type_(camera_type),
      distortion_(new NullDistortion()) {}

void Camera::printParameters(std::ostream& out, const std::string& text) const {
  if (text.size() > 0) {
    out << text << std::endl;
  }
  out << "Camera(" << this->id_ << "): " << this->id_ << std::endl;
  out << "  line delay: " << this->line_delay_nanoseconds_ << std::endl;
  out << "  image (cols,rows): " << imageWidth() << ", " << imageHeight()
      << std::endl;
}

bool Camera::isEqualCameraImpl(const Camera& other, const bool verbose) const {
  bool is_equal =
      (this->intrinsics_.isApprox(
          other.intrinsics_, aslam::common::macros::kEpsilon)) &&
      (this->line_delay_nanoseconds_ == other.line_delay_nanoseconds_) &&
      (this->image_width_ == other.image_width_) &&
      (this->image_height_ == other.image_height_) &&
      (this->is_compressed_ == other.is_compressed_);
  if (!is_equal) {
    LOG_IF(WARNING, verbose)
        << "Cameras are not the same,"
        << "this sensor: "
        << "\n intrinsics: " << this->intrinsics_.transpose()
        << "\n line_delay_nanoseconds: " << this->line_delay_nanoseconds_
        << "\n image_width: " << this->image_width_
        << "\n image_height: " << this->image_height_
        << "\n compressed: " << this->is_compressed_;
    LOG_IF(WARNING, verbose)
        << "other sensor: "
        << "\n intrinsics: " << other.intrinsics_.transpose()
        << "\n line_delay_nanoseconds: " << other.line_delay_nanoseconds_
        << "\n image_width: " << other.image_width_
        << "\n image_height: " << other.image_height_
        << "\n compressed: " << other.is_compressed_;
  }

  return is_equal;
}  // namespace aslam

bool Camera::loadFromYamlNodeImpl(const YAML::Node& yaml_node) {
  if (!yaml_node.IsMap()) {
    LOG(ERROR) << "Unable to parse the camera because the node is not a map.";
    return false;
  }

  // Determine the distortion type. Start with no distortion.
  const YAML::Node& distortion_config = yaml_node["distortion"];
  if (distortion_config.IsDefined() && !distortion_config.IsNull()) {
    if (!distortion_config.IsMap()) {
      LOG(ERROR) << "Unable to parse the camera because the distortion node is "
                    "not a map.";
      return false;
    }

    std::string distortion_type;
    Eigen::VectorXd distortion_parameters;
    if (YAML::safeGet(distortion_config, "type", &distortion_type) &&
        YAML::safeGet(
            distortion_config, "parameters", &distortion_parameters)) {
      if (distortion_type == "none") {
        distortion_.reset(new aslam::NullDistortion());
      } else if (distortion_type == "equidistant") {
        if (aslam::EquidistantDistortion::areParametersValid(
                distortion_parameters)) {
          distortion_.reset(
              new aslam::EquidistantDistortion(distortion_parameters));
        } else {
          LOG(ERROR)
              << "Invalid distortion parameters for the Equidistant distortion "
                 "model: "
              << distortion_parameters.transpose() << std::endl
              << "See aslam::EquidistantDistortion::areParametersValid(...) "
                 "for conditions on what "
                 "valid Equidistant distortion parameters look like.";
          return false;
        }
      } else if (distortion_type == "fisheye") {
        if (aslam::FisheyeDistortion::areParametersValid(
                distortion_parameters)) {
          distortion_.reset(
              new aslam::FisheyeDistortion(distortion_parameters));
        } else {
          LOG(ERROR) << "Invalid distortion parameters for the Fisheye "
                        "distortion model: "
                     << distortion_parameters.transpose() << std::endl
                     << "See aslam::FisheyeDistortion::areParametersValid(...) "
                        "for conditions on what "
                        "valid Fisheye distortion parameters look like.";
          return false;
        }
      } else if (distortion_type == "radial-tangential") {
        if (aslam::RadTanDistortion::areParametersValid(
                distortion_parameters)) {
          distortion_.reset(new aslam::RadTanDistortion(distortion_parameters));
        } else {
          LOG(ERROR) << "Invalid distortion parameters for the RadTan "
                        "distortion model: "
                     << distortion_parameters.transpose() << std::endl
                     << "See aslam::RadTanDistortion::areParametersValid(...) "
                        "for conditions on what "
                        "valid RadTan distortion parameters look like.";
          return false;
        }
      } else {
        LOG(ERROR) << "Unknown distortion model: \"" << distortion_type
                   << "\". "
                   << "Valid values are {none, equidistant, fisheye, "
                      "radial-tangential}.";
        return false;
      }

      if (!distortion_->distortionParametersValid(distortion_parameters)) {
        LOG(ERROR) << "Invalid distortion parameters: "
                   << distortion_parameters.transpose();
        return false;
      }
    } else {
      LOG(ERROR)
          << "Unable to get the required parameters from the distortion. "
          << "Required: string type, VectorXd parameters.";
      return false;
    }
  } else {
    distortion_.reset(new aslam::NullDistortion());
  }

  // Get the image width and height
  if (!YAML::safeGet(yaml_node, "image_width", &image_width_) ||
      !YAML::safeGet(yaml_node, "image_height", &image_height_)) {
    LOG(ERROR) << "Unable to get the image size.";
    return false;
  }

  // Get the camera type
  std::string camera_type;
  if (!YAML::safeGet(yaml_node, "type", &camera_type)) {
    LOG(ERROR) << "Unable to get camera type";
    return false;
  }

  if (camera_type == "pinhole") {
    camera_type_ = Type::kPinhole;
  } else if (camera_type == "unified-projection") {
    camera_type_ = Type::kUnifiedProjection;
  } else if (camera_type == "camera-3d-lidar") {
    camera_type_ = Type::kLidar3D;
  } else {
    LOG(ERROR) << "Unknown camera model: \"" << camera_type << "\". "
               << "Valid values are {pinhole, unified-projection}.";
    return false;
  }

  // Get the camera intrinsics
  if (!YAML::safeGet(yaml_node, "intrinsics", &intrinsics_)) {
    LOG(ERROR) << "Unable to get image width.";
    return false;
  }

  if (!intrinsicsValid(intrinsics_)) {
    LOG(ERROR) << "Invalid intrinsics parameters for the " << camera_type
               << " camera model" << intrinsics_.transpose() << std::endl;
    return false;
  }

  // Get the optional linedelay in nanoseconds or set the default
  if (!YAML::safeGet(
          yaml_node, "line-delay-nanoseconds", &line_delay_nanoseconds_)) {
    LOG(WARNING) << "Unable to parse parameter line-delay-nanoseconds."
                 << "Setting to default value = 0.";
    line_delay_nanoseconds_ = 0;
  }

  // Get the optional compressed definition for images or set the default
  if (YAML::hasKey(yaml_node, "compressed")) {
    if (!YAML::safeGet(yaml_node, "compressed", &is_compressed_)) {
      LOG(WARNING) << "Unable to parse parameter compressed."
                   << "Setting to default value = false.";
      is_compressed_ = false;
    }
  }
  return true;
}

void Camera::saveToYamlNodeImpl(YAML::Node* yaml_node) const {
  CHECK_NOTNULL(yaml_node);
  YAML::Node& node = *yaml_node;

  node["compressed"] = hasCompressedImages();
  node["line-delay-nanoseconds"] = getLineDelayNanoSeconds();
  node["image_height"] = imageHeight();
  node["image_width"] = imageWidth();
  switch (getType()) {
    case aslam::Camera::Type::kPinhole:
      node["type"] = "pinhole";
      break;
    case aslam::Camera::Type::kUnifiedProjection:
      node["type"] = "unified-projection";
      break;
    case aslam::Camera::Type::kLidar3D:
      node["type"] = "camera-3d-lidar";
      break;
    default:
      LOG(ERROR)
          << "Unknown camera model: "
          << static_cast<std::underlying_type<aslam::Camera::Type>::type>(
                 getType());
  }
  node["intrinsics"] = getParameters();

  const aslam::Distortion& distortion = getDistortion();
  if (distortion.getType() != aslam::Distortion::Type::kNoDistortion) {
    YAML::Node distortion_node;
    switch (distortion.getType()) {
      case aslam::Distortion::Type::kEquidistant:
        distortion_node["type"] = "equidistant";
        break;
      case aslam::Distortion::Type::kFisheye:
        distortion_node["type"] = "fisheye";
        break;
      case aslam::Distortion::Type::kRadTan:
        distortion_node["type"] = "radial-tangential";
        break;
      default:
        LOG(ERROR)
            << "Unknown distortion model: "
            << static_cast<std::underlying_type<aslam::Distortion::Type>::type>(
                   distortion.getType());
    }
    distortion_node["parameters"] = distortion.getParameters();
    node["distortion"] = distortion_node;
  }
}

const ProjectionResult Camera::project3(
    const Eigen::Ref<const Eigen::Vector3d>& point_3d,
    Eigen::Vector2d* out_keypoint) const {
  CHECK_NOTNULL(out_keypoint);
  return project3Functional(
      point_3d,
      nullptr,  // Use internal intrinsic parameters.
      nullptr,  // Use internal distortion parameters.
      out_keypoint,
      nullptr,   // J_point3d not needed.
      nullptr,   // J_intrinsic not needed.
      nullptr);  // J_distortion not needed.
}

const ProjectionResult Camera::project3(
    const Eigen::Ref<const Eigen::Vector3d>& point_3d,
    Eigen::Vector2d* out_keypoint,
    Eigen::Matrix<double, 2, 3>* out_jacobian) const {
  CHECK_NOTNULL(out_keypoint);
  return project3Functional(
      point_3d,
      nullptr,  // Use internal intrinsic parameters.
      nullptr,  // Use internal distortion parameters.
      out_keypoint, out_jacobian,
      nullptr,   // J_intrinsic not needed.
      nullptr);  // J_distortion not needed.
}

const ProjectionResult Camera::project3Functional(
    const Eigen::Ref<const Eigen::Vector3d>& point_3d,
    const Eigen::VectorXd* intrinsics_external,
    const Eigen::VectorXd* distortion_coefficients_external,
    Eigen::Vector2d* out_keypoint) const {
  CHECK_NOTNULL(out_keypoint);

  return project3Functional(
      point_3d, intrinsics_external, distortion_coefficients_external,
      out_keypoint, nullptr, nullptr, nullptr);
}

const ProjectionResult Camera::project4(
    const Eigen::Ref<const Eigen::Vector4d>& point_4d,
    Eigen::Vector2d* out_keypoint) const {
  CHECK_NOTNULL(out_keypoint);

  Eigen::Vector3d point_3d;
  if (point_4d[3] < 0)
    point_3d = -point_4d.head<3>();
  else
    point_3d = point_4d.head<3>();

  return project3(point_3d, out_keypoint);
}

const ProjectionResult Camera::project4(
    const Eigen::Ref<const Eigen::Vector4d>& point_4d,
    Eigen::Vector2d* out_keypoint,
    Eigen::Matrix<double, 2, 4>* out_jacobian) const {
  CHECK_NOTNULL(out_keypoint);

  Eigen::Vector3d point_3d;
  if (point_4d[3] < 0)
    point_3d = -point_4d.head<3>();
  else
    point_3d = point_4d.head<3>();

  Eigen::Matrix<double, 2, 3> Je;
  ProjectionResult ret = project3(point_3d, out_keypoint, &Je);
  out_jacobian->setZero();
  out_jacobian->topLeftCorner<2, 3>() = Je;

  return ret;
}

bool Camera::backProject4(
    const Eigen::Ref<const Eigen::Vector2d>& keypoint,
    Eigen::Vector4d* out_point4d) const {
  CHECK_NOTNULL(out_point4d);

  Eigen::Vector3d point_3d;
  bool success = backProject3(keypoint, &point_3d);
  (*out_point4d) << point_3d, 0.0;

  return success;
}

bool Camera::isProjectable3(const Eigen::Ref<const Eigen::Vector3d>& p) const {
  Eigen::Vector2d k;
  const ProjectionResult& ret = project3(p, &k);
  return ret.isKeypointVisible();
}

bool Camera::isProjectable4(const Eigen::Ref<const Eigen::Vector4d>& ph) const {
  Eigen::Vector2d k;
  const ProjectionResult& ret = project4(ph, &k);
  return ret.isKeypointVisible();
}

void Camera::project3Vectorized(
    const Eigen::Ref<const Eigen::Matrix3Xd>& points_3d,
    Eigen::Matrix2Xd* out_keypoints,
    std::vector<ProjectionResult>* out_results) const {
  CHECK_NOTNULL(out_keypoints);
  CHECK_NOTNULL(out_results);
  out_keypoints->resize(Eigen::NoChange, points_3d.cols());
  out_results->resize(
      points_3d.cols(), ProjectionResult::Status::UNINITIALIZED);
  Eigen::Vector2d projection;
  for (int i = 0; i < points_3d.cols(); ++i) {
    (*out_results)[i] = project3(points_3d.col(i), &projection);
    out_keypoints->col(i) = projection;
  }
}

void Camera::backProject3Vectorized(
    const Eigen::Ref<const Eigen::Matrix2Xd>& keypoints,
    Eigen::Matrix3Xd* out_points_3d,
    std::vector<unsigned char>* out_success) const {
  CHECK_NOTNULL(out_points_3d);
  CHECK_NOTNULL(out_success);
  out_points_3d->resize(Eigen::NoChange, keypoints.cols());
  out_success->resize(keypoints.cols(), false);
  Eigen::Vector3d bearing;
  for (int i = 0; i < keypoints.cols(); ++i) {
    (*out_success)[i] = backProject3(keypoints.col(i), &bearing);
    out_points_3d->col(i) = bearing;
  }
}

void Camera::setMask(const cv::Mat& mask) {
  CHECK_EQ(image_height_, static_cast<size_t>(mask.rows));
  CHECK_EQ(image_width_, static_cast<size_t>(mask.cols));
  CHECK_EQ(mask.type(), CV_8UC1);
  mask_ = mask;
}

void Camera::clearMask() {
  mask_ = cv::Mat();
}

bool Camera::hasMask() const {
  return !mask_.empty();
}

const cv::Mat& Camera::getMask() const {
  return mask_;
}

ProjectionResult::Status ProjectionResult::KEYPOINT_VISIBLE =
    ProjectionResult::Status::KEYPOINT_VISIBLE;
ProjectionResult::Status ProjectionResult::KEYPOINT_OUTSIDE_IMAGE_BOX =
    ProjectionResult::Status::KEYPOINT_OUTSIDE_IMAGE_BOX;
ProjectionResult::Status ProjectionResult::POINT_BEHIND_CAMERA =
    ProjectionResult::Status::POINT_BEHIND_CAMERA;
ProjectionResult::Status ProjectionResult::PROJECTION_INVALID =
    ProjectionResult::Status::PROJECTION_INVALID;
ProjectionResult::Status ProjectionResult::UNINITIALIZED =
    ProjectionResult::Status::UNINITIALIZED;
}  // namespace aslam
