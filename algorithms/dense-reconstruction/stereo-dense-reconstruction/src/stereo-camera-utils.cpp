#include "dense-reconstruction/stereo-camera-utils.h"

#include <memory>
#include <string>
#include <vector>

#include <glog/logging.h>

namespace dense_reconstruction {
namespace stereo {

CameraParametersBase::CameraParametersBase(
    const cv::Size& resolution_in, const Eigen::Matrix<double, 4, 4>& T_C_G_in,
    const Eigen::Matrix<double, 3, 3>& K_in)
    : resolution_(resolution_in),
      T_C_G_(T_C_G_in),
      P_((Eigen::Matrix<double, 3, 4>() << K_in, Eigen::Vector3d::Constant(0))
             .finished() *
         T_C_G_in),
      K_(K_in) {}

const cv::Size& CameraParametersBase::resolution() const {
  return resolution_;
}

const Eigen::Matrix<double, 4, 4>& CameraParametersBase::T_C_G() const {
  return T_C_G_;
}
const Eigen::Ref<const Eigen::Matrix<double, 3, 3>> CameraParametersBase::R()
    const {
  return T_C_G_.topLeftCorner<3, 3>();
}
const Eigen::Ref<const Eigen::Matrix<double, 3, 1>> CameraParametersBase::p()
    const {
  return T_C_G_.topRightCorner<3, 1>();
}

const Eigen::Matrix<double, 3, 4>& CameraParametersBase::P() const {
  return P_;
}
const Eigen::Matrix<double, 3, 3>& CameraParametersBase::K() const {
  return K_;
}

bool CameraParametersBase::operator==(const CameraParametersBase& B) const {
  return (resolution() == B.resolution()) && (T_C_G() == B.T_C_G()) &&
         (P() == B.P()) && (K() == B.K());
}

bool CameraParametersBase::operator!=(const CameraParametersBase& B) const {
  return !(*this == B);
}

InputCameraParameters::InputCameraParameters(
    const cv::Size& resolution_in, const Eigen::Matrix<double, 4, 4>& T_C_G_in,
    const Eigen::Matrix<double, 3, 3>& K_in, const std::vector<double>& D_in,
    const DistortionModel& distortion_model)
    : CameraParametersBase(resolution_in, T_C_G_in, K_in),
      D_(D_in),
      distortion_model_(distortion_model) {
  // Ensure D always has at least 5 elements.
  while (D_.size() < 5) {
    D_.push_back(0);
  }
}

const std::vector<double>& InputCameraParameters::D() const {
  return D_;
}

const DistortionModel& InputCameraParameters::distortionModel() const {
  return distortion_model_;
}

DistortionModel InputCameraParameters::stringToDistortion(
    const std::string& distortion_model) {
  std::string lower_case_distortion_model = distortion_model;

  std::transform(
      lower_case_distortion_model.begin(), lower_case_distortion_model.end(),
      lower_case_distortion_model.begin(), ::tolower);
  if ((lower_case_distortion_model == std::string("plumb bob")) ||
      (lower_case_distortion_model == std::string("plumb_bob")) ||
      (lower_case_distortion_model == std::string("radtan"))) {
    return DistortionModel::RADTAN;
  } else if (lower_case_distortion_model == std::string("equidistant")) {
    return DistortionModel::EQUIDISTANT;
  } else if (lower_case_distortion_model == std::string("fov")) {
    return DistortionModel::FOV;
  } else {
    LOG(FATAL) << "Unrecognized distortion model. Valid options are 'radtan', "
               << "'Plumb Bob', 'plumb_bob' 'equidistant' and 'fov'";
  }
}

bool InputCameraParameters::operator==(const InputCameraParameters& B) const {
  return (*dynamic_cast<const CameraParametersBase*>(this) == B) &&
         (D() == B.D()) && (distortionModel() == B.distortionModel());
}

bool InputCameraParameters::operator!=(const InputCameraParameters& B) const {
  return !(*this == B);
}

// Holds the camera parameters of the input camera and virtual output camera.
CameraParametersPair::CameraParametersPair(
    const DistortionProcessing distortion_processing)
    : distortion_processing_(distortion_processing) {}

bool CameraParametersPair::setInputCameraParameters(
    const cv::Size& resolution, const Eigen::Matrix<double, 4, 4>& T,
    const Eigen::Matrix<double, 3, 3>& K, const std::vector<double>& D,
    const DistortionModel& distortion_model) {
  try {
    input_ptr_ = std::make_shared<InputCameraParameters>(
        resolution, T, K, D, distortion_model);
    return true;
  } catch (std::runtime_error e) {
    LOG(ERROR) << e.what();
    return false;
  }
}

bool CameraParametersPair::setOutputCameraParameters(
    const cv::Size& resolution, const Eigen::Matrix<double, 4, 4>& T,
    const Eigen::Matrix<double, 3, 3>& K) {
  try {
    output_ptr_ = std::make_shared<OutputCameraParameters>(resolution, T, K);
    return true;
  } catch (std::runtime_error e) {
    LOG(ERROR) << e.what();
    return false;
  }
}

bool CameraParametersPair::setOutputFromInput() {
  if (!valid(CameraIO::INPUT)) {
    LOG(ERROR) << "Cannot set output to same values as input, as input is not "
               << "currently set";
    return false;
  } else {
    setOutputCameraParameters(
        input_ptr_->resolution(), input_ptr_->T_C_G(), input_ptr_->K());
    return true;
  }
}

bool CameraParametersPair::setOptimalOutputCameraParameters(
    const double scale) {
  if (!valid(CameraIO::INPUT)) {
    LOG(ERROR) << "Optimal output camera parameters cannot be set until the "
               << "input camera parameters have been given";
    return false;
  }
  cv::Size resolution_estimate(
      std::ceil(input_ptr_->resolution().width * scale),
      std::ceil(input_ptr_->resolution().height * scale));
  double focal_length =
      scale * (input_ptr_->K()(0, 0) + input_ptr_->K()(1, 1)) / 2;
  Eigen::Matrix<double, 3, 4> P = Eigen::Matrix<double, 3, 4>::Zero();
  P(0, 0) = focal_length;
  P(1, 1) = focal_length;
  P(2, 2) = 1;
  P(0, 2) = static_cast<double>(resolution_estimate.width) / 2.0;
  P(1, 2) = static_cast<double>(resolution_estimate.height) / 2.0;
  P.topRightCorner<3, 1>() = focal_length * input_ptr_->p();

  std::vector<double> D;
  if (distortion_processing_ == DistortionProcessing::UNDISTORT) {
    D = input_ptr_->D();
  } else {
    D = std::vector<double>(0, 5);
  }

  // Find the resolution of the output image.
  // Thanks to weird corner cases this is way more complex then it should be.
  // The general case is even more of a nightmare, so we constrain the problem
  // such that the center of focus must be in the center of the final image.
  // As we are missing the forward projection model we iteratively estimate
  // image size assuming a linear relationship between warping and size at each
  // step.
  CHECK_GT(resolution_estimate.width, 0);
  CHECK_GT(resolution_estimate.height, 0);
  for (size_t i = 0; i < kFocalLengthEstimationAttempts; ++i) {
    // get list of edge points to check
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>
        pixel_locations;
    for (size_t v = 0; v < static_cast<size_t>(resolution_estimate.height);
         ++v) {
      pixel_locations.emplace_back(0, v);
      pixel_locations.emplace_back(resolution_estimate.width - 1, v);
    }
    for (size_t u = 1; u < static_cast<size_t>(resolution_estimate.width - 1);
         ++u) {
      pixel_locations.emplace_back(u, 0);
      pixel_locations.emplace_back(u, resolution_estimate.height - 1);
    }

    // Find extreme points.
    double max_x = 0;
    double max_y = 0;
    for (Eigen::Vector2d pixel_location : pixel_locations) {
      Eigen::Vector2d distorted_pixel_location;
      Undistorter::distortPixel(
          input_ptr_->K(), input_ptr_->R(), P, input_ptr_->distortionModel(), D,
          pixel_location, &distorted_pixel_location);

      max_x = std::max(
          max_x, std::abs(
                     static_cast<double>(input_ptr_->resolution().width) / 2.0 -
                     distorted_pixel_location.x()));
      max_y = std::max(
          max_y,
          std::abs(
              static_cast<double>(input_ptr_->resolution().height) / 2.0 -
              distorted_pixel_location.y()));
    }

    // Change resolution estimate so that extreme points lie on edges (under
    // the aforementioned linear assumption).
    cv::Size resolution_update;
    resolution_update.width = std::floor(
        static_cast<double>(resolution_estimate.width) *
        std::abs(static_cast<double>(input_ptr_->resolution().width) - max_x) /
        (static_cast<double>(input_ptr_->resolution().width) / 2.0));
    resolution_update.height = std::floor(
        static_cast<double>(resolution_estimate.height) *
        std::abs(static_cast<double>(input_ptr_->resolution().height) - max_y) /
        (static_cast<double>(input_ptr_->resolution().height) / 2.0));

    if (resolution_update == resolution_estimate) {
      break;
    } else {
      resolution_estimate = resolution_update;
      P(0, 2) = static_cast<double>(resolution_estimate.width) / 2.0;
      P(1, 2) = static_cast<double>(resolution_estimate.height) / 2.0;
    }
  }

  // Create final camera parameters.
  Eigen::Matrix3d K = P.topLeftCorner<3, 3>();
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topRightCorner<3, 1>() = input_ptr_->p();

  return setOutputCameraParameters(resolution_estimate, T, K);
}

const DistortionProcessing& CameraParametersPair::distortionProcessing() const {
  return distortion_processing_;
}

const std::shared_ptr<InputCameraParameters>&
CameraParametersPair::getInputPtr() const {
  return input_ptr_;
}
const std::shared_ptr<OutputCameraParameters>&
CameraParametersPair::getOutputPtr() const {
  return output_ptr_;
}

bool CameraParametersPair::valid() const {
  return (input_ptr_ != nullptr) && (output_ptr_ != nullptr);
}

bool CameraParametersPair::valid(const CameraIO& io) const {
  if (io == CameraIO::INPUT) {
    return input_ptr_ != nullptr;
  } else {
    return output_ptr_ != nullptr;
  }
}

bool CameraParametersPair::operator==(const CameraParametersPair& B) const {
  return *getInputPtr() == *B.getInputPtr() &&
         (*getOutputPtr() == *B.getOutputPtr());
}

bool CameraParametersPair::operator!=(const CameraParametersPair& B) const {
  return !(*this == B);
}

StereoCameraParameters::StereoCameraParameters(const double scale)
    : scale_(scale) {}

bool StereoCameraParameters::setInputCameraParameters(
    const cv::Size& resolution, const Eigen::Matrix<double, 4, 4>& T,
    const Eigen::Matrix<double, 3, 3>& K, const std::vector<double>& D,
    const DistortionModel& distortion_model, const CameraSide& side) {
  try {
    if (side == CameraSide::FIRST) {
      first_.setInputCameraParameters(resolution, T, K, D, distortion_model);
    } else {
      second_.setInputCameraParameters(resolution, T, K, D, distortion_model);
    }
    if (valid(CameraSide::FIRST, CameraIO::INPUT) &&
        valid(CameraSide::SECOND, CameraIO::INPUT)) {
      generateRectificationParameters();
    }
    return true;
  } catch (std::runtime_error e) {
    LOG(ERROR) << e.what();
    return false;
  }
}

bool StereoCameraParameters::valid() const {
  return first_.valid() && second_.valid();
}
bool StereoCameraParameters::valid(
    const CameraSide& side, const CameraIO& io) const {
  if (side == CameraSide::FIRST) {
    return first_.valid(io);
  } else {
    return second_.valid(io);
  }
}

bool StereoCameraParameters::generateRectificationParameters() {
  if (first_.getInputPtr()->p().isApprox(second_.getInputPtr()->p())) {
    LOG(ERROR) << "Stereo rectification cannot be performed on cameras with a "
               << "baseline of zero";
    return false;
  }

  // Twist inputs to align on x axis.
  Eigen::Vector3d x = first_.getInputPtr()->p() - second_.getInputPtr()->p();
  Eigen::Vector3d y = first_.getInputPtr()->R().col(2).cross(x);
  Eigen::Vector3d z = x.cross(y);

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3, 3>() << x.normalized(), y.normalized(), z.normalized();

  // Took wrong camera as left (redo other way round).
  if (T(0, 0) < 0) {
    x = second_.getInputPtr()->p() - first_.getInputPtr()->p();
    y = second_.getInputPtr()->R().col(2).cross(x);
    z = x.cross(y);
    T.topLeftCorner<3, 3>() << x.normalized(), y.normalized(), z.normalized();

    LOG(ERROR)
        << "Left and right stereo camera are switched! Trying to correct!";
  }

  first_.setInputCameraParameters(
      first_.getInputPtr()->resolution(),
      T.inverse() * first_.getInputPtr()->T_C_G(), first_.getInputPtr()->K(),
      first_.getInputPtr()->D(), first_.getInputPtr()->distortionModel());
  second_.setInputCameraParameters(
      second_.getInputPtr()->resolution(),
      T.inverse() * second_.getInputPtr()->T_C_G(), second_.getInputPtr()->K(),
      second_.getInputPtr()->D(), second_.getInputPtr()->distortionModel());

  // Set individual outputs.
  if (!first_.setOptimalOutputCameraParameters(scale_) ||
      !second_.setOptimalOutputCameraParameters(scale_)) {
    LOG(ERROR) << "Automatic generation of stereo output parameters failed";
    return false;
  }

  // Grab most conservative values.
  cv::Size resolution(
      std::min(
          first_.getOutputPtr()->resolution().width,
          second_.getOutputPtr()->resolution().width),
      std::min(
          first_.getOutputPtr()->resolution().height,
          second_.getOutputPtr()->resolution().height));

  Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
  K(0, 0) = std::max(
      first_.getOutputPtr()->K()(0, 0), second_.getOutputPtr()->K()(0, 0));
  K(1, 1) = K(0, 0);
  K(0, 2) = static_cast<double>(resolution.width) / 2.0;
  K(1, 2) = static_cast<double>(resolution.height) / 2.0;
  K(2, 2) = 1;

  // Set the new consistent outputs.
  if (!first_.setOutputCameraParameters(
          resolution, first_.getOutputPtr()->T_C_G(), K) ||
      !second_.setOutputCameraParameters(
          resolution, second_.getOutputPtr()->T_C_G(), K)) {
    LOG(ERROR) << "Automatic generation of stereo output parameters failed";
    return false;
  }

  return true;
}

const CameraParametersPair& StereoCameraParameters::getFirst() const {
  return first_;
}

const CameraParametersPair& StereoCameraParameters::getSecond() const {
  return second_;
}

Undistorter::Undistorter(
    const CameraParametersPair& input_camera_parameters_pair)
    : used_camera_parameters_pair_(input_camera_parameters_pair) {
  if (!used_camera_parameters_pair_.valid()) {
    throw std::runtime_error(
        "Attempted to create undistorter from invalid camera parameters");
  }
  const cv::Size& resolution_out =
      used_camera_parameters_pair_.getOutputPtr()->resolution();
  const cv::Size& resolution_in =
      used_camera_parameters_pair_.getInputPtr()->resolution();
  // Initialize maps.
  cv::Mat map_x_float(resolution_out, CV_32FC1);
  cv::Mat map_y_float(resolution_out, CV_32FC1);

  std::vector<double> D;
  if (used_camera_parameters_pair_.distortionProcessing() ==
      DistortionProcessing::UNDISTORT) {
    D = used_camera_parameters_pair_.getInputPtr()->D();
  } else {
    D = std::vector<double>(0, 5);
  }

  empty_pixels_ = false;

  // Compute the remap maps.
  CHECK_GT(resolution_out.height, 0);
  CHECK_GT(resolution_out.width, 0);
  for (size_t v = 0; v < static_cast<size_t>(resolution_out.height); ++v) {
    for (size_t u = 0; u < static_cast<size_t>(resolution_out.width); ++u) {
      Eigen::Vector2d pixel_location(u, v);
      Eigen::Vector2d distorted_pixel_location;
      distortPixel(
          used_camera_parameters_pair_.getInputPtr()->K(),
          used_camera_parameters_pair_.getInputPtr()->R(),
          used_camera_parameters_pair_.getOutputPtr()->P(),
          used_camera_parameters_pair_.getInputPtr()->distortionModel(), D,
          pixel_location, &distorted_pixel_location);

      // Insert in map.
      map_x_float.at<float>(v, u) =
          static_cast<float>(distorted_pixel_location.x());
      map_y_float.at<float>(v, u) =
          static_cast<float>(distorted_pixel_location.y());

      if ((distorted_pixel_location.x() < 0) ||
          (distorted_pixel_location.y() < 0) ||
          (distorted_pixel_location.x() >= resolution_in.width) ||
          (distorted_pixel_location.y() >= resolution_in.height)) {
        empty_pixels_ = true;
      }
    }
  }

  // Convert to fixed point maps for increased speed.
  cv::convertMaps(map_x_float, map_y_float, map_x_, map_y_, CV_16SC2);
}

void Undistorter::undistortImage(
    const cv::Mat& image, cv::Mat* undistorted_image) {
  if (empty_pixels_) {
    cv::remap(
        image, *undistorted_image, map_x_, map_y_, cv::INTER_LINEAR,
        cv::BORDER_CONSTANT);
  } else {
    // Replicate is more efficient for gpus to calculate.
    cv::remap(
        image, *undistorted_image, map_x_, map_y_, cv::INTER_LINEAR,
        cv::BORDER_REPLICATE);
  }
}

const CameraParametersPair& Undistorter::getCameraParametersPair() {
  return used_camera_parameters_pair_;
}

void Undistorter::distortPixel(
    const Eigen::Matrix<double, 3, 3>& K_in,
    const Eigen::Matrix<double, 3, 3>& R_in,
    const Eigen::Matrix<double, 3, 4>& P_out,
    const DistortionModel& distortion_model, const std::vector<double>& D,
    const Eigen::Vector2d& pixel_location,
    Eigen::Vector2d* distorted_pixel_location) {
  Eigen::Vector3d pixel_location_3(pixel_location.x(), pixel_location.y(), 1.0);

  // Transform image coordinates to be size and focus independent.
  Eigen::Vector3d norm_pixel_location =
      R_in * P_out.topLeftCorner<3, 3>().inverse() * pixel_location_3;

  const double& x = norm_pixel_location.x();
  const double& y = norm_pixel_location.y();

  Eigen::Vector3d norm_distorted_pixel_location(
      0.0, 0.0, norm_pixel_location.z());
  double& xd = norm_distorted_pixel_location.x();
  double& yd = norm_distorted_pixel_location.y();

  switch (distortion_model) {
    case DistortionModel::RADTAN: {
      // Split out parameters for easier reading.
      const double& k1 = D[0];
      const double& k2 = D[1];
      const double& k3 = D[4];
      const double& p1 = D[2];
      const double& p2 = D[3];

      // Undistort.
      const double r2 = x * x + y * y;
      const double r4 = r2 * r2;
      const double r6 = r4 * r2;
      const double kr = (1.0 + k1 * r2 + k2 * r4 + k3 * r6);
      xd = x * kr + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
      yd = y * kr + 2.0 * p2 * x * y + p1 * (r2 + 2.0 * y * y);
    } break;
    case DistortionModel::EQUIDISTANT: {
      // Split out distortion parameters for easier reading.
      const double& k1 = D[0];
      const double& k2 = D[1];
      const double& k3 = D[2];
      const double& k4 = D[3];

      // Undistort.
      const double r = std::sqrt(x * x + y * y);
      if (r < 1e-10) {
        *distorted_pixel_location = pixel_location;
        return;
      }
      const double theta = atan(r);
      const double theta_2 = theta * theta;
      const double theta_4 = theta_2 * theta_2;
      const double theta_6 = theta_2 * theta_4;
      const double theta_8 = theta_4 * theta_4;
      const double thetad = theta * (1.0 + k1 * theta_2 + k2 * theta_4 +
                                     k3 * theta_6 + k4 * theta_8);

      const double scaling = (r > 1e-8) ? thetad / r : 1.0;
      xd = x * scaling;
      yd = y * scaling;
    } break;
    case DistortionModel::FOV: {
      // Split out parameters for easier reading.
      const double& fov = D[0];

      const double r = std::sqrt(x * x + y * y);
      if (r < 1e-10) {
        *distorted_pixel_location = pixel_location;
        return;
      }
      double rd = (1.0 / fov) * atan(2.0 * tan(fov / 2.0) * r);
      xd = x * (rd / r);
      yd = y * (rd / r);
    } break;
    default:
      throw std::runtime_error(
          "Distortion model not implemented - model: " +
          static_cast<int>(distortion_model));
  }

  Eigen::Vector3d distorted_pixel_location_3 =
      K_in * norm_distorted_pixel_location;

  distorted_pixel_location->x() =
      distorted_pixel_location_3.x() / distorted_pixel_location_3.z();
  distorted_pixel_location->y() =
      distorted_pixel_location_3.y() / distorted_pixel_location_3.z();
}

}  // namespace stereo
}  // namespace dense_reconstruction
