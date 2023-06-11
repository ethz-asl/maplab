#include <memory>

#include <aslam/cameras/camera-unified-projection.h>

#include <aslam/cameras/camera-factory.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/common/types.h>

#include "aslam/cameras/random-camera-generator.h"

namespace aslam {
std::ostream& operator<<(std::ostream& out,
                                  const UnifiedProjectionCamera& camera) {
  camera.printParameters(out, std::string(""));
  return out;
}

UnifiedProjectionCamera::UnifiedProjectionCamera()
    : Base((Eigen::Matrix<double, 5, 1>() << 0.0, 0.0, 0.0, 0.0, 0.0).finished(), 0, 0,
           Camera::Type::kUnifiedProjection) {
}

UnifiedProjectionCamera::UnifiedProjectionCamera(const Eigen::VectorXd& intrinsics,
                                                 uint32_t image_width, uint32_t image_height,
                                                 aslam::Distortion::UniquePtr& distortion)
    : Base(intrinsics, distortion, image_width, image_height,
           Camera::Type::kUnifiedProjection) {
  CHECK(intrinsicsValid(intrinsics));
}

UnifiedProjectionCamera::UnifiedProjectionCamera(const Eigen::VectorXd& intrinsics,
                                                 uint32_t image_width, uint32_t image_height)
    : Base(intrinsics, image_width, image_height, Camera::Type::kUnifiedProjection) {
  CHECK(intrinsicsValid(intrinsics));
}

UnifiedProjectionCamera::UnifiedProjectionCamera(double xi, double focallength_cols,
                                                 double focallength_rows, double imagecenter_cols,
                                                 double imagecenter_rows, uint32_t image_width,
                                                 uint32_t image_height,
                                                 aslam::Distortion::UniquePtr& distortion)
    : UnifiedProjectionCamera(
        (Eigen::Matrix<double, 5, 1>() << xi, focallength_cols, focallength_rows, imagecenter_cols,
            imagecenter_rows).finished(), image_width, image_height, distortion) {}

UnifiedProjectionCamera::UnifiedProjectionCamera(double xi, double focallength_cols,
                                                 double focallength_rows, double imagecenter_cols,
                                                 double imagecenter_rows, uint32_t image_width,
                                                 uint32_t image_height)
    : UnifiedProjectionCamera(
        (Eigen::Matrix<double, 5, 1>() << xi, focallength_cols, focallength_rows, imagecenter_cols,
            imagecenter_rows).finished(), image_width, image_height) {}

bool UnifiedProjectionCamera::backProject3(const Eigen::Ref<const Eigen::Vector2d>& keypoint,
                                           Eigen::Vector3d* out_point_3d) const {
  CHECK_NOTNULL(out_point_3d);

  Eigen::Vector2d kp = keypoint;
  kp[0] = (kp[0] - cu()) / fu();
  kp[1] = (kp[1] - cv()) / fv();

  distortion_->undistort(&kp);

  const double rho2_d = kp[0] * kp[0] + kp[1] * kp[1];
  const double tmpD = std::max(1 + (1 - xi()*xi()) * rho2_d, 0.0);

  (*out_point_3d)[0] = kp[0];
  (*out_point_3d)[1] = kp[1];
  (*out_point_3d)[2] = 1 - xi() * (rho2_d + 1) / (xi() + sqrt(tmpD));

  return isUndistortedKeypointValid(rho2_d, xi());
}

const ProjectionResult UnifiedProjectionCamera::project3Functional(
    const Eigen::Ref<const Eigen::Vector3d>& point_3d,
    const Eigen::VectorXd* intrinsics_external,
    const Eigen::VectorXd* distortion_coefficients_external,
    Eigen::Vector2d* out_keypoint,
    Eigen::Matrix<double, 2, 3>* out_jacobian_point,
    Eigen::Matrix<double, 2, Eigen::Dynamic>* out_jacobian_intrinsics,
    Eigen::Matrix<double, 2, Eigen::Dynamic>* out_jacobian_distortion) const {
  CHECK_NOTNULL(out_keypoint);

  // Determine the parameter source. (if nullptr, use internal)
  const Eigen::VectorXd* intrinsics;
  if (!intrinsics_external)
    intrinsics = &getParameters();
  else
    intrinsics = intrinsics_external;
  CHECK_EQ(intrinsics->size(), kNumOfParams) << "intrinsics: invalid size!";

  const Eigen::VectorXd* distortion_coefficients;
  if(!distortion_coefficients_external) {
    distortion_coefficients = &getDistortion().getParameters();
  } else {
    distortion_coefficients = distortion_coefficients_external;
  }

  const double& xi = (*intrinsics)[0];
  const double& fu = (*intrinsics)[1];
  const double& fv = (*intrinsics)[2];
  const double& cu = (*intrinsics)[3];
  const double& cv = (*intrinsics)[4];

  // Project the point.
  const double& x = point_3d[0];
  const double& y = point_3d[1];
  const double& z = point_3d[2];

  const double d = point_3d.norm();
  const double rz = 1.0 / (z + xi * d);

  // Check if point will lead to a valid projection
  const bool valid_proj = z > -(fov_parameter(xi) * d);
  if (!valid_proj) {
    if (out_jacobian_intrinsics) {
      out_jacobian_intrinsics->setZero(2, kNumOfParams);
    }
    if (out_jacobian_distortion) {
      out_jacobian_distortion->setZero(2, distortion_->getParameterSize());
    }
    out_keypoint->setZero();
    return ProjectionResult(ProjectionResult::Status::PROJECTION_INVALID);
  }

  (*out_keypoint)[0] = x * rz;
  (*out_keypoint)[1] = y * rz;

  // Distort the point and get the Jacobian wrt. keypoint.
  Eigen::Matrix2d J_distortion = Eigen::Matrix2d::Identity();

  // Calculate the Jacobian w.r.t to the distortion parameters, if requested.
  if(out_jacobian_distortion) {
    distortion_->distortParameterJacobian(distortion_coefficients,
                                          *out_keypoint,
                                          out_jacobian_distortion);
    (*out_jacobian_distortion).row(0) *= fu;
    (*out_jacobian_distortion).row(1) *= fv;
  }

  if (out_jacobian_point || out_jacobian_intrinsics) {
    // Distortion active and we want the Jacobian.
    distortion_->distortUsingExternalCoefficients(distortion_coefficients,
                                                  out_keypoint,
                                                  &J_distortion);
  } else {
    // Distortion active but Jacobian NOT wanted.
    distortion_->distortUsingExternalCoefficients(distortion_coefficients,
                                                  out_keypoint,
                                                  nullptr);
  }

  // Calculate the Jacobian w.r.t to the 3d point, if requested.
  if(out_jacobian_point) {
    // Jacobian including distortion
    Eigen::Matrix<double, 2, 3>& J = *out_jacobian_point;
    double rz2 = rz * rz / d;
    J(0, 0) = rz2 * (d * z + xi * (y * y + z * z));
    J(1, 0) = -rz2 * xi * x * y;
    J(0, 1) = J(1, 0);
    J(1, 1) = rz2 * (d * z + xi * (x * x + z * z));
    rz2 = rz2 * (-xi * z - d);
    J(0, 2) = x * rz2;
    J(1, 2) = y * rz2;
    rz2 = fu * (J(0, 0) * J_distortion(0, 0) + J(1, 0) * J_distortion(0, 1));
    J(1, 0) = fv * (J(0, 0) * J_distortion(1, 0) + J(1, 0) * J_distortion(1, 1));
    J(0, 0) = rz2;
    rz2 = fu * (J(0, 1) * J_distortion(0, 0) + J(1, 1) * J_distortion(0, 1));
    J(1, 1) = fv * (J(0, 1) * J_distortion(1, 0) + J(1, 1) * J_distortion(1, 1));
    J(0, 1) = rz2;
    rz2 = fu * (J(0, 2) * J_distortion(0, 0) + J(1, 2) * J_distortion(0, 1));
    J(1, 2) = fv * (J(0, 2) * J_distortion(1, 0) + J(1, 2) * J_distortion(1, 1));
    J(0, 2) = rz2;
  }

  // Calculate the Jacobian w.r.t to the intrinsic parameters, if requested.
  if(out_jacobian_intrinsics) {
    out_jacobian_intrinsics->setZero(2, kNumOfParams);

    Eigen::Vector2d Jxi;
    Jxi[0] = -x * rz * d * rz;
    Jxi[1] = -y * rz * d * rz;
    J_distortion.row(0) *= fu;
    J_distortion.row(1) *= fv;
    (*out_jacobian_intrinsics).col(0) = J_distortion * Jxi;

    (*out_jacobian_intrinsics)(0, 1) = (*out_keypoint)[0];
    (*out_jacobian_intrinsics)(0, 3) = 1;
    (*out_jacobian_intrinsics)(1, 2) = (*out_keypoint)[1];
    (*out_jacobian_intrinsics)(1, 4) = 1;
  }

  // Normalized image plane to camera plane.
  (*out_keypoint)[0] = fu * (*out_keypoint)[0] + cu;
  (*out_keypoint)[1] = fv * (*out_keypoint)[1] + cv;

  return evaluateProjectionResult(*out_keypoint, point_3d);
}

inline const ProjectionResult UnifiedProjectionCamera::evaluateProjectionResult(
    const Eigen::Ref<const Eigen::Vector2d>& keypoint,
    const Eigen::Vector3d& point_3d) const {

  const bool visibility = isKeypointVisible(keypoint);

  const double d2 = point_3d.squaredNorm();
  const double minDepth2 = kMinimumDepth*kMinimumDepth;

  if (visibility && (d2 > minDepth2))
    return ProjectionResult(ProjectionResult::Status::KEYPOINT_VISIBLE);
  else if (!visibility && (d2 > minDepth2))
    return ProjectionResult(ProjectionResult::Status::KEYPOINT_OUTSIDE_IMAGE_BOX);
  else if (d2 <= minDepth2)
    return ProjectionResult(ProjectionResult::Status::PROJECTION_INVALID);

  return ProjectionResult(ProjectionResult::Status::PROJECTION_INVALID);
}

inline bool UnifiedProjectionCamera::isUndistortedKeypointValid(const double& rho2_d,
                                                                const double& xi) const {
  return xi <= 1.0 || rho2_d <= (1.0 / (xi * xi - 1));
}

bool UnifiedProjectionCamera::isLiftable(const Eigen::Ref<const Eigen::Vector2d>& keypoint) const {
  Eigen::Vector2d y;
  y[0] = 1.0 / fu() * (keypoint[0] - cu());
  y[1] = 1.0 / fv() * (keypoint[1] - cv());

  distortion_->undistort(&y);

  // Now check if it is on the sensor.
  double rho2_d = y[0] * y[0] + y[1] * y[1];
  return isUndistortedKeypointValid(rho2_d, xi());
}

Eigen::Vector2d UnifiedProjectionCamera::createRandomKeypoint() const {
  // This is tricky...The camera model defines a circle on the normalized image
  // plane and the projection equations don't work outside of it.
  // With some manipulation, we can see that, on the normalized image plane,
  // the edge of this circle is at u^2 + v^2 = 1/(xi^2 - 1)
  // So: this function creates keypoints inside this boundary.


  // Create a point on the normalized image plane inside the boundary.
  // This is not efficient, but it should be correct.
  const double ru = imageWidth(),
               rv = imageHeight();

  Eigen::Vector2d u(ru + 1, rv + 1);
  double one_over_xixi_m_1 = 1.0 / (xi() * xi() - 1);

  int max_tries = 20;
  while ( !(isLiftable(u) && isKeypointVisible(u)) ) {
    u.setRandom();
    // 0.8 is a magic number to keep the points away from the border.
    u *= ((double) rand() / (double) RAND_MAX) * one_over_xixi_m_1 * 0.8;

    // Now we run the point through distortion and projection.
    // Apply distortion
    distortion_->distort(&u);

    u[0] = fu() * u[0] + cu();
    u[1] = fv() * u[1] + cv();

    // Protect against infinite loops.
    if(--max_tries < 1) {
      u << cu(), cv();  //image center
      LOG(ERROR) << "UnifiedProjectionCamera::createRandomKeypoint "
          << "failed to produce a random keypoint!";
      break;
    }
  }
  return u;
}

Eigen::Vector3d UnifiedProjectionCamera::createRandomVisiblePoint(double depth) const {
  CHECK_GT(depth, 0.0) << "Depth needs to be positive!";

  Eigen::Vector2d y = createRandomKeypoint();

  Eigen::Vector3d point_3d;
  bool success = backProject3(y, &point_3d);
  CHECK(success) << "backprojection of createRandomVisiblePoint was unsuccessful!";
  point_3d /= point_3d.norm();

  // Muck with the depth. This doesn't change the pointing direction.
  point_3d *= depth;

  Eigen::Vector2d yhat;
  CHECK(project3(point_3d, &yhat));
  return point_3d;
}

bool UnifiedProjectionCamera::areParametersValid(const Eigen::VectorXd& parameters) {
  return (parameters.size() == parameterCount()) &&
         (parameters[0] >= 0.0) && //xi
         (parameters[1] > 0.0)  && //fu
         (parameters[2] > 0.0)  && //fv
         (parameters[3] > 0.0)  && //cu
         (parameters[4] > 0.0);    //cv
}

bool UnifiedProjectionCamera::intrinsicsValid(
    const Eigen::VectorXd& intrinsics) const {
  return areParametersValid(intrinsics);
}

void UnifiedProjectionCamera::printParameters(std::ostream& out, const std::string& text) const {
  Camera::printParameters(out, text);
  out << "  mirror parameter (xi): "
      << xi() << std::endl;
  out << "  focal length (cols,rows): "
      << fu() << ", " << fv() << std::endl;
  out << "  optical center (cols,rows): "
      << cu() << ", " << cv() << std::endl;

  out << "  distortion: ";
  distortion_->printParameters(out, text);
}

bool UnifiedProjectionCamera::isValidImpl() const {
  return intrinsicsValid(intrinsics_);
}

void UnifiedProjectionCamera::setRandomImpl() {
  UnifiedProjectionCamera::Ptr test_camera =
      UnifiedProjectionCamera::createTestCamera();
  CHECK(test_camera);
  line_delay_nanoseconds_ = test_camera->line_delay_nanoseconds_;
  image_width_ = test_camera->image_width_;
  image_height_ = test_camera->image_height_;
  mask_= test_camera->mask_;
  intrinsics_ = test_camera->intrinsics_;
  camera_type_ = test_camera->camera_type_;
  if (test_camera->distortion_) {
    distortion_ = std::move(test_camera->distortion_);
  }
}

bool UnifiedProjectionCamera::isEqualImpl(const Sensor& other, const bool verbose) const {
  const UnifiedProjectionCamera* other_camera =
      dynamic_cast<const UnifiedProjectionCamera*>(&other);
  if (other_camera == nullptr) {
    return false;
  }

  // Verify that the base members are equal.
  if (!isEqualCameraImpl(*other_camera, verbose)) {
    return false;
  }

  // Compare the distortion model (if distortion is set for both).
  if (!(*(this->distortion_) == *(other_camera->distortion_))) {
    return false;
  }

  return true;
}

UnifiedProjectionCamera::Ptr UnifiedProjectionCamera::createTestCamera() {
  UnifiedProjectionCamera::Ptr camera(
      new UnifiedProjectionCamera(0.9, 400, 300, 320, 240, 640, 480));
  CameraId id;
  generateId(&id);
  camera->setId(id);
  return camera;
}
}  // namespace aslam
