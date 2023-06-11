#include <memory>
#include <utility>

#include <aslam/cameras/camera-pinhole.h>

#include <aslam/cameras/camera-factory.h>
#include <aslam/common/types.h>

#include "aslam/cameras/random-camera-generator.h"

namespace aslam {
std::ostream& operator<<(std::ostream& out, const PinholeCamera& camera) {
  camera.printParameters(out, std::string(""));
  return out;
}

PinholeCamera::PinholeCamera()
    : Base(Eigen::Vector4d::Zero(), 0, 0, Camera::Type::kPinhole) {}

PinholeCamera::PinholeCamera(const Eigen::VectorXd& intrinsics,
                             uint32_t image_width, uint32_t image_height,
                             aslam::Distortion::UniquePtr& distortion)
  : Base(intrinsics, distortion, image_width, image_height, Camera::Type::kPinhole) {
  CHECK(intrinsicsValid(intrinsics));
}

PinholeCamera::PinholeCamera(const Eigen::VectorXd& intrinsics, uint32_t image_width,
                             uint32_t image_height)
    : Base(intrinsics, image_width, image_height, Camera::Type::kPinhole) {
  CHECK(intrinsicsValid(intrinsics));
}

PinholeCamera::PinholeCamera(double focallength_cols, double focallength_rows,
                             double imagecenter_cols, double imagecenter_rows, uint32_t image_width,
                             uint32_t image_height, aslam::Distortion::UniquePtr& distortion)
    : PinholeCamera(
        Eigen::Vector4d(focallength_cols, focallength_rows, imagecenter_cols, imagecenter_rows),
        image_width, image_height, distortion) {}

PinholeCamera::PinholeCamera(double focallength_cols, double focallength_rows,
                             double imagecenter_cols, double imagecenter_rows, uint32_t image_width,
                             uint32_t image_height)
    : PinholeCamera(
        Eigen::Vector4d(focallength_cols, focallength_rows, imagecenter_cols, imagecenter_rows),
        image_width, image_height) {}

bool PinholeCamera::backProject3(const Eigen::Ref<const Eigen::Vector2d>& keypoint,
                                 Eigen::Vector3d* out_point_3d) const {
  CHECK_NOTNULL(out_point_3d);

  Eigen::Vector2d kp = keypoint;
  kp[0] = (kp[0] - cu()) / fu();
  kp[1] = (kp[1] - cv()) / fv();

  distortion_->undistort(&kp);

  (*out_point_3d)[0] = kp[0];
  (*out_point_3d)[1] = kp[1];
  (*out_point_3d)[2] = 1;

  // Always valid for the pinhole model.
  return true;
}

const ProjectionResult PinholeCamera::project3Functional(
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

  const double& fu = (*intrinsics)[0];
  const double& fv = (*intrinsics)[1];
  const double& cu = (*intrinsics)[2];
  const double& cv = (*intrinsics)[3];

  // Project the point.
  const double& x = point_3d[0];
  const double& y = point_3d[1];
  const double& z = point_3d[2];

  const double rz = 1.0 / z;
  (*out_keypoint)[0] = x * rz;
  (*out_keypoint)[1] = y * rz;

  // Distort the point and get the Jacobian wrt. keypoint.
  Eigen::Matrix2d J_distortion = Eigen::Matrix2d::Identity();
  if(out_jacobian_distortion) {
    // Calculate the Jacobian w.r.t to the distortion parameters,
    // if requested (and distortion set).
    distortion_->distortParameterJacobian(distortion_coefficients,
                                          *out_keypoint,
                                          out_jacobian_distortion);
    out_jacobian_distortion->row(0) *= fu;
    out_jacobian_distortion->row(1) *= fv;
  }

  if(out_jacobian_point) {
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

  if(out_jacobian_point) {
    // Jacobian including distortion
    const double rz2 = rz * rz;

    const double duf_dx =  fu * J_distortion(0, 0) * rz;
    const double duf_dy =  fu * J_distortion(0, 1) * rz;
    const double duf_dz = -fu * (x * J_distortion(0, 0) + y * J_distortion(0, 1)) * rz2;
    const double dvf_dx =  fv * J_distortion(1, 0) * rz;
    const double dvf_dy =  fv * J_distortion(1, 1) * rz;
    const double dvf_dz = -fv * (x * J_distortion(1, 0) + y * J_distortion(1, 1)) * rz2;

    (*out_jacobian_point) << duf_dx, duf_dy, duf_dz,
                             dvf_dx, dvf_dy, dvf_dz;
  }

  // Calculate the Jacobian w.r.t to the intrinsic parameters, if requested.
  if(out_jacobian_intrinsics) {
    out_jacobian_intrinsics->resize(2, kNumOfParams);
    const double duf_dfu = (*out_keypoint)[0];
    const double duf_dfv = 0.0;
    const double duf_dcu = 1.0;
    const double duf_dcv = 0.0;
    const double dvf_dfu = 0.0;
    const double dvf_dfv = (*out_keypoint)[1];
    const double dvf_dcu = 0.0;
    const double dvf_dcv = 1.0;

    (*out_jacobian_intrinsics) << duf_dfu, duf_dfv, duf_dcu, duf_dcv,
                                  dvf_dfu, dvf_dfv, dvf_dcu, dvf_dcv;
  }

  // Normalized image plane to camera plane.
  (*out_keypoint)[0] = fu * (*out_keypoint)[0] + cu;
  (*out_keypoint)[1] = fv * (*out_keypoint)[1] + cv;

  return evaluateProjectionResult(*out_keypoint, point_3d);
}

Eigen::Vector2d PinholeCamera::createRandomKeypoint() const {
  Eigen::Vector2d out;
  out.setRandom();
  // Unit tests often fail when the point is near the border. Keep the point
  // away from the border.
  double border = std::min(imageWidth(), imageHeight()) * 0.1;

  out(0) = border + std::abs(out(0)) * (imageWidth() - border * 2.0);
  out(1) = border + std::abs(out(1)) * (imageHeight() - border * 2.0);

  return out;
}

Eigen::Vector3d PinholeCamera::createRandomVisiblePoint(double depth) const {
  CHECK_GT(depth, 0.0) << "Depth needs to be positive!";
  Eigen::Vector3d point_3d;

  Eigen::Vector2d y = createRandomKeypoint();
  backProject3(y, &point_3d);
  point_3d /= point_3d.norm();

  // Muck with the depth. This doesn't change the pointing direction.
  return point_3d * depth;
}

void PinholeCamera::getBorderRays(Eigen::MatrixXd& rays) const {
  rays.resize(4, 8);
  Eigen::Vector4d ray;
  backProject4(Eigen::Vector2d(0.0, 0.0), &ray);
  rays.col(0) = ray;
  backProject4(Eigen::Vector2d(0.0, imageHeight() * 0.5), &ray);
  rays.col(1) = ray;
  backProject4(Eigen::Vector2d(0.0, imageHeight() - 1.0), &ray);
  rays.col(2) = ray;
  backProject4(Eigen::Vector2d(imageWidth() - 1.0, 0.0), &ray);
  rays.col(3) = ray;
  backProject4(Eigen::Vector2d(imageWidth() - 1.0, imageHeight() * 0.5), &ray);
  rays.col(4) = ray;
  backProject4(Eigen::Vector2d(imageWidth() - 1.0, imageHeight() - 1.0), &ray);
  rays.col(5) = ray;
  backProject4(Eigen::Vector2d(imageWidth() * 0.5, 0.0), &ray);
  rays.col(6) = ray;
  backProject4(Eigen::Vector2d(imageWidth() * 0.5, imageHeight() - 1.0), &ray);
  rays.col(7) = ray;
}

bool PinholeCamera::areParametersValid(const Eigen::VectorXd& parameters) {
  return (parameters.size() == parameterCount()) &&
         (parameters[0] > 0.0)  && //fu
         (parameters[1] > 0.0)  && //fv
         (parameters[2] > 0.0)  && //cu
         (parameters[3] > 0.0);    //cv
}

bool PinholeCamera::intrinsicsValid(const Eigen::VectorXd& intrinsics) const {
  return areParametersValid(intrinsics);
}

void PinholeCamera::printParameters(std::ostream& out, const std::string& text) const {
  Camera::printParameters(out, text);
  out << "  focal length (cols,rows): "
      << fu() << ", " << fv() << std::endl;
  out << "  optical center (cols,rows): "
      << cu() << ", " << cv() << std::endl;

  out << "  distortion: ";
  distortion_->printParameters(out, text);
}
const double PinholeCamera::kMinimumDepth = 1e-10;

bool PinholeCamera::isValidImpl() const {
  return intrinsicsValid(intrinsics_);
}

void PinholeCamera::setRandomImpl() {
  PinholeCamera::Ptr test_camera = PinholeCamera::createTestCamera();
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

bool PinholeCamera::isEqualImpl(const Sensor& other, const bool verbose) const {
  const PinholeCamera* other_camera =
      dynamic_cast<const PinholeCamera*>(&other);
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

PinholeCamera::Ptr PinholeCamera::createTestCamera() {
  PinholeCamera::Ptr camera(new PinholeCamera(400, 300, 320, 240, 640, 480));
  CameraId id;
  generateId(&id);
  camera->setId(id);
  return camera;
}

}  // namespace aslam
