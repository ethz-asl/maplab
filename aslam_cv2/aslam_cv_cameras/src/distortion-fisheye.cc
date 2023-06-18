#include <aslam/cameras/distortion-fisheye.h>

namespace aslam {
std::ostream& operator<<(std::ostream& out, const FisheyeDistortion& distortion) {
  distortion.printParameters(out, std::string(""));
  return out;
}

FisheyeDistortion::FisheyeDistortion(const Eigen::VectorXd& dist_coeffs)
: Base(dist_coeffs, Distortion::Type::kFisheye) {
  CHECK(distortionParametersValid(dist_coeffs)) << dist_coeffs.transpose();
}

void FisheyeDistortion::distortUsingExternalCoefficients(const Eigen::VectorXd* dist_coeffs,
                                                         Eigen::Vector2d* point,
                                                         Eigen::Matrix2d* out_jacobian) const {
  CHECK_NOTNULL(point);

  // Use internal params if dist_coeffs==nullptr
  if(!dist_coeffs)
    dist_coeffs = &distortion_coefficients_;
  CHECK_EQ(dist_coeffs->size(), kNumOfParams) << "dist_coeffs: invalid size!";

  const double& w = (*dist_coeffs)(0);
  const double r_u = point->norm();
  const double r_u_cubed = r_u * r_u * r_u;
  const double tanwhalf = tan(w / 2.);
  const double tanwhalfsq = tanwhalf * tanwhalf;
  const double atan_wrd = atan(2. * tanwhalf * r_u);
  double r_rd;

  if (w * w < 1e-5) {
    // Limit w > 0.
    r_rd = 1.0;
  } else {
    if (r_u * r_u < 1e-5) {
      // Limit r_u > 0.
      r_rd = 2. * tanwhalf / w;
    } else {
      r_rd = atan_wrd / (r_u * w);
    }
  }

  const double& u = (*point)(0);
  const double& v = (*point)(1);

  // If Jacobian calculation is requested.
  if (out_jacobian) {
    out_jacobian->resize(2, 2);
    if (w * w < 1e-5) {
      out_jacobian->setIdentity();
    }
    else if (r_u * r_u < 1e-5) {
      out_jacobian->setIdentity();
      // The coordinates get multiplied by an expression not depending on r_u.
      *out_jacobian *= (2. * tanwhalf / w);
    }
    else {
      const double duf_du = (atan_wrd) / (w * r_u)
                - (u * u * atan_wrd) / (w * r_u_cubed)
                + (2 * u * u * tanwhalf)
                / (w * (u * u + v * v) * (4 * tanwhalfsq * (u * u + v * v) + 1));
      const double duf_dv = (2 * u * v * tanwhalf)
                / (w * (u * u + v * v) * (4 * tanwhalfsq * (u * u + v * v) + 1))
                - (u * v * atan_wrd) / (w * r_u_cubed);
      const double dvf_du = (2 * u * v * tanwhalf)
                / (w * (u * u + v * v) * (4 * tanwhalfsq * (u * u + v * v) + 1))
                - (u * v * atan_wrd) / (w * r_u_cubed);
      const double dvf_dv = (atan_wrd) / (w * r_u)
                - (v * v * atan_wrd) / (w * r_u_cubed)
                + (2 * v * v * tanwhalf)
                / (w * (u * u + v * v) * (4 * tanwhalfsq * (u * u + v * v) + 1));

      *out_jacobian << duf_du, duf_dv,
                       dvf_du, dvf_dv;
    }
  }

  *point *= r_rd;
}

void FisheyeDistortion::distortParameterJacobian(const Eigen::VectorXd* dist_coeffs,
                                                 const Eigen::Vector2d& point,
                                                 Eigen::Matrix<double, 2, Eigen::Dynamic>* out_jacobian) const {
  CHECK_EQ(dist_coeffs->size(), kNumOfParams) << "dist_coeffs: invalid size!";
  CHECK_NOTNULL(out_jacobian);

  const double& w = (*dist_coeffs)(0);

  const double tanwhalf = tan(w / 2.);
  const double tanwhalfsq = tanwhalf * tanwhalf;
  const double r_u = point.norm();
  const double atan_wrd = atan(2. * tanwhalf * r_u);

  const double& u = point(0);
  const double& v = point(1);

  out_jacobian->resize(2, kNumOfParams);
  if (w * w < 1e-5) {
    out_jacobian->setZero();
  }
  else if (r_u * r_u < 1e-5) {
    out_jacobian->setOnes();
    *out_jacobian *= (w - sin(w)) / (w * w * cos(w / 2) * cos(w / 2));
  }
  else {
    const double dxd_d_w = (2 * u * (tanwhalfsq / 2 + 0.5))
          / (w * (4 * tanwhalfsq * r_u * r_u + 1))
          - (u * atan_wrd) / (w * w * r_u);

    const double dyd_d_w = (2 * v * (tanwhalfsq / 2 + 0.5))
          / (w * (4 * tanwhalfsq * r_u * r_u + 1))
          - (v * atan_wrd) / (w * w * r_u);

    *out_jacobian << dxd_d_w, dyd_d_w;
  }
}

void FisheyeDistortion::undistortUsingExternalCoefficients(const Eigen::VectorXd& dist_coeffs,
                                                           Eigen::Vector2d* point) const {
  CHECK_NOTNULL(point);
  CHECK_EQ(dist_coeffs.size(), kNumOfParams) << "dist_coeffs: invalid size!";

  const double& w = dist_coeffs(0);
  double mul2tanwby2 = tan(w / 2.0) * 2.0;

  // Calculate distance from point to center.
  double r_d = point->norm();

  if (mul2tanwby2 == 0 || r_d == 0) {
    return;
  }

  // Calculate undistorted radius of point.
  double r_u;
  if (fabs(r_d * w) <= kMaxValidAngle) {
    r_u = tan(r_d * w) / (r_d * mul2tanwby2);
  } else {
    return;
  }

  (*point) *= r_u;
}

bool FisheyeDistortion::areParametersValid(const Eigen::VectorXd& parameters) {
  // Check the vector size.
  if (parameters.size() != kNumOfParams)
    return false;

  // Expect w to have sane magnitude.
  double w = parameters(0);
  bool valid = std::abs(w) < 1e-16 || (w >= kMinValidW && w <= kMaxValidW);
  LOG_IF(INFO, !valid) << "Invalid w parameter: " << w << ", expected w in [" << kMinValidW
      << ", " << kMaxValidW << "].";
  return valid;
}

bool FisheyeDistortion::distortionParametersValid(const Eigen::VectorXd& dist_coeffs) const {
  return areParametersValid(dist_coeffs);
}

void FisheyeDistortion::printParameters(std::ostream& out, const std::string& text) const {
  const Eigen::VectorXd& distortion_coefficients = getParameters();
  CHECK_EQ(distortion_coefficients.size(), kNumOfParams) << "dist_coeffs: invalid size!";

  out << text << std::endl;
  out << "Distortion: (FisheyeDistortion) " << std::endl;
  out << "  w: " << distortion_coefficients(0) << std::endl;
}

} // namespace aslam
