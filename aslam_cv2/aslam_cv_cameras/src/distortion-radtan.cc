#include <aslam/cameras/distortion-radtan.h>

namespace aslam {
std::ostream& operator<<(std::ostream& out, const RadTanDistortion& distortion) {
  distortion.printParameters(out, std::string(""));
  return out;
}

RadTanDistortion::RadTanDistortion(const Eigen::VectorXd& dist_coeffs)
: Base(dist_coeffs, Distortion::Type::kRadTan) {
  CHECK(distortionParametersValid(dist_coeffs)) << dist_coeffs.transpose();
}

void RadTanDistortion::distortUsingExternalCoefficients(
    const Eigen::VectorXd* dist_coeffs,
    Eigen::Vector2d* point,
    Eigen::Matrix2d* out_jacobian) const {
  CHECK_NOTNULL(point);

  double& x = (*point)(0);
  double& y = (*point)(1);

  // Use internal params if dist_coeffs==nullptr
  if(!dist_coeffs)
    dist_coeffs = &distortion_coefficients_;
  CHECK_EQ(dist_coeffs->size(), kNumOfParams) << "dist_coeffs: invalid size!";

  const double& k1 = (*dist_coeffs)(0); // The first radial distortion parameter.
  const double& k2 = (*dist_coeffs)(1); // The second radial distortion parameter.
  const double& p1 = (*dist_coeffs)(2); // The first tangential distortion parameter.
  const double& p2 = (*dist_coeffs)(3); // The second tangential distortion parameter.

  double mx2_u = x * x;
  double my2_u = y * y;
  double mxy_u = x * y;
  double rho2_u = mx2_u + my2_u;
  double rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;

  if (out_jacobian) {
    const double duf_du =   1.0 + rad_dist_u
                          + 2.0 * k1 * mx2_u
                          + 4.0 * k2 * rho2_u * mx2_u
                          + 2.0 * p1 * y
                          + 6.0 * p2 * x;

    const double duf_dv =   2.0 * k1 * mxy_u
                          + 4.0 * k2 * rho2_u * mxy_u
                          + 2.0 * p1 * x
                          + 2.0 * p2 * y;

    const double dvf_du = duf_dv;

    const double dvf_dv =   1.0 + rad_dist_u
                          + 2.0 * k1 * my2_u
                          + 4.0 * k2 * rho2_u * my2_u
                          + 2.0 * p2 * x
                          + 6.0 * p1 * y;

    *out_jacobian << duf_du, duf_dv,
                     dvf_du, dvf_dv;
  }

  x += x * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u);
  y += y * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

void RadTanDistortion::distortParameterJacobian(
    const Eigen::VectorXd* dist_coeffs,
    const Eigen::Vector2d& point,
    Eigen::Matrix<double, 2, Eigen::Dynamic>* out_jacobian) const {
  CHECK_EQ(dist_coeffs->size(), kNumOfParams) << "dist_coeffs: invalid size!";
  CHECK_NOTNULL(out_jacobian);

  const double& y0 = point(0);
  const double& y1 = point(1);
  const double r2 = y0 * y0 + y1 * y1;
  const double r4 = r2 * r2;

  const double duf_dk1 = y0 * r2;
  const double duf_dk2 = y0 * r4;
  const double duf_dp1 = 2.0 * y0 * y1;
  const double duf_dp2 = r2 + 2.0 * y0 * y0;
  const double dvf_dk1 = y1 * r2;
  const double dvf_dk2 = y1 * r4;
  const double dvf_dp1 = r2 + 2.0 * y1 * y1;
  const double dvf_dp2 = 2.0 * y0 * y1;

  out_jacobian->resize(2, kNumOfParams);
  (*out_jacobian) << duf_dk1, duf_dk2, duf_dp1, duf_dp2,
                     dvf_dk1, dvf_dk2, dvf_dp1, dvf_dp2;
}

void RadTanDistortion::undistortUsingExternalCoefficients(const Eigen::VectorXd& dist_coeffs,
                                                          Eigen::Vector2d* point) const {
  CHECK_EQ(dist_coeffs.size(), kNumOfParams) << "dist_coeffs: invalid size!";
  CHECK_NOTNULL(point);

  const int n = 30;  // Max. number of iterations

  Eigen::Vector2d& y = *point;
  Eigen::Vector2d ybar = y;
  Eigen::Matrix2d F;
  Eigen::Vector2d y_tmp;

  int i;
  for (i = 0; i < n; ++i) {
    y_tmp = ybar;
    distortUsingExternalCoefficients(&dist_coeffs, &y_tmp, &F);
    Eigen::Vector2d e(y - y_tmp);
    Eigen::Vector2d du = (F.transpose() * F).inverse() * F.transpose() * e;
    ybar += du;
    if (e.dot(e) <= FLAGS_acv_inv_distortion_tolerance)
      break;
  }
  LOG_IF(WARNING, i >= n) << "Did not converge with max. iterations.";

  y = ybar;
}

bool RadTanDistortion::areParametersValid(const Eigen::VectorXd& parameters) {
  // Just check the vector size.
  if (parameters.size() != kNumOfParams)
    return false;

  return true;
}

bool RadTanDistortion::distortionParametersValid(const Eigen::VectorXd& dist_coeffs) const {
  return areParametersValid(dist_coeffs);
}

void RadTanDistortion::printParameters(std::ostream& out, const std::string& text) const {
  Eigen::VectorXd distortion_coefficients = getParameters();
  CHECK_EQ(distortion_coefficients.size(), kNumOfParams) << "dist_coeffs: invalid size!";

  out << text << std::endl;
  out << "Distortion: (RadTanDistortion) " << std::endl;
  out << "  k1 (radial):     " << distortion_coefficients(0) << std::endl;
  out << "  k2 (radial):     " << distortion_coefficients(1) << std::endl;
  out << "  p1 (tangential): " << distortion_coefficients(2) << std::endl;
  out << "  p2 (tangential): " << distortion_coefficients(3) << std::endl;
}

} // namespace aslam
