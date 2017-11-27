#include <geometric-vision/linear-triangulation.h>
#include <glog/logging.h>

#include <aslam/common/timer.h>
#include <maplab-common/quaternion-math.h>

namespace geometric_vision {

bool LinearTriangulation::triangulateFromNormalizedTwoViewsHomogeneous(
    const Eigen::Vector2d& measurement0,
    const pose::Transformation& camera_pose0,
    const Eigen::Vector2d& measurement1,
    const pose::Transformation& camera_pose1,
    Eigen::Vector3d* triangulated_point) {
  CHECK_NOTNULL(triangulated_point);

  const Eigen::Vector3d& G_p_C0 = camera_pose0.getPosition();
  const Eigen::Vector3d& G_p_C1 = camera_pose1.getPosition();
  const Eigen::Quaterniond& G_q_C0 =
      camera_pose0.getRotation().toImplementation();
  const Eigen::Quaterniond& G_q_C1 =
      camera_pose1.getRotation().toImplementation();
  Eigen::Matrix3d G_R_C0, G_R_C1;
  G_R_C0 = G_q_C0.toRotationMatrix();
  G_R_C1 = G_q_C1.toRotationMatrix();

  Eigen::Matrix<double, 3, 4> P0, P1;
  P0.block<3, 3>(0, 0) = G_R_C0;
  P1.block<3, 3>(0, 0) = G_R_C1;
  P0.block<3, 1>(0, 3) = G_p_C0;
  P1.block<3, 1>(0, 3) = G_p_C1;

  Eigen::Matrix<double, 4, 4> A;
  A.row(0) = measurement0(0) * P0.row(2) - P0.row(0);
  A.row(1) = measurement0(1) * P0.row(2) - P0.row(1);
  A.row(2) = measurement1(0) * P1.row(2) - P1.row(0);
  A.row(3) = measurement1(1) * P1.row(2) - P1.row(1);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
  Eigen::Vector4d triangulated_point_homogeneous = svd.matrixV().rightCols<1>();

  if (triangulated_point_homogeneous[3] == 0) {
    return false;
  }

  *triangulated_point = triangulated_point_homogeneous.hnormalized().head<3>();
  return true;
}

bool LinearTriangulation::triangulateFromNormalizedTwoViews(
    const Eigen::Vector2d& measurement0,
    const pose::Transformation& camera_pose0,
    const Eigen::Vector2d& measurement1,
    const pose::Transformation& camera_pose1,
    Eigen::Vector3d* triangulated_point) {
  CHECK_NOTNULL(triangulated_point);

  const Eigen::Vector3d& G_p_C0 = camera_pose0.getPosition();
  const Eigen::Vector3d& G_p_C1 = camera_pose1.getPosition();
  const Eigen::Quaterniond& G_q_C0 =
      camera_pose0.getRotation().toImplementation();
  const Eigen::Quaterniond& G_q_C1 =
      camera_pose1.getRotation().toImplementation();
  Eigen::Matrix3d G_R_C0, G_R_C1;
  G_R_C0 = G_q_C0.toRotationMatrix();
  G_R_C1 = G_q_C1.toRotationMatrix();

  Eigen::Matrix<double, 4, 3> A;
  Eigen::Vector4d b;

  A.row(0) = measurement0(0) * G_R_C0.row(2) - G_R_C0.row(0);
  A.row(1) = measurement0(1) * G_R_C0.row(2) - G_R_C0.row(1);
  A.row(2) = measurement1(0) * G_R_C1.row(2) - G_R_C1.row(0);
  A.row(3) = measurement1(1) * G_R_C1.row(2) - G_R_C1.row(1);

  b(0) = -(measurement0(0) * G_p_C0(2) - G_p_C0(0));
  b(1) = -(measurement0(1) * G_p_C0(2) - G_p_C0(1));
  b(2) = -(measurement1(0) * G_p_C1(2) - G_p_C1(0));
  b(3) = -(measurement1(1) * G_p_C1(2) - G_p_C1(1));

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (svd.nonzeroSingularValues() < 3) {
    VLOG(3) << "At least one singular value is zero";
    return false;
  }

  double condition_number;
  const Eigen::Vector3d singular_values = svd.singularValues();
  condition_number = singular_values.maxCoeff() / singular_values.minCoeff();
  VLOG(3) << "Condition number: " << condition_number;

  *triangulated_point = svd.solve(b);
  return true;
}

bool LinearTriangulation::triangulateFromNormalizedNViews(
    const Aligned<std::vector, Eigen::Vector2d>& measurements,
    const Aligned<std::vector, pose::Transformation>& G_T_C,
    Eigen::Vector3d* triangulated_point) {
  CHECK_NOTNULL(triangulated_point);
  CHECK_EQ(measurements.size(), G_T_C.size());
  CHECK_GE(measurements.size(), 2u);

  Eigen::Matrix<double, Eigen::Dynamic, 3> A;
  Eigen::VectorXd b;
  A.resize(2 * measurements.size(), Eigen::NoChange);
  b.resize(2 * measurements.size());

  for (unsigned int i = 0; i < measurements.size(); ++i) {
    pose::Transformation C_T_G = G_T_C[i].inverse();
    Eigen::Matrix3d C_R_G = C_T_G.getRotation().getRotationMatrix();
    const Eigen::Vector3d& C_p_G = C_T_G.getPosition();

    A.row(2 * i + 0) = measurements[i](0) * C_R_G.row(2) - C_R_G.row(0);
    A.row(2 * i + 1) = measurements[i](1) * C_R_G.row(2) - C_R_G.row(1);

    b(2 * i + 0) = -(measurements[i](0) * C_p_G(2) - C_p_G(0));
    b(2 * i + 1) = -(measurements[i](1) * C_p_G(2) - C_p_G(1));
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  *triangulated_point = svd.solve(b);

  //   Eigen::MatrixXd design_matrix(3 * measurements.size(),
  //                                 4 + measurements.size());
  //
  //   for (size_t i = 0; i < measurements.size(); i++) {
  //     design_matrix.block<3, 4>(3 * i, 0) =
  //         -camera_poses[i].getTransformationMatrix().block<3, 4>(0, 0);
  //     design_matrix.block<3, 1>(3 * i, 4 + i) << measurements[i], 1.0;
  //   }
  //
  //   // Computing SVD on A'A is more efficient and gives the same null-space.
  //   Eigen::Vector4d homog_triangulated_point =
  //       (design_matrix.transpose() *
  // design_matrix).jacobiSvd(Eigen::ComputeFullV)
  //           .matrixV().rightCols<1>().head(4);
  //   if (homog_triangulated_point[3] == 0) {
  //     return false;
  //   }
  //
  //   *triangulated_point = homog_triangulated_point.hnormalized();
  return true;
}

}  // namespace geometric_vision
