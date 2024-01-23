#include <aslam/common/covariance-helpers.h>

namespace aslam {
namespace common {

void rotateCovariance(
    const aslam::Transformation& T_B_A,
    const aslam::TransformationCovariance& A_covariance,
    aslam::TransformationCovariance* B_covariance) {
  Eigen::Matrix<double, 6, 6> covariance_transformation_B_A =
      Eigen::Matrix<double, 6, 6>::Identity();
  covariance_transformation_B_A.block<3, 3>(0, 0) = T_B_A.getRotationMatrix();
  covariance_transformation_B_A.block<3, 3>(3, 3) = T_B_A.getRotationMatrix();
  *B_covariance = covariance_transformation_B_A * A_covariance *
                  covariance_transformation_B_A.transpose();
}

}  // namespace common
}  // namespace aslam
