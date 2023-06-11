#include <aslam/common/covariance-helpers.h>
#include <aslam/common/entrypoint.h>
#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>

namespace aslam {
namespace common {

TEST(TestCovarianceHelpers, ForwardAndBackwardRotation) {
  for (int i = 0; i < 1000; i++) {
    aslam::Transformation T_A_B, T_B_A;
    T_A_B.setRandom();
    T_B_A = T_A_B.inverse();
    aslam::TransformationCovariance A_covariance, B_covariance,
        A_covariance_expected;

    A_covariance_expected = Eigen::Matrix<double, 6, 6>::Random();
    rotateCovariance(T_B_A, A_covariance_expected, &B_covariance);
    rotateCovariance(T_A_B, B_covariance, &A_covariance);
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(A_covariance, A_covariance_expected, 1E-4));
  }
}

}  // namespace common
}  // namespace aslam

ASLAM_UNITTEST_ENTRYPOINT
