#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <nav_msgs/Odometry.h>
#include "rovioli/ros-helpers.h"

namespace rovioli {

TEST(EigenOdometryConversion, EigenOdometryConversion) {
  // setting up expected values
  double expected_odom_format[36] = {
      0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17,
      18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35};
  nav_msgs::Odometry expected_odom;
  for (int i = 0; i < 36; i++) {
    expected_odom.pose.covariance[i] = expected_odom_format[i];
  }
  Eigen::Matrix<double, 6, 6> covariance, expected_covariance;
  expected_covariance << 0, 6, 12, 18, 24, 30, 1, 7, 13, 19, 25, 31, 2, 8, 14,
      20, 26, 32, 3, 9, 15, 21, 27, 33, 4, 10, 16, 22, 28, 34, 5, 11, 17, 23,
      29, 35;
  nav_msgs::Odometry odom;
  eigenMatrixToOdometryCovariance(
      expected_covariance, odom.pose.covariance.data());
  for (int i = 0; i < 36; i++) {
    CHECK_EQ(odom.pose.covariance[i], expected_odom_format[i]);
  }

  rovioli::odometryCovarianceToEigenMatrix(
      expected_odom.pose.covariance, covariance);
  for (int i = 0; i < expected_covariance.rows(); i++) {
    for (int j = 0; j < expected_covariance.cols(); j++) {
      CHECK_EQ(expected_covariance(i, j), covariance(i, j));
    }
  }
}

}  // namespace rovioli

MAPLAB_UNITTEST_ENTRYPOINT
