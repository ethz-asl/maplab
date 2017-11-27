#include <algorithm>
#include <functional>
#include <vector>

#include <Eigen/Core>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include <product-quantization/learn-product-quantization.h>

namespace product_quantization {

TEST(LearnProductQuantizationTest, EigenvalueAllocationWorks) {
  Eigen::MatrixXf rotation_matrix;
  rotation_matrix.setIdentity(6, 6);
  std::vector<float> variances = {3.0, 1.0, 2.0, 4.0, 7.0, 1.5};

  Eigen::MatrixXf expected_matrix_one_component(6, 6);
  expected_matrix_one_component << 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
  Eigen::MatrixXf permutated_rotation;
  EigenvalueAllocation(rotation_matrix, variances, 1, &permutated_rotation);
  EXPECT_NEAR_EIGEN(expected_matrix_one_component, permutated_rotation, 0.0);

  Eigen::MatrixXf expected_matrix_two_components(6, 6);
  expected_matrix_two_components << 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
      0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  EigenvalueAllocation(rotation_matrix, variances, 2, &permutated_rotation);
  EXPECT_NEAR_EIGEN(expected_matrix_two_components, permutated_rotation, 0.0);

  Eigen::MatrixXf expected_matrix_three_components(6, 6);
  expected_matrix_three_components << 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
  EigenvalueAllocation(rotation_matrix, variances, 3, &permutated_rotation);
  EXPECT_NEAR_EIGEN(expected_matrix_three_components, permutated_rotation, 0.0);
}

TEST(LearnProductQuantizationTest, ComputePCARotationWorks) {
  Eigen::MatrixXf data_points(3, 7);
  data_points << 0.16218, 0.52853, 0.26297, 0.74815, 0.22898, 0.82582, 0.078176,
      0.79428, 0.16565, 0.65408, 0.45054, 0.91334, 0.53834, 0.44268, 0.31122,
      0.60198, 0.68921, 0.083821, 0.15238, 0.99613, 0.10665;

  Eigen::MatrixXf expected_rotation(3, 3);
  expected_rotation << 0.58313, -0.30308, 0.75372, -0.46414, 0.63716, 0.6153,
      0.66673, 0.70864, -0.23088;

  std::vector<float> expected_variances = {0.166135221203798, 0.067722866745168,
                                           0.037692582036796};

  Eigen::MatrixXf rotation_matrix;
  std::vector<float> variances;
  ComputePCARotation(data_points, &rotation_matrix, &variances);

  // For convenience, we use the EigenvalueAllocation algorithm to sort the
  // rotation matrix based on the variances. This simplifies the comparison with
  // the expected rotation and variances.
  Eigen::MatrixXf sorted_rotation_matrix;
  EigenvalueAllocation(rotation_matrix, variances, 1, &sorted_rotation_matrix);
  // Correct the signs of the rotation matrix.
  for (int i = 0; i < 3; ++i) {
    if (sorted_rotation_matrix.row(i).dot(expected_rotation.row(i)) < 0.0) {
      sorted_rotation_matrix.row(i) *= -1.0f;
    }
  }
  EXPECT_NEAR_EIGEN(expected_rotation, sorted_rotation_matrix, 1e-4);

  // Compares the eigenvalues / variances.
  std::sort(variances.begin(), variances.end(), std::greater<float>());
  EXPECT_NEAR(expected_variances[0], variances[0], 1e-3);
  EXPECT_NEAR(expected_variances[1], variances[1], 1e-3);
  EXPECT_NEAR(expected_variances[2], variances[2], 1e-3);
}
}  // namespace product_quantization

MAPLAB_UNITTEST_ENTRYPOINT
