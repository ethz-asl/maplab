/*
 * Copyright (C) 2014 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MAPLAB_COMMON_TESTING_PREDICATES_H_
#define MAPLAB_COMMON_TESTING_PREDICATES_H_
#include <cmath>
#include <string>

#include <glog/logging.h>
#include <gtest/gtest.h>

namespace common {
template <typename LeftMat, typename RightMat>
::testing::AssertionResult MatricesEqual(
    const LeftMat& A, const RightMat& B, double threshold) {
  if (A.rows() != B.rows() || A.cols() != B.cols()) {
    return ::testing::AssertionFailure()
           << "Matrix size mismatch: " << A.rows() << "x" << A.cols()
           << " != " << B.rows() << "x" << B.cols();
  }

  bool success = true;
  std::string message;
  for (int i = 0; i < A.rows(); ++i) {
    for (int j = 0; j < A.cols(); ++j) {
      double Aij = A(i, j);
      double Bij = B(i, j);
      if (std::abs(Aij - Bij) > threshold) {
        success = false;
        message += "\n  Mismatch at [" + std::to_string(i) + "," +
                   std::to_string(j) + "] : " + std::to_string(Aij) + " != " +
                   std::to_string(Bij);
      }
    }
  }

  return success ? ::testing::AssertionSuccess() : ::testing::AssertionFailure()
                                                       << message << std::endl;
}
}  // namespace common

#define __INTERNAL_GTEST_NEAR_EIGEN(PREDICATE, matrix_A, matrix_B, precision) \
  PREDICATE##_TRUE(                                                           \
      ((matrix_A) - (matrix_B)).cwiseAbs().maxCoeff() <= precision)           \
      << "For matrices '" << #matrix_A << "' and '" << #matrix_B << "'."      \
      << std::endl                                                            \
      << "Where '" << #matrix_A << "' equals: " << std::endl                  \
      << (matrix_A) << std::endl                                              \
      << "and '" << #matrix_B << "' equals: " << std::endl                    \
      << (matrix_B) << std::endl                                              \
      << "and precision equals: " << precision;

#define __INTERNAL_GTEST_ZERO_EIGEN(PREDICATE, matrix_A, precision) \
  PREDICATE##_TRUE((matrix_A).isZero(precision))                    \
      << "For matrix '" << #matrix_A << "'." << std::endl           \
      << "Where '" << #matrix_A << "' equals: " << std::endl        \
      << (matrix_A) << std::endl                                    \
      << "and precision equals: " << precision;

#define __INTERNAL_GTEST_NEAR_KINDR_QUATERNION(                               \
    PREDICATE, quat_A, quat_B, precision)                                     \
  PREDICATE##_TRUE((quat_A).getDisparityAngle(quat_B) <= precision)           \
      << "For quaternions '" << #quat_A << "' and '" << #quat_B << "'."       \
      << std::endl                                                            \
      << "Where '" << #quat_A << "' equals: " << (quat_A).getUnique()         \
      << std::endl                                                            \
      << "and '" << #quat_B << "' equals: " << (quat_B).getUnique()           \
      << std::endl                                                            \
      << "the disparity angle equals: " << (quat_A).getDisparityAngle(quat_B) \
      << " rad" << std::endl                                                  \
      << "and precision equals: " << precision << " rad";

#define __INTERNAL_GTEST_NEAR_EIGEN_QUATERNION(                               \
    PREDICATE, quat_A, quat_B, precision)                                     \
  PREDICATE##_TRUE((quat_A).isApprox((quat_B), precision))                    \
      << "For quaternions '" << #quat_A << "' and '" << #quat_B << "'."       \
      << std::endl                                                            \
      << "Where '" << #quat_A << "' equals: " << (quat_A).coeffs()            \
      << std::endl                                                            \
      << "and '" << #quat_B << "' equals: " << (quat_B).coeffs() << std::endl \
      << "and precision equals: " << precision << " rad";

#define EXPECT_NEAR_EIGEN(matrix_A, matrix_B, precision) \
  __INTERNAL_GTEST_NEAR_EIGEN(EXPECT, matrix_A, matrix_B, precision)

#define ASSERT_NEAR_EIGEN(matrix_A, matrix_B, precision) \
  __INTERNAL_GTEST_NEAR_EIGEN(ASSERT, matrix_A, matrix_B, precision)

#define EXPECT_ZERO_EIGEN(matrix_A, precision) \
  __INTERNAL_GTEST_ZERO_EIGEN(EXPECT, matrix_A, precision)

#define ASSERT_ZERO_EIGEN(matrix_A, precision) \
  __INTERNAL_GTEST_ZERO_EIGEN(ASSERT, matrix_A, precision)

#define EXPECT_NEAR_EIGEN_QUATERNION(quat_A, quat_B, precision) \
  __INTERNAL_GTEST_NEAR_EIGEN_QUATERNION(EXPECT, quat_A, quat_B, precision)

#define ASSERT_NEAR_EIGEN_QUATERNION(quat_A, quat_B, precision) \
  __INTERNAL_GTEST_NEAR_EIGEN_QUATERNION(ASSERT, quat_A, quat_B, precision)

#define EXPECT_NEAR_KINDR_QUATERNION(quat_A, quat_B, precision) \
  __INTERNAL_GTEST_NEAR_KINDR_QUATERNION(EXPECT, quat_A, quat_B, precision)

#define ASSERT_NEAR_KINDR_QUATERNION(quat_A, quat_B, precision) \
  __INTERNAL_GTEST_NEAR_KINDR_QUATERNION(ASSERT, quat_A, quat_B, precision)

#define EXPECT_NEAR_ASLAM_TRANSFORMATION(T_A, T_B, precision) \
  __INTERNAL_GTEST_NEAR_EIGEN(                                \
      EXPECT, (T_A).getTransformationMatrix(),                \
      (T_B).getTransformationMatrix(), precision)

#define ASSERT_NEAR_ASLAM_TRANSFORMATION(T_A, T_B, precision) \
  __INTERNAL_GTEST_NEAR_EIGEN(                                \
      ASSERT, (T_A).getTransformationMatrix(),                \
      (T_B).getTransformationMatrix(), precision)

#endif  // MAPLAB_COMMON_TESTING_PREDICATES_H_
