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

#ifndef TESTING_PREDICATES_H_
#define TESTING_PREDICATES_H_

#include <gtest/gtest.h>
#include <glog/logging.h>

#define __INTERNAL_GTEST_NEAR_EIGEN(PREDICATE, matrix_A, matrix_B, precision) \
  PREDICATE##_TRUE((matrix_A).isApprox((matrix_B), precision))                \
      << "For matrices '" << #matrix_A << "' and '" << #matrix_B << "'."      \
      << std::endl << "Where '" << #matrix_A << "' equals: " << std::endl     \
      << (matrix_A) << std::endl << "and '" << #matrix_B                      \
      << "' equals: " << std::endl << (matrix_B) << std::endl                 \
      << "and precision equals: " << precision;

#define EXPECT_NEAR_EIGEN(matrix_A, matrix_B, precision) \
  __INTERNAL_GTEST_NEAR_EIGEN(EXPECT, matrix_A, matrix_B, precision)

#define ASSERT_NEAR_EIGEN(matrix_A, matrix_B, precision) \
  __INTERNAL_GTEST_NEAR_EIGEN(ASSERT, matrix_A, matrix_B, precision)

#endif  // TESTING_PREDICATES_H_
