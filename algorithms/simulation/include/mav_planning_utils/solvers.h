/*

Copyright (c) 2013, Markus Achtelik, ASL, ETH Zurich, Switzerland
You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef MAV_PLANNING_UTILS_SOLVERS_H_
#define MAV_PLANNING_UTILS_SOLVERS_H_

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, 1> pseudoInverseSolver(
    const Eigen::MatrixBase<Derived>& A,
    const Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, 1>& b,
    double tol = 1e-12) {
  // Eigen 3.2.9 changed the Jacobi SVD solver which lead to inaccurate
  // results in the trajectory generation. To fix this, directly use the
  // inverse if the matrix is square.
  if (A.rows() == A.cols()) {
    return A.inverse() * b;
  }
  typedef Derived A_type;
  typedef Eigen::Matrix<typename A_type::Scalar, A_type::RowsAtCompileTime, 1> X_type;

  Eigen::JacobiSVD<A_type> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

  A_type U, V;
  X_type S;

  U = svd.matrixU();
  V = svd.matrixV();
  S = svd.singularValues();

  for (int i = 0; i < A.rows(); i++) {
    if (S[i] < tol)
      S[i] = 0;
    else
      S[i] = 1.0 / S[i];
  }

  return V * S.asDiagonal() * U.transpose() * b;
}

#endif  // MAV_PLANNING_UTILS_SOLVERS_H_
