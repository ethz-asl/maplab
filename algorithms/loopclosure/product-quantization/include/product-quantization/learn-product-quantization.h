#ifndef PRODUCT_QUANTIZATION_LEARN_PRODUCT_QUANTIZATION_H_
#define PRODUCT_QUANTIZATION_LEARN_PRODUCT_QUANTIZATION_H_

#include <vector>

#include <Eigen/Core>

namespace product_quantization {
// Computes a rotation matrix based on PCA and returns both the rotation matrix
// and the variances along the corresponding axes. The i-th variance corresponds
// to the axis defined by the i-th row of the rotation matrix. Each column in
// data_points corresponds to a single data point.
void ComputePCARotation(
    const Eigen::MatrixXf& data_points, Eigen::MatrixXf* rotation_matrix,
    std::vector<float>* variances);

// Performs Eigenvalue allocation as described in
//   T. Ge, K. He, Q. Ke, J. Sun, Optimized Product Quantization, PAMI 2014:
// Given a rotation matrix and the variances corresponding to each row,
// e.g., computed with ComputePCARotation, and the number of parts into
// which the descriptor space should be divided (corresponding to the number
// of components of Product Quantization), re-orders the rotation matrix such
// that the variance is balanced among the different parts.
// Returns the re-ordered rotation matrix.
void EigenvalueAllocation(
    const Eigen::MatrixXf& rotation_matrix, const std::vector<float>& variances,
    int num_components, Eigen::MatrixXf* permutated_rotation_matrix);

}  // namespace product_quantization
#endif  // PRODUCT_QUANTIZATION_LEARN_PRODUCT_QUANTIZATION_H_
