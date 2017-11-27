#include <algorithm>
#include <functional>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <glog/logging.h>

#include <product-quantization/learn-product-quantization.h>

namespace product_quantization {
void ComputePCARotation(
    const Eigen::MatrixXf& data_points, Eigen::MatrixXf* rotation_matrix,
    std::vector<float>* variances) {
  CHECK_NOTNULL(rotation_matrix);
  CHECK_NOTNULL(variances);
  CHECK_GE(data_points.cols(), data_points.rows());

  int num_dimensions = data_points.rows();
  int num_data_points = data_points.cols();
  rotation_matrix->resize(num_dimensions, num_dimensions);
  variances->resize(num_dimensions);

  Eigen::VectorXf mean = data_points.rowwise().mean();

  Eigen::MatrixXf covariance_matrix;
  covariance_matrix.setZero(num_dimensions, num_dimensions);
  for (int i = 0; i < num_data_points; ++i) {
    Eigen::VectorXf p = data_points.col(i) - mean;
    covariance_matrix += p * p.transpose();
  }
  covariance_matrix /= static_cast<float>(num_data_points - 1);

  Eigen::EigenSolver<Eigen::MatrixXf> eigen_solver(covariance_matrix);
  Eigen::VectorXf eigenvalues = eigen_solver.eigenvalues().real();
  *rotation_matrix = eigen_solver.eigenvectors().real().transpose();

  for (int i = 0; i < num_dimensions; ++i) {
    CHECK_GE(eigenvalues(i), 0.0);
    (*variances)[i] = eigenvalues(i);
  }
}

void EigenvalueAllocation(
    const Eigen::MatrixXf& rotation_matrix, const std::vector<float>& variances,
    int num_components, Eigen::MatrixXf* permutated_rotation_matrix) {
  CHECK_EQ(rotation_matrix.cols(), rotation_matrix.rows());
  CHECK_EQ(static_cast<unsigned int>(rotation_matrix.cols()), variances.size());
  CHECK_EQ(rotation_matrix.cols() % num_components, 0);
  CHECK_NOTNULL(permutated_rotation_matrix);

  int num_dimensions = rotation_matrix.cols();
  permutated_rotation_matrix->setIdentity(num_dimensions, num_dimensions);
  // Sorts the rows in decreasing order of variance.
  std::vector<std::pair<float, int> > variance_index_pairs(num_dimensions);
  for (int i = 0; i < num_dimensions; ++i) {
    variance_index_pairs[i].first = variances[i];
    variance_index_pairs[i].second = i;
  }
  std::sort(
      variance_index_pairs.begin(), variance_index_pairs.end(),
      std::greater<std::pair<float, int> >());

  // Performs EigenvalueAllocation by balancing the variances in a greedy
  // fashion: Given the sorted variances, the algorithm iteratively selects the
  // component with the minimum product of variances for which we have not
  // yet selected enough rows.
  std::vector<float> variance_product_per_component(num_components, -1.0f);
  std::vector<int> num_selected_dimensions_per_component(num_components, 0);
  int max_num_dimensions_per_component = num_dimensions / num_components;
  for (int i = 0; i < num_dimensions; ++i) {
    // Finds the component with the minimum product of variances.
    int selected_component = -1;
    float min_product_variance = std::numeric_limits<float>::max();
    for (int j = 0; j < num_components; ++j) {
      if (num_selected_dimensions_per_component[j] ==
          max_num_dimensions_per_component) {
        continue;
      }

      if (variance_product_per_component[j] < min_product_variance) {
        min_product_variance = variance_product_per_component[j];
        selected_component = j;
      }
    }
    CHECK_GE(selected_component, 0);
    if (min_product_variance == -1.0f) {
      variance_product_per_component[selected_component] =
          variance_index_pairs[i].first;
    } else {
      variance_product_per_component[selected_component] *=
          variance_index_pairs[i].first;
    }
    permutated_rotation_matrix->row(
        selected_component * max_num_dimensions_per_component +
        num_selected_dimensions_per_component[selected_component]) =
        rotation_matrix.row(variance_index_pairs[i].second);
    ++num_selected_dimensions_per_component[selected_component];
  }
}
}  // namespace product_quantization
