#include "maplab-common/histograms.h"

#include <cmath>
#include <limits>
#include <vector>

#include <glog/logging.h>

namespace common {

namespace histograms {

Eigen::MatrixXd logHistogram2dWithIndexAsX(
    const Eigen::MatrixXi y_values, const size_t num_x_bins,
    const size_t num_y_bins) {
  return logHistogram2dForKBestIndexMatching(
      y_values.colwise() +
          Eigen::VectorXi::Constant(y_values.rows(), 1, y_values.minCoeff()),
      num_x_bins, num_y_bins);
}

Eigen::MatrixXd logHistogram2dForKBestIndexMatching(
    const Eigen::MatrixXi matching_indices, const size_t num_x_bins,
    const size_t num_y_bins) {
  CHECK_GT(matching_indices.rows(), 0);
  const int x_ceiling = matching_indices.cols();
  CHECK_GT(x_ceiling, 0);
  const int y_ceiling = matching_indices.maxCoeff() + 1;
  Eigen::MatrixXi bin_counts(num_y_bins, num_x_bins);
  bin_counts.setZero();
  for (int x_index = 0; x_index < x_ceiling; ++x_index) {
    const int x = x_index * num_x_bins / x_ceiling;
    for (int i = 0; i < matching_indices.rows(); ++i) {
      if (matching_indices(i, x_index) < 0) {
        continue;
      }
      const int y = matching_indices(i, x_index) * num_y_bins / y_ceiling;
      ++bin_counts(y, x);
    }
  }

  Eigen::MatrixXd result(bin_counts.rows(), bin_counts.cols());
  // Avoid "log(0)".
  bin_counts += Eigen::MatrixXi::Ones(num_y_bins, num_x_bins);
  for (int i = 0; i < bin_counts.size(); ++i) {
    result(i) = std::log(static_cast<double>(bin_counts(i)));
  }
  return result;
}

Eigen::MatrixXd downsampleWithIndexAsX(
    const std::vector<std::vector<size_t>>& y_values, const size_t num_x_bins,
    const size_t num_y_bins) {
  CHECK_GT(num_x_bins, 0u);
  CHECK_GT(num_y_bins, 0u);
  const size_t x_ceiling = y_values.size();
  CHECK_GT(x_ceiling, 0u);
  size_t y_max = 0u;
  for (const std::vector<size_t>& y_of_x : y_values) {
    for (const size_t y : y_of_x) {
      if (y > y_max) {
        y_max = y;
      }
    }
  }
  const size_t y_ceiling = y_max + 1;

  Eigen::MatrixXi bin_counts(num_y_bins, num_x_bins);
  bin_counts.setZero();
  for (size_t x_index = 0u; x_index < x_ceiling; ++x_index) {
    const int x = x_index * num_x_bins / x_ceiling;
    for (size_t y_index = 0u; y_index < y_values[x_index].size(); ++y_index) {
      const int y = y_values[x_index][y_index] * num_y_bins / y_ceiling;
      ++bin_counts(y, x);
    }
  }

  const double bin_density = (static_cast<double>(x_ceiling) / num_x_bins) *
                             (static_cast<double>(y_ceiling) / num_y_bins);

  return bin_counts.cast<double>() / bin_density;
}

}  // namespace histograms

}  // namespace common
